"""
Phase 1: Rule-based Cone Detection + Lane-Keeping Steering
============================================================
Detects cones on the left/right of the frame using HSV color
thresholding, and computes a steering + throttle command to keep
the car centered between them.

Standalone test (no robot needed — use a webcam, video file, or photo):
    python cone_detection.py --source 0
    python cone_detection.py --source my_video.mp4
    python cone_detection.py --source cone_photo.jpg

Use as a module (integrate into the robot controller):
    from cone_detection import ConeLaneKeeper
    keeper = ConeLaneKeeper()
    steer, throttle, debug_frame, mask = keeper.process(bgr_frame)

Run hsv_calibrator.py FIRST to tune hsv thresholds for your actual
cones/lighting before trusting this module's output.
"""

import cv2
import numpy as np
import json
import os
import argparse

# Only needed for --source esp32 in the standalone test below; the
# ConeLaneKeeper class itself has no dependency on this.
try:
    from esp32_video_source import ESP32VideoSource
except ImportError:
    ESP32VideoSource = None


# ============================================================
# CONFIGURATION
# ============================================================

CONFIG_PATH = "cone_config.json"

DEFAULT_CONFIG = {
    # HSV thresholds. range2 is only needed for colors that wrap around
    # the hue circle (e.g. red, which spans both ends of H). For a
    # single-color cone (typical orange/yellow), leave use_range2 False.
    "hsv_range1_lower": [5, 100, 100],
    "hsv_range1_upper": [20, 255, 255],
    "hsv_range2_lower": [0, 0, 0],
    "hsv_range2_upper": [0, 0, 0],
    "use_range2": False,

    # Contour filtering — tune these to your cone's real pixel size/shape
    "min_contour_area": 150,
    "max_contour_area": 50000,
    "min_aspect_ratio": 0.2,   # width / height (cones are usually taller than wide)
    "max_aspect_ratio": 1.3,

    # Steering (simple P-controller on pixel offset from frame center)
    "steer_gain": 1.2,
    "steer_deadzone_px": 15,   # ignore tiny offsets to avoid jitter/oscillation
    "max_steer": 1.0,
    "base_throttle": 0.35,     # constant forward speed while lane is visible
    "slow_down_no_cone_frames": 5,  # consecutive lost-lane frames before stopping

    # Used only when just ONE side's cone is visible: assumes the target
    # point sits this fraction of the frame width to the side, away from
    # the visible cone. THIS NEEDS FIELD-TUNING on your actual camera —
    # it depends on mounting height/angle and real cone spacing, which
    # can't be guessed from code alone. If the car turns the wrong way
    # when only one cone is visible, this value is too small (raise it).
    "single_side_offset_ratio": 0.35,

    # Morphology (cleans up the raw threshold mask)
    "morph_kernel_size": 5,
}


def load_config(path=CONFIG_PATH):
    if os.path.exists(path):
        with open(path, "r") as f:
            user_cfg = json.load(f)
        cfg = DEFAULT_CONFIG.copy()
        cfg.update(user_cfg)
        return cfg
    return DEFAULT_CONFIG.copy()


def save_config(cfg, path=CONFIG_PATH):
    with open(path, "w") as f:
        json.dump(cfg, f, indent=2)


# ============================================================
# CORE DETECTION
# ============================================================

class Cone:
    __slots__ = ("cx", "cy", "area", "bbox")

    def __init__(self, cx, cy, area, bbox):
        self.cx = cx
        self.cy = cy
        self.area = area
        self.bbox = bbox  # (x, y, w, h)


def detect_cone_mask(hsv_frame, cfg):
    lower1 = np.array(cfg["hsv_range1_lower"], dtype=np.uint8)
    upper1 = np.array(cfg["hsv_range1_upper"], dtype=np.uint8)
    mask = cv2.inRange(hsv_frame, lower1, upper1)

    if cfg.get("use_range2"):
        lower2 = np.array(cfg["hsv_range2_lower"], dtype=np.uint8)
        upper2 = np.array(cfg["hsv_range2_upper"], dtype=np.uint8)
        mask2 = cv2.inRange(hsv_frame, lower2, upper2)
        mask = cv2.bitwise_or(mask, mask2)

    k = max(1, cfg["morph_kernel_size"])
    kernel = np.ones((k, k), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def find_cones(mask, cfg):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cones = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < cfg["min_contour_area"] or area > cfg["max_contour_area"]:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if h == 0:
            continue
        aspect = w / float(h)
        if aspect < cfg["min_aspect_ratio"] or aspect > cfg["max_aspect_ratio"]:
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        cones.append(Cone(cx, cy, area, (x, y, w, h)))
    return cones


def separate_left_right(cones, frame_width):
    """Split detected cones into left-lane / right-lane groups based on
    position relative to the frame's horizontal center. Within each side,
    sort by area descending — the largest (usually closest) cone is used
    as the reference for steering."""
    mid_x = frame_width / 2.0
    left = [c for c in cones if c.cx < mid_x]
    right = [c for c in cones if c.cx >= mid_x]
    left.sort(key=lambda c: -c.area)
    right.sort(key=lambda c: -c.area)
    return left, right


# ============================================================
# STEERING CONTROLLER
# ============================================================

class ConeLaneKeeper:
    def __init__(self, config_path=CONFIG_PATH):
        self.cfg = load_config(config_path)
        self.no_cone_counter = 0

    def process(self, bgr_frame):
        """
        bgr_frame: a BGR frame (e.g. decoded straight from the ESP32-CAM
                   JPEG via cv2.imdecode — do NOT pre-convert to RGB).
        Returns: (steer, throttle, debug_frame, mask)
          steer    : -1.0 (full left) .. +1.0 (full right)
          throttle : 0.0 .. 1.0
          debug_frame : BGR frame with detections drawn on it
          mask     : the binary threshold mask (useful for tuning/logging)
        """
        h, w = bgr_frame.shape[:2]
        hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
        mask = detect_cone_mask(hsv, self.cfg)
        cones = find_cones(mask, self.cfg)
        left, right = separate_left_right(cones, w)

        frame_center = w / 2.0
        target_x = frame_center
        have_target = False

        if left and right:
            # Both lane sides visible: aim for the midpoint between the
            # closest left cone and the closest right cone.
            target_x = (left[0].cx + right[0].cx) / 2.0
            have_target = True
        elif left and not right:
            # Only the left boundary is visible — assume the lane
            # continues and steer away from it (to the right).
            target_x = left[0].cx + w * self.cfg["single_side_offset_ratio"]
            have_target = True
        elif right and not left:
            target_x = right[0].cx - w * self.cfg["single_side_offset_ratio"]
            have_target = True

        if have_target:
            self.no_cone_counter = 0
        else:
            self.no_cone_counter += 1

        offset = target_x - frame_center
        if abs(offset) < self.cfg["steer_deadzone_px"]:
            steer = 0.0
        else:
            norm_offset = offset / (w / 2.0)   # normalize to roughly -1..1
            steer = norm_offset * self.cfg["steer_gain"]
            steer = max(-self.cfg["max_steer"], min(self.cfg["max_steer"], steer))

        if self.no_cone_counter >= self.cfg["slow_down_no_cone_frames"]:
            throttle = 0.0  # lane lost — stop rather than guess and crash
        else:
            throttle = self.cfg["base_throttle"]

        debug_frame = self._draw_debug(bgr_frame, left, right, target_x,
                                        frame_center, steer, throttle, have_target)
        return steer, throttle, debug_frame, mask

    @staticmethod
    def _draw_debug(frame, left, right, target_x, frame_center, steer, throttle, have_target):
        debug_frame = frame.copy()
        h = debug_frame.shape[0]
        cv2.line(debug_frame, (int(frame_center), 0), (int(frame_center), h), (255, 255, 255), 1)
        for c in left:
            x, y, w_, h_ = c.bbox
            cv2.rectangle(debug_frame, (x, y), (x + w_, y + h_), (255, 100, 0), 2)  # blue-ish = left
        for c in right:
            x, y, w_, h_ = c.bbox
            cv2.rectangle(debug_frame, (x, y), (x + w_, y + h_), (0, 100, 255), 2)  # orange = right
        if have_target:
            cv2.circle(debug_frame, (int(target_x), int(h * 0.6)), 8, (0, 255, 0), -1)
        status = f"steer={steer:+.2f} throttle={throttle:.2f} L={len(left)} R={len(right)}"
        cv2.putText(debug_frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return debug_frame


# ============================================================
# STANDALONE TEST / DEMO
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="Test cone detection + lane-keeping on a video source")
    parser.add_argument("--source", default="0", help="Camera index (e.g. 0), video file path, image file path, or 'esp32' for the live ESP32-CAM feed")
    parser.add_argument("--esp-ip", default=None, help="ESP32-CAM IP address (only used with --source esp32; omit to broadcast-discover instead)")
    parser.add_argument("--config", default=CONFIG_PATH, help="Path to config JSON")
    args = parser.parse_args()

    keeper = ConeLaneKeeper(config_path=args.config)
    source = args.source
    is_image = source.lower().endswith((".jpg", ".jpeg", ".png"))
    is_esp32 = source.lower() == "esp32"

    if is_esp32:
        if ESP32VideoSource is None:
            print("esp32_video_source.py not found — put it in the same folder as this script.")
            return
        esp_src = ESP32VideoSource(esp_ip=args.esp_ip)
        esp_src.start()
        print("Connecting to ESP32-CAM... (waiting for first frame — make sure it's powered on)")
        print("Press 'q' to quit.")
        try:
            while True:
                frame = esp_src.get_frame()
                if frame is None:
                    if cv2.waitKey(100) & 0xFF == ord('q'):
                        break
                    continue
                steer, throttle, debug_frame, mask = keeper.process(frame)
                cv2.imshow("Cone Detection", debug_frame)
                cv2.imshow("Mask", mask)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            esp_src.stop()
            cv2.destroyAllWindows()
        return

    if is_image:
        frame = cv2.imread(source)
        if frame is None:
            print(f"Cannot read image: {source}")
            return
        steer, throttle, debug_frame, mask = keeper.process(frame)
        print(f"steer={steer:.3f} throttle={throttle:.3f}")
        cv2.imshow("Cone Detection", debug_frame)
        cv2.imshow("Mask", mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return

    cap_source = int(source) if source.isdigit() else source
    cap = cv2.VideoCapture(cap_source)
    if not cap.isOpened():
        print(f"Cannot open source: {source}")
        return

    print("Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        steer, throttle, debug_frame, mask = keeper.process(frame)
        cv2.imshow("Cone Detection", debug_frame)
        cv2.imshow("Mask", mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
