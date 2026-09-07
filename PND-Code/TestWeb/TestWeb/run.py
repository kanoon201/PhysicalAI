"""
One-command runner
===================
Everything in a single process: finds the ESP32-CAM on the network,
opens its MJPEG stream, and shows the cone detection with live HSV
trackbars — so tuning and testing happen in the same window instead of
being three separate commands.

    python run.py                          # auto-find the camera
    python run.py --esp-ip 192.168.8.191   # skip the scan if you know the IP
    python run.py --source esp32           # robot firmware (UDP) instead

Controls:
    trackbars : adjust until the Mask window shows ONLY the cones white
    's'       : save the current HSV values to cone_config.json
    'q'       : quit

The steering readout on the video is what the car WOULD be commanded to
do — nothing is sent to any motor from here.
"""

import argparse
import socket
import sys
import time
from concurrent.futures import ThreadPoolExecutor

import cv2

from camera_control import RESOLUTIONS, set_resolution
from cone_detection import ConeLaneKeeper, CONFIG_PATH, save_config
from video_source import make_stream_source

STREAM_PORT = 81
WEB_PORT = 80
FIRST_FRAME_TIMEOUT = 15.0


# ============================================================
# CAMERA DISCOVERY
# ============================================================

def local_subnet_prefix():
    """Best guess at the /24 this PC is on. The UDP 'connect' sends no
    packets — it just makes the OS pick the outbound interface."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        return None
    finally:
        s.close()
    return ip.rsplit(".", 1)[0]


def _probe(ip, timeout=0.4):
    """A host is the camera if its stream port is open. Plain socket check
    only — actually opening /stream would consume the camera's single
    client slot, and we'd have to drop it again a moment later."""
    try:
        with socket.create_connection((ip, STREAM_PORT), timeout=timeout):
            return ip
    except Exception:
        return None


def discover_camera(prefix, workers=128):
    print(f"Scanning {prefix}.1-254 for an ESP32-CAM on port {STREAM_PORT} ...")
    hosts = [f"{prefix}.{i}" for i in range(1, 255)]
    found = []
    with ThreadPoolExecutor(max_workers=workers) as pool:
        for result in pool.map(_probe, hosts):
            if result:
                found.append(result)
    return found


def resolve_camera_ip(explicit_ip):
    if explicit_ip:
        return explicit_ip

    prefix = local_subnet_prefix()
    if not prefix:
        print("Could not work out this PC's subnet — pass --esp-ip manually.")
        return None

    found = discover_camera(prefix)
    if not found:
        print("No camera found. Check that:")
        print("  - the ESP32-CAM is powered and finished booting")
        print("  - it joined the same WiFi as this PC (SSID 'PND')")
        print("  - CameraWebServer.ino is the sketch that's flashed")
        print("Otherwise read the IP off the Serial Monitor and pass --esp-ip.")
        return None
    if len(found) > 1:
        print(f"Multiple candidates found: {', '.join(found)} — using {found[0]}.")
        print("Pass --esp-ip if that's the wrong one.")
    print(f"Found camera at {found[0]}")
    return found[0]


# ============================================================
# UI
# ============================================================

def _nothing(_):
    pass


def build_trackbars(window, cfg):
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 420, 300)
    lo, hi = cfg["hsv_range1_lower"], cfg["hsv_range1_upper"]
    cv2.createTrackbar("H min", window, lo[0], 179, _nothing)
    cv2.createTrackbar("S min", window, lo[1], 255, _nothing)
    cv2.createTrackbar("V min", window, lo[2], 255, _nothing)
    cv2.createTrackbar("H max", window, hi[0], 179, _nothing)
    cv2.createTrackbar("S max", window, hi[1], 255, _nothing)
    cv2.createTrackbar("V max", window, hi[2], 255, _nothing)
    # Cheap way to kill speckle noise without reopening the config file.
    # Upper bound is sized for VGA — a cone covers ~4x the pixels it does
    # at QVGA, so a range that felt generous there is not enough here.
    cv2.createTrackbar("min area", window, int(cfg["min_contour_area"]), 8000, _nothing)


def read_trackbars(window, cfg):
    cfg["hsv_range1_lower"] = [cv2.getTrackbarPos("H min", window),
                               cv2.getTrackbarPos("S min", window),
                               cv2.getTrackbarPos("V min", window)]
    cfg["hsv_range1_upper"] = [cv2.getTrackbarPos("H max", window),
                               cv2.getTrackbarPos("S max", window),
                               cv2.getTrackbarPos("V max", window)]
    cfg["min_contour_area"] = max(1, cv2.getTrackbarPos("min area", window))


# ============================================================
# MAIN
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="Find the ESP32-CAM, stream it, and run cone detection — all in one command")
    parser.add_argument("--esp-ip", default=None, help="Camera IP. Omit to scan the local subnet for it.")
    parser.add_argument("--source", default="webserver", help="'webserver' (CameraWebServer MJPEG) or 'esp32' (sketch_jul13a UDP)")
    parser.add_argument("--config", default=CONFIG_PATH, help="Path to config JSON")
    parser.add_argument("--framesize", default="vga", choices=sorted(RESOLUTIONS),
                        help="Resolution to put the camera in (default vga, to match the robot firmware)")
    args = parser.parse_args()

    # The UDP firmware can broadcast-discover itself, so a scan is only
    # worth doing for the HTTP one.
    esp_ip = args.esp_ip
    if args.source.lower() == "webserver":
        esp_ip = resolve_camera_ip(esp_ip)
        if not esp_ip:
            return 1

        # Do this before opening the stream: the camera serves one client,
        # and /capture (used to verify the change actually took) can't run
        # while the MJPEG stream holds the slot.
        want_w, want_h = RESOLUTIONS[args.framesize][0]
        actual = set_resolution(esp_ip, args.framesize)
        if actual is None:
            print("Camera found but not answering on port 80 — leaving the resolution as-is.")
        elif actual != (want_w, want_h):
            print(f"Camera refused {args.framesize.upper()} ({want_w}x{want_h}); "
                  f"running at {actual[0]}x{actual[1]} instead.")
        else:
            print(f"Camera set to {args.framesize.upper()} ({want_w}x{want_h}).")

    keeper = ConeLaneKeeper(config_path=args.config)
    src = make_stream_source(args.source, esp_ip=esp_ip)
    src.start()

    control_win = "Controls (s=save, q=quit)"
    build_trackbars(control_win, keeper.cfg)

    print("Waiting for the first frame ...")
    started_at = time.time()
    warned = False
    got_first_frame = False

    try:
        while True:
            frame = src.get_frame()
            if frame is None:
                if not warned and time.time() - started_at > FIRST_FRAME_TIMEOUT:
                    print(f"Still no frames after {FIRST_FRAME_TIMEOUT:.0f}s. The ESP32-CAM only "
                          "serves ONE stream client — close any browser tab showing the camera page.")
                    warned = True
                if cv2.waitKey(100) & 0xFF == ord('q'):
                    break
                continue

            if not got_first_frame:
                got_first_frame = True
                print(f"Streaming {frame.shape[1]}x{frame.shape[0]}. "
                      "Adjust the trackbars until only the cones are white in Mask.")

            read_trackbars(control_win, keeper.cfg)
            steer, throttle, debug_frame, mask = keeper.process(frame)

            cv2.imshow("Cone Detection", debug_frame)
            cv2.imshow("Mask", mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('s'):
                save_config(keeper.cfg, args.config)
                print(f"Saved to {args.config}: lower={keeper.cfg['hsv_range1_lower']} "
                      f"upper={keeper.cfg['hsv_range1_upper']} min_area={keeper.cfg['min_contour_area']}")
    finally:
        src.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
