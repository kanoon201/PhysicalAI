"""
Blue cone detection (color + shape, no steering)  -- V4
==================================================
Finds BLUE traffic cones in a BGR frame and hands back where they are.

V4: what changed and why
------------------------
Three things were seen on the real track that V3 could not handle:

1. LEFT AND RIGHT LIT DIFFERENTLY. One row of cones sat by a window, the
   other in shade. A cone in the light is BRIGHT and WASHED OUT (high V,
   low S); the same cone in shade is DARK and DEEP (low V, high S). One
   HSV box cannot hold both without also holding the floor. V4 uses a
   LIST of HSV ranges (`color_ranges`) OR-ed together: one for "lit",
   one for "shaded", add more if a room needs them.

2. UNEVEN LIGHT ACROSS THE FRAME. Before thresholding, the V channel is
   run through CLAHE (`light_normalize`), which equalises brightness
   tile by tile -- the dim half of the frame is lifted, the bright half
   is pulled down, and the HSV ranges then see roughly the same cone
   on both sides.

3. ROWS OF CONES MERGING INTO ONE BLOB. Cones placed close together
   touch in the mask, becoming a wide landscape blob that the aspect
   filter threw away -- so a whole side of the lane vanished exactly
   when it had the most cones. V4 no longer discards a too-wide blob; it
   SPLITS it along the FLOOR LINE. Cones stand at different distances,
   so the bottom edge of the blob is a staircase -- one step per cone
   base, nearest cone lowest. Every step becomes a cone. (Column-count
   and distance-transform splitting were both tried first and failed:
   from a camera this low, cones in a row overlap into one solid mass
   with no gaps between them, but their bases never overlap.)
Nothing here decides how to drive — that's a later step on purpose, so
the detection can be judged on its own.

Why blue rather than orange:
  - Orange sits on the H=0/179 seam, so it needs two inRange passes
    stitched together. Blue sits mid-range (~100) and takes one.
  - The things that ruin an orange detector indoors — skin, cardboard,
    wooden floors and desks — all live at H 5-25, right on top of an
    orange cone. Almost nothing indoors is saturated blue.

Color alone is not enough, though, so every blob has to survive four
cheap filters (area, aspect, solidity, ROI) before it counts as a cone.
See DEFAULT_CONFIG below for what each one throws away.

    python cone_detection.py --source webserver --esp-ip 192.168.8.191
    python cone_detection.py --source esp32
    python cone_detection.py --source esp32_samples/esp32_sample_123.jpg

Controls:
    trackbars : adjust until the Mask window shows ONLY the cones white
                "range #" picks which HSV box (shade / lit) you are editing
    'w'       : write the current values to cone_config.json
    'q'       : quit

Lock the camera's AWB/exposure BEFORE tuning (run.py does it for you, or
call camera_control.lock_image_settings). HSV values tuned while auto
white balance is running describe the light, not the cone.

Usage as a module:
    from cone_detection import ConeDetector
    det = ConeDetector()
    cones, mask, debug = det.detect(bgr_frame)
    for c in cones:
        print(c.cx, c.cy, c.area, c.bbox)
"""

import argparse
import json
import os
import sys

import cv2
import numpy as np

CONFIG_PATH = "cone_config.json"

# Starting point for a blue cone under white indoor light. These are a
# place to begin tuning, not an answer — every camera and every room
# shifts them.
DEFAULT_CONFIG = {
    # A LIST of HSV boxes, OR-ed together. Hue 90-130 is blue in every
    # one; what differs is how much S and V a cone shows under the light
    # it happens to be standing in:
    #   shade : dark, saturated   -> low V allowed, S must be high
    #   lit   : bright, washed out -> V must be high, S allowed lower
    # A grey floor is low S at every V, so the "lit" box is safe as long
    # as its S floor stays above ~35. Tune each box on the trackbars
    # (the "range #" slider picks which one you are editing).
    "color_ranges": [
        {"name": "shade", "lower": [90, 80, 40],  "upper": [130, 255, 255]},
        {"name": "lit",   "lower": [90, 40, 150], "upper": [130, 255, 255]},
    ],

    # Brightness equalisation before thresholding. "clahe" runs adaptive
    # histogram equalisation on the V channel in tiles, so a window on
    # one side of the room stops making that side's cones look like a
    # different colour. "none" = raw camera V.
    "light_normalize": "clahe",
    "clahe_clip": 2.0,
    "clahe_tiles": 8,

    # --- FILTERS ---
    # Area is in ABSOLUTE PIXELS, so it is tied to the resolution the
    # camera happens to be sending. 1500 px is ~0.5% of a VGA frame but
    # ~2% of QVGA -- re-tune it if you change --framesize.
    "min_contour_area": 1500,
    "max_contour_area": 60000,

    # width / height of the bounding box. A cone is taller than it is
    # wide, so anything square or landscape is something else. This is
    # what throws out jeans, chair backs and monitors.
    "min_aspect_ratio": 0.2,
    "max_aspect_ratio": 1.3,

    # contour area / convex hull area. A cone is a solid, convex-ish blob
    # (~0.85+). Ragged noise and things seen through gaps score low.
    "min_solidity": 0.75,

    # Ignore everything above this fraction of the frame height. Cones
    # sit on the floor; the sky through a window, monitors, and people's
    # clothing are all above the horizon. Cheapest filter here by far --
    # set it to 0.0 to disable.
    "roi_top_ratio": 0.4,

    # Kernel for the open/close pass. Blue suffers more than most colors
    # from JPEG chroma subsampling, so distant cones arrive speckled and
    # need the CLOSE to knit them back together.
    "morph_kernel_size": 5,

    # A blob wider than max_aspect_ratio (or bigger than max area) is
    # probably a ROW of touching cones, not junk. Split it where the
    # blob's bottom edge (the floor line) jumps by at least this many
    # pixels between neighbouring columns -- each jump is the step from
    # one cone's base to the next one's. 0 disables splitting.
    "split_step_px": 4,
    # A piece narrower than this many pixels is a spike, not a cone.
    "split_min_width_px": 8,
}


def _migrate_v3(cfg):
    """A V3 cone_config.json has one hsv_lower/hsv_upper pair. Turn it
    into the first range so an old tuning is not lost, and add the "lit"
    range next to it."""
    if "hsv_lower" in cfg and "hsv_upper" in cfg:
        lo, hi = list(cfg.pop("hsv_lower")), list(cfg.pop("hsv_upper"))
        cfg["color_ranges"] = [
            {"name": "shade", "lower": lo, "upper": hi},
            {"name": "lit", "lower": [lo[0], 40, 150], "upper": hi},
        ]
        print("Migrated V3 hsv_lower/hsv_upper into color_ranges[0] ('shade').")
    return cfg


def load_config(path=CONFIG_PATH):
    """Missing or partial config falls back to the defaults key by key, so
    an old file doesn't crash the detector when a new filter is added."""
    cfg = dict(DEFAULT_CONFIG)
    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                cfg.update(_migrate_v3(json.load(f)))
        except (json.JSONDecodeError, OSError) as e:
            print(f"Could not read {path} ({e}) — using defaults.")
    return cfg


def save_config(cfg, path=CONFIG_PATH):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)


# ============================================================
# DETECTION
# ============================================================

class Cone:
    __slots__ = ("cx", "cy", "area", "bbox")

    def __init__(self, cx, cy, area, bbox):
        self.cx = cx
        self.cy = cy
        self.area = area
        self.bbox = bbox  # (x, y, w, h)

    def __repr__(self):
        return f"Cone(cx={self.cx}, cy={self.cy}, area={self.area:.0f})"


def normalize_light(hsv, cfg):
    """Equalise the V channel so the two sides of the frame are lit alike.
    H and S are left alone: CLAHE on them would invent colours."""
    mode = str(cfg.get("light_normalize", "none")).lower()
    if mode != "clahe":
        return hsv
    clip = max(0.1, float(cfg.get("clahe_clip", 2.0)))
    tiles = max(1, int(cfg.get("clahe_tiles", 8)))
    clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(tiles, tiles))
    h, s_, v = cv2.split(hsv)
    return cv2.merge((h, s_, clahe.apply(v)))


def color_mask(hsv, cfg):
    """OR of every range in color_ranges. One box per lighting condition."""
    mask = None
    for r in cfg["color_ranges"]:
        m = cv2.inRange(hsv, np.array(r["lower"], dtype=np.uint8),
                        np.array(r["upper"], dtype=np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    if mask is None:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    return mask


def build_mask(bgr_frame, cfg):
    """BGR in, binary mask out. Takes BGR straight from cv2.imdecode -- not
    an RGB frame built for display, which swaps the red and blue channels
    and so turns a blue cone into an orange one."""
    hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
    hsv = normalize_light(hsv, cfg)
    mask = color_mask(hsv, cfg)

    k = max(1, int(cfg["morph_kernel_size"]))
    kernel = np.ones((k, k), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # kill speckle
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # close JPEG gaps

    # Blank the ROI in the mask rather than cropping the frame: contour
    # coordinates stay in full-frame space, so nothing downstream has to
    # remember to add an offset back.
    top = int(mask.shape[0] * float(cfg["roi_top_ratio"]))
    if top > 0:
        mask[:top, :] = 0
    return mask


def _cone_from_patch(patch, x0, y0):
    """Build a Cone from a binary sub-image whose top-left is (x0, y0) in
    frame coordinates. Returns None for an empty patch."""
    ys, xs = np.nonzero(patch)
    if len(xs) == 0:
        return None
    area = float(len(xs))
    x, y = int(xs.min()), int(ys.min())
    w, h = int(xs.max() - x + 1), int(ys.max() - y + 1)
    return Cone(x0 + int(xs.mean()), y0 + int(ys.mean()), area, (x0 + x, y0 + y, w, h))


def split_wide_blob(mask, bbox, cfg):
    """A row of touching cones is one wide blob. Walk its bottom edge (the
    lowest white pixel in each column): within one cone's base it changes
    by a pixel or so per column; between two cones it JUMPS, because the
    next cone stands further away and its base sits higher in the frame.
    Cut at every jump of split_step_px or more. Each run of columns is a
    cone: cx = middle of the run, bottom = its floor line."""
    x, y, w, h = bbox
    patch = mask[y:y + h, x:x + w] > 0
    has = patch.any(axis=0)
    # Lowest white row per column (-1 for an empty column).
    bottom = np.where(has, h - 1 - patch[::-1].argmax(axis=0), -1)

    step = max(1, int(cfg.get("split_step_px", 4)))
    min_w = max(1, int(cfg.get("split_min_width_px", 8)))

    cuts = []
    prev = None
    for i in range(w):
        if bottom[i] < 0:
            continue
        if prev is not None and abs(int(bottom[i]) - int(bottom[prev])) >= step:
            cuts.append(i)
        prev = i

    edges = [0] + cuts + [w]
    pieces = []
    for a_, b_ in zip(edges[:-1], edges[1:]):
        if b_ - a_ < min_w:
            continue
        sub = np.zeros_like(patch)
        sub[:, a_:b_] = patch[:, a_:b_]
        c = _cone_from_patch(sub.astype(np.uint8), x, y)
        if c is not None:
            pieces.append(c)
    return pieces


def _passes_shape(cone, cfg, min_area):
    """For a piece cut out of a row. Area only: a far cone in a dense row
    is a narrow sliver of columns beside its taller, nearer neighbour, so
    the aspect test that guards single blobs would throw away exactly the
    cones the split just recovered."""
    x, y, w, h = cone.bbox
    return h > 0 and min_area <= cone.area <= cfg["max_contour_area"]


def find_cones(mask, cfg):
    """Every blob in the mask that survives the filters, biggest first
    (biggest is usually nearest). A blob that fails only for being too
    WIDE or too BIG is split into a row of cones instead of dropped."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cones = []
    min_area = float(cfg["min_contour_area"])
    split_on = int(cfg.get("split_step_px", 0)) > 0
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue

        x, y, w, h = cv2.boundingRect(c)
        if h == 0:
            continue
        aspect = w / float(h)

        too_wide = aspect > cfg["max_aspect_ratio"]
        too_big = area > cfg["max_contour_area"]
        if too_wide or too_big:
            if not split_on:
                continue
            # Pieces of a row are each smaller than the row; let a piece
            # through at half the usual minimum so far cones in a dense
            # row are not lost to the very filter that found the row.
            for piece in split_wide_blob(mask, (x, y, w, h), cfg):
                if _passes_shape(piece, cfg, min_area * 0.5):
                    cones.append(piece)
            continue

        if aspect < cfg["min_aspect_ratio"]:
            continue

        hull_area = cv2.contourArea(cv2.convexHull(c))
        if hull_area <= 0 or (area / hull_area) < cfg["min_solidity"]:
            continue

        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cones.append(Cone(int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]),
                          area, (x, y, w, h)))

    cones.sort(key=lambda c: -c.area)
    return cones


class ConeDetector:
    def __init__(self, config_path=CONFIG_PATH):
        self.cfg = load_config(config_path)

    def detect(self, bgr_frame, draw=True):
        """Returns (cones, mask, debug_frame). cones is largest-first;
        debug_frame is None when draw=False."""
        mask = build_mask(bgr_frame, self.cfg)
        cones = find_cones(mask, self.cfg)
        debug = self.draw(bgr_frame, cones) if draw else None
        return cones, mask, debug

    def draw(self, bgr_frame, cones):
        out = bgr_frame.copy()
        h, w = out.shape[:2]

        top = int(h * float(self.cfg["roi_top_ratio"]))
        if top > 0:
            # Dim the ignored band so it's obvious why a cone up there
            # isn't being found.
            out[:top, :] = (out[:top, :] * 0.35).astype(np.uint8)
            cv2.line(out, (0, top), (w, top), (80, 80, 80), 1)

        for i, c in enumerate(cones):
            x, y, cw, ch = c.bbox
            # Largest cone in green, the rest in yellow.
            color = (0, 255, 0) if i == 0 else (0, 255, 255)
            cv2.rectangle(out, (x, y), (x + cw, y + ch), color, 2)
            cv2.circle(out, (c.cx, c.cy), 4, color, -1)
            cv2.putText(out, f"{c.area:.0f}", (x, max(12, y - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        cv2.putText(out, f"cones: {len(cones)}", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return out


# ============================================================
# TUNING UI
# ============================================================

def _nothing(_):
    pass


TRACKBARS = (
    ("H min", "lower", 0, 179),
    ("S min", "lower", 1, 255),
    ("V min", "lower", 2, 255),
    ("H max", "upper", 0, 179),
    ("S max", "upper", 1, 255),
    ("V max", "upper", 2, 255),
)

# Which color range the six HSV sliders are showing, per window.
_editing = {}


def _load_range_into_trackbars(window, r):
    for name, key, idx, _maximum in TRACKBARS:
        cv2.setTrackbarPos(name, window, int(r[key][idx]))


def build_trackbars(window, cfg):
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 420, 460)
    ranges = cfg["color_ranges"]
    # "range #" picks which box the six HSV sliders edit. Moving it
    # reloads the sliders from that box; moving a slider writes into it.
    cv2.createTrackbar("range #", window, 0, max(1, len(ranges) - 1), _nothing)
    for name, key, idx, maximum in TRACKBARS:
        cv2.createTrackbar(name, window, int(ranges[0][key][idx]), maximum, _nothing)
    _editing[window] = 0
    cv2.createTrackbar("light norm", window,
                       1 if str(cfg.get("light_normalize", "none")).lower() == "clahe" else 0,
                       1, _nothing)
    cv2.createTrackbar("clahe clip x10", window, int(float(cfg.get("clahe_clip", 2.0)) * 10), 80, _nothing)
    cv2.createTrackbar("split step px", window, int(cfg.get("split_step_px", 4)), 20, _nothing)
    # Upper bound sized for VGA -- a cone covers ~4x the pixels it does at
    # QVGA, so a range that felt generous there is not enough here.
    cv2.createTrackbar("min area", window, int(cfg["min_contour_area"]), 8000, _nothing)
    cv2.createTrackbar("solidity %", window, int(cfg["min_solidity"] * 100), 100, _nothing)
    cv2.createTrackbar("roi top %", window, int(cfg["roi_top_ratio"] * 100), 90, _nothing)


def read_trackbars(window, cfg):
    ranges = cfg["color_ranges"]
    sel = min(cv2.getTrackbarPos("range #", window), len(ranges) - 1)
    if sel != _editing.get(window, 0):
        # Selector moved: show that range's values, don't overwrite them.
        _editing[window] = sel
        _load_range_into_trackbars(window, ranges[sel])
    else:
        for name, key, idx, _maximum in TRACKBARS:
            ranges[sel][key][idx] = cv2.getTrackbarPos(name, window)

    cfg["light_normalize"] = "clahe" if cv2.getTrackbarPos("light norm", window) else "none"
    cfg["clahe_clip"] = max(0.1, cv2.getTrackbarPos("clahe clip x10", window) / 10.0)
    cfg["split_step_px"] = cv2.getTrackbarPos("split step px", window)
    cfg["min_contour_area"] = max(1, cv2.getTrackbarPos("min area", window))
    cfg["min_solidity"] = cv2.getTrackbarPos("solidity %", window) / 100.0
    cfg["roi_top_ratio"] = cv2.getTrackbarPos("roi top %", window) / 100.0


def print_saved(cfg, path):
    boxes = "  ".join(f"{r.get('name', i)}={r['lower']}..{r['upper']}"
                      for i, r in enumerate(cfg["color_ranges"]))
    print(f"Saved to {path}: {boxes}  light={cfg.get('light_normalize')}  "
          f"split_step={cfg.get('split_step_px', 0)}px  "
          f"min_area={cfg['min_contour_area']}  solidity={cfg['min_solidity']:.2f}  "
          f"roi_top={cfg['roi_top_ratio']:.2f}")


# ============================================================
# STANDALONE
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="Detect blue cones in a live feed or a still image")
    parser.add_argument("--source", default="webserver",
                        help="'webserver', 'esp32', an http:// stream URL, or a path to an image")
    parser.add_argument("--esp-ip", default=None, help="Camera IP (needed for --source webserver)")
    parser.add_argument("--config", default=CONFIG_PATH, help="Path to config JSON")
    args = parser.parse_args()

    from video_source import make_stream_source

    det = ConeDetector(config_path=args.config)
    control_win = "Controls (w=write config, q=quit)"
    build_trackbars(control_win, det.cfg)

    src = make_stream_source(args.source, esp_ip=args.esp_ip)

    # Not a live source — treat it as an image on disk. Handy for
    # re-tuning against saved frames with no camera plugged in.
    if src is None:
        still = cv2.imread(args.source)
        if still is None:
            print(f"Not a live source and not a readable image: {args.source}")
            return 1
        try:
            while True:
                read_trackbars(control_win, det.cfg)
                _cones, mask, debug = det.detect(still)
                cv2.imshow("Cone Detection", debug)
                cv2.imshow("Mask", mask)
                key = cv2.waitKey(30) & 0xFF
                if key == ord('q'):
                    break
                if key == ord('w'):
                    save_config(det.cfg, args.config)
                    print_saved(det.cfg, args.config)
        finally:
            cv2.destroyAllWindows()
        return 0

    src.start()
    print("Waiting for the first frame ... ('w' writes the config, 'q' quits)")
    try:
        while True:
            frame = src.get_frame()
            if frame is None:
                if cv2.waitKey(100) & 0xFF == ord('q'):
                    break
                continue

            read_trackbars(control_win, det.cfg)
            _cones, mask, debug = det.detect(frame)
            cv2.imshow("Cone Detection", debug)
            cv2.imshow("Mask", mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('w'):
                save_config(det.cfg, args.config)
                print_saved(det.cfg, args.config)
    finally:
        src.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
