"""V3 entry point: find the camera, set it up, show the video.

    python run.py                          # scan the subnet, then stream
    python run.py --esp-ip 192.168.1.42
    python run.py --framesize qvga --quality 16 --ae-level -1
    python run.py --esp-ip 192.168.1.42 --bench 12    # measure, no window

Keys: v / h flip, s save a frame, q quit.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

import cv2

import camera
from mjpeg_source import MjpegSource

WINDOW = "V3 - ESP32-CAM"
SAVE_DIR = Path("captures")


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="ESP32-CAM video for V3")
    p.add_argument("--esp-ip", help="camera IP; omit to scan the subnet")
    p.add_argument("--subnet", help="subnet to scan, e.g. 192.168.1")
    p.add_argument("--framesize", default="vga", choices=sorted(camera.FRAMESIZES))
    p.add_argument("--quality", type=int, default=12, help="JPEG quality, LOWER is better (10-63)")
    p.add_argument("--ae-level", type=int, default=-1, help="exposure bias; -1 keeps the shutter short")
    p.add_argument("--no-lock", action="store_true", help="leave AWB/AEC/AGC on auto")
    p.add_argument("--no-setup", action="store_true", help="do not touch the sensor at all")
    p.add_argument("--flip", choices=["none", "v", "h", "both"], default="none")
    p.add_argument("--bench", type=float, metavar="SEC", help="measure the link for SEC seconds, no window")
    return p.parse_args(argv)


def apply_flip(frame, flip: str):
    if flip == "v":
        return cv2.flip(frame, 0)
    if flip == "h":
        return cv2.flip(frame, 1)
    if flip == "both":
        return cv2.flip(frame, -1)
    return frame


class KeyHandler:
    """One handler, called from BOTH loop paths.

    cv2.waitKey consumes the keystroke wherever it is called, and the idle
    path (no new frame) runs far more often than the fresh-frame path. A
    handler that only lives on the fresh-frame path silently drops most
    keypresses.
    """

    def __init__(self, flip: str):
        self.flip = flip
        self.quit = False
        self.save_next = False

    def __call__(self, key: int) -> None:
        if key in (-1, 255):
            return
        ch = chr(key & 0xFF).lower()
        if ch == "q" or key == 27:
            self.quit = True
        elif ch == "v":
            self.flip = {"none": "v", "v": "none", "h": "both", "both": "h"}[self.flip]
            print(f"[key] flip = {self.flip}")
        elif ch == "h":
            self.flip = {"none": "h", "h": "none", "v": "both", "both": "v"}[self.flip]
            print(f"[key] flip = {self.flip}")
        elif ch == "s":
            self.save_next = True


def resolve_ip(args) -> str | None:
    if args.esp_ip:
        return args.esp_ip
    ip = camera.find_camera(args.subnet)
    if ip is None:
        print("[run] no camera found. Check it is on the same WiFi, then pass --esp-ip.")
    return ip


def run_bench(src: MjpegSource, seconds: float) -> int:
    """Average fps hides stutter: a stream that delivers frames in pairs with
    long holes averages the same as a steady one. So report the spacing."""
    print(f"[bench] measuring {seconds:.0f}s ...")
    stamps: list[float] = []
    seen = src.frame_count
    start_bytes = src.bytes_received
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        if src.frame_count != seen:
            seen = src.frame_count
            stamps.append(time.monotonic())
        else:
            time.sleep(0.002)

    elapsed = time.monotonic() - t0
    kb = (src.bytes_received - start_bytes) / 1024.0
    print(f"[bench] {len(stamps)} frames in {elapsed:.1f}s = {len(stamps)/elapsed:.1f} fps, "
          f"{kb/elapsed:.0f} KB/s, {src.reconnects} reconnect(s)")

    gaps = [(b - a) * 1000 for a, b in zip(stamps, stamps[1:])]
    if len(gaps) < 3:
        print("[bench] too few frames to judge the shape of the problem.")
        return 1
    gaps.sort()
    med = statistics.median(gaps)
    p95 = gaps[int(len(gaps) * 0.95) - 1]
    print(f"[bench] frame spacing  median {med:.0f} ms | p95 {p95:.0f} ms | worst {gaps[-1]:.0f} ms")

    fps = len(stamps) / elapsed
    kbs = kb / elapsed
    if fps >= 12:
        print("[bench] verdict: healthy.")
    elif kbs > 120:
        print("[bench] verdict: bandwidth-bound -- try --quality 16-18 or a smaller --framesize.")
    elif p95 < med * 2.0:
        print("[bench] verdict: the camera is not producing frames (even spacing, few bytes) "
              "-- try --ae-level -1, then --quality, then --no-lock.")
    else:
        print("[bench] verdict: radio link (long holes) -- check ping per-line, the antenna "
              "jumper, and the distance. Compressing harder will not help.")
    return 0


def main(argv=None) -> int:
    args = parse_args(argv)

    ip = resolve_ip(args)
    if ip is None:
        return 2

    if not args.no_setup:
        # Before the stream opens: the sensor serves one client, and the
        # /capture used to verify a resolution cannot run while it is busy.
        camera.prepare_camera(
            ip,
            framesize=args.framesize,
            quality=args.quality,
            ae_level=args.ae_level,
            lock=not args.no_lock,
        )

    src = MjpegSource(ip)
    src.start()
    print(f"[run] streaming from http://{ip}:81/stream")
    if not src.wait_for_frame(10.0):
        print(f"[run] no frame after 10s. last error: {src.last_error}")
        src.stop()
        return 3

    try:
        if args.bench:
            return run_bench(src, args.bench)
        return show_loop(src, args)
    finally:
        src.stop()


def show_loop(src: MjpegSource, args) -> int:
    keys = KeyHandler(args.flip)
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    seen = -1
    shown = 0
    t0 = time.monotonic()

    while not keys.quit:
        if src.frame_count == seen:
            # Nothing new. get_frame() would happily hand back the last picture
            # forever, and re-processing it just burns a core and starves the
            # receive thread through the GIL.
            keys(cv2.waitKey(5))
            continue

        seen = src.frame_count
        frame = src.get_frame()
        if frame is None:
            continue
        frame = apply_flip(frame, keys.flip)

        shown += 1
        elapsed = time.monotonic() - t0
        label = f"{frame.shape[1]}x{frame.shape[0]}  {shown/elapsed:.1f} fps  frames {seen}"
        cv2.putText(frame, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
        cv2.putText(frame, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        if keys.save_next:
            SAVE_DIR.mkdir(exist_ok=True)
            path = SAVE_DIR / f"frame_{int(time.time())}_{seen}.jpg"
            cv2.imwrite(str(path), frame)
            print(f"[key] saved {path}")
            keys.save_next = False

        cv2.imshow(WINDOW, frame)
        keys(cv2.waitKey(1))

    cv2.destroyAllWindows()
    print(f"[run] {shown} frames shown, {src.reconnects} reconnect(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
