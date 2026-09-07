"""
One-command runner
===================
Everything in a single process: finds the ESP32-CAM on the network, puts
it in the resolution we want, opens its stream, and shows the live video.

    python run.py                          # auto-find the camera
    python run.py --esp-ip 192.168.8.191   # skip the scan if you know the IP
    python run.py --source esp32           # robot firmware (UDP) instead
    python run.py --flip both              # camera mounted upside down
    python run.py --detect                 # blue cone detection + lane center

Controls:
    'v' : flip vertically (toggle)
    'h' : mirror horizontally (toggle)
    's' : save the current frame as a .jpg
    'w' : (--detect only) write the tuned values to cone_config.json
    'q' : quit

Without --detect this is video only. With it, cones are found, the middle
of the gap between them is worked out, and both are drawn — but nothing is
driven. See cone_detection.py and lane_center.py.
"""

import argparse
import os
import socket
import sys
import time
from concurrent.futures import ThreadPoolExecutor

import cv2

from camera_control import RESOLUTIONS, lock_image_settings, set_flip, set_resolution
from video_source import make_stream_source

STREAM_PORT = 81
WEB_PORT = 80
FIRST_FRAME_TIMEOUT = 15.0
SNAPSHOT_DIR = "esp32_samples"

# --flip name -> (vertical, horizontal)
FLIP_MODES = {
    "none": (False, False),
    "v": (True, False),
    "h": (False, True),
    "both": (True, True),
}


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
        print("  - it joined the same WiFi as this PC")
        print("  - CameraWebServer.ino is the sketch that's flashed")
        print("Otherwise read the IP off the Serial Monitor and pass --esp-ip.")
        return None
    if len(found) > 1:
        print(f"Multiple candidates found: {', '.join(found)} — using {found[0]}.")
        print("Pass --esp-ip if that's the wrong one.")
    print(f"Found camera at {found[0]}")
    return found[0]


# ============================================================
# FLIP
# ============================================================

def flip_frame(frame, vflip, hmirror):
    """Software flip, for when the sensor can't do it for us (the UDP
    firmware has no HTTP control endpoint) or refuses to."""
    if vflip and hmirror:
        return cv2.flip(frame, -1)
    if vflip:
        return cv2.flip(frame, 0)
    if hmirror:
        return cv2.flip(frame, 1)
    return frame


def push_flip_to_sensor(source, esp_ip, vflip, hmirror):
    """Try to move the flip into the sensor. Returns True if the camera
    took it, in which case the frames already arrive flipped and doing it
    again in OpenCV would undo the work.

    Only the CameraWebServer firmware has the HTTP endpoint. The UDP one
    doesn't serve port 80 at all, so asking it would just burn two HTTP
    timeouts before falling back — skip straight to software there."""
    if not esp_ip or source.lower() != "webserver":
        return False
    return set_flip(esp_ip, vflip=vflip, hmirror=hmirror)


# ============================================================
# MAIN
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="Find the ESP32-CAM, open its stream, show the live video, and optionally detect blue cones")
    parser.add_argument("--esp-ip", default=None, help="Camera IP. Omit to scan the local subnet for it.")
    parser.add_argument("--source", default="webserver", help="'webserver' (CameraWebServer MJPEG) or 'esp32' (sketch_jul13a UDP)")
    parser.add_argument("--framesize", default="vga", choices=sorted(RESOLUTIONS),
                        help="Resolution to put the camera in (default vga)")
    parser.add_argument("--flip", default="none", choices=sorted(FLIP_MODES),
                        help="Flip the image: 'v' upside down, 'h' mirrored, 'both', or 'none' (default)")
    parser.add_argument("--detect", action="store_true",
                        help="Run blue cone detection on the feed, with live tuning trackbars")
    parser.add_argument("--config", default="cone_config.json", help="Detector config JSON (with --detect)")
    parser.add_argument("--no-lock", action="store_true",
                        help="Leave the camera's auto white balance / exposure running (they are locked by default)")
    parser.add_argument("--out", default=SNAPSHOT_DIR, help="Folder that 's' saves frames into")
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

        # Before the stream opens, and after a settle pause, so the auto
        # loops freeze on a properly adjusted image of the real scene.
        if not args.no_lock:
            print("Locking white balance / exposure (2s settle — point the camera at the scene) ...")
            if lock_image_settings(esp_ip):
                print("Locked. Colors will now stay put; re-run with --no-lock to undo.")
            else:
                print("Camera did not accept the lock — colors will drift as the light changes.")

    vflip, hmirror = FLIP_MODES[args.flip]
    # Always push it, even for 'none' — that clears a flip left in the
    # sensor by an earlier run (it survives until the camera reboots).
    flip_in_sensor = push_flip_to_sensor(args.source, esp_ip, vflip, hmirror)
    if vflip or hmirror:
        where = "in the sensor" if flip_in_sensor else "in software"
        print(f"Flip: {args.flip} ({where}).")

    src = make_stream_source(args.source, esp_ip=esp_ip)
    if src is None:
        print(f"--source must be 'esp32', 'webserver', or an http:// stream URL (got: {args.source})")
        return 1
    src.start()

    detector = None
    control_win = "Cone tuning (w=write config)"
    if args.detect:
        from cone_detection import ConeDetector, build_trackbars, print_saved, read_trackbars, save_config
        from lane_center import LaneFinder, draw_lane
        detector = ConeDetector(config_path=args.config)
        # Shares the detector's dict, so the trackbars and the saved JSON
        # cover the lane settings too.
        lane = LaneFinder(detector.cfg)
        build_trackbars(control_win, detector.cfg)

    os.makedirs(args.out, exist_ok=True)
    saved_count = len([f for f in os.listdir(args.out) if f.lower().endswith(".jpg")])

    print("Waiting for the first frame ...")
    started_at = time.time()
    warned = False
    got_first_frame = False
    fps = 0.0
    last_tick = time.time()

    last_count = -1

    try:
        while True:
            # get_frame() hands back the last picture forever, so without
            # this check the loop re-runs detection and imshow hundreds of
            # times per second on frames it has already seen. That saturates
            # a core and starves the receive thread — which on the UDP path
            # drops chunks and makes the video visibly stutter.
            count = src.frame_count
            if count == last_count:
                key = cv2.waitKey(5) & 0xFF   # 5ms yields to the receiver
                if key == ord('q'):
                    break
                if not warned and not got_first_frame and time.time() - started_at > FIRST_FRAME_TIMEOUT:
                    print(f"Still no frames after {FIRST_FRAME_TIMEOUT:.0f}s. The ESP32-CAM only "
                          "serves ONE stream client — close any browser tab showing the camera page.")
                    warned = True
                continue
            last_count = count

            frame = src.get_frame()
            if frame is None:
                continue

            if not got_first_frame:
                got_first_frame = True
                print(f"Streaming {frame.shape[1]}x{frame.shape[0]}. "
                      f"'v'/'h' flip, 's' saves a frame into ./{args.out}/, 'q' quits.")
                if detector is not None:
                    print("Adjust the trackbars until Mask shows ONLY the cones white, then press 'w'.")

            if not flip_in_sensor:
                frame = flip_frame(frame, vflip, hmirror)

            now = time.time()
            # Smoothed so the readout doesn't flicker on every frame.
            fps = 0.9 * fps + 0.1 / max(now - last_tick, 1e-6)
            last_tick = now

            if detector is not None:
                read_trackbars(control_win, detector.cfg)
                cones, mask, display = detector.detect(frame)
                lane_result = lane.update(cones, frame.shape[1], frame.shape[0])
                draw_lane(display, lane_result)
                cv2.imshow("Mask", mask)
            else:
                display = frame.copy()

            flip_txt = ("V" if vflip else "") + ("H" if hmirror else "") or "-"
            # The detector writes its own line at y=20, so sit below it.
            info_y = 40 if detector is not None else 20
            cv2.putText(display, f"{frame.shape[1]}x{frame.shape[0]}  {fps:4.1f} fps  "
                                 f"flip:{flip_txt}  saved:{saved_count}",
                        (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.imshow("ESP32-CAM Live Feed", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key in (ord('v'), ord('h')):
                if key == ord('v'):
                    vflip = not vflip
                else:
                    hmirror = not hmirror
                flip_in_sensor = push_flip_to_sensor(args.source, esp_ip, vflip, hmirror)
                where = "sensor" if flip_in_sensor else "software"
                print(f"Flip: vflip={int(vflip)} hmirror={int(hmirror)} ({where})")
            if key == ord('w') and detector is not None:
                save_config(detector.cfg, args.config)
                print_saved(detector.cfg, args.config)
            if key == ord('s'):
                filename = os.path.join(args.out, f"esp32_sample_{int(time.time() * 1000)}.jpg")
                cv2.imwrite(filename, frame)
                saved_count += 1
                print(f"Saved: {filename}")
    finally:
        src.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
