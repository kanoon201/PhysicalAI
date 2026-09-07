"""Find an ESP32-CAM on the LAN and set it up before the stream is opened.

Everything here talks to the CameraWebServer control server on port 80.
The stream itself (port 81) is a separate httpd instance, so /status and
/control keep answering while a stream is running -- but the *sensor* can
only serve one stream client, so all resolution changes must happen before
mjpeg_source.MjpegSource.start().
"""

from __future__ import annotations

import json
import socket
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np

# Sizes we care about, as (width, height). The numeric `val` for
# /control?var=framesize is a raw index into the core's framesize_t enum and
# it MOVES between esp32 core versions -- so it is probed, never hardcoded.
FRAMESIZES = {
    "qqvga": (160, 120),
    "qvga": (320, 240),
    "cif": (400, 296),
    "vga": (640, 480),
    "svga": (800, 600),
    "xga": (1024, 768),
    "sxga": (1280, 1024),
    "uxga": (1600, 1200),
}

PROBE_RANGE = range(0, 24)


class CameraControl:
    """Thin wrapper over the /status, /control and /capture endpoints."""

    def __init__(self, ip: str, timeout: float = 4.0):
        self.ip = ip
        self.timeout = timeout
        self.base = f"http://{ip}"

    def _get(self, path: str) -> bytes:
        with urllib.request.urlopen(self.base + path, timeout=self.timeout) as r:
            return r.read()

    def status(self) -> dict:
        return json.loads(self._get("/status").decode("utf-8", "replace"))

    def set_var(self, var: str, val) -> bool:
        try:
            self._get(f"/control?var={var}&val={val}")
            return True
        except (urllib.error.URLError, OSError, TimeoutError):
            return False

    def capture_size(self) -> tuple[int, int] | None:
        """Grab one still and report (width, height), or None if it failed."""
        try:
            raw = self._get("/capture")
        except (urllib.error.URLError, OSError, TimeoutError):
            return None
        img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return None
        h, w = img.shape[:2]
        return w, h


def local_subnet() -> str | None:
    """Best guess at the /24 this PC is on, e.g. '192.168.1'."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))  # no packet is sent, just picks a route
        ip = s.getsockname()[0]
    except OSError:
        return None
    finally:
        s.close()
    return ip.rsplit(".", 1)[0]


def _probe_host(ip: str, timeout: float) -> str | None:
    try:
        with urllib.request.urlopen(f"http://{ip}/status", timeout=timeout) as r:
            body = r.read(400)
    except (urllib.error.URLError, OSError, TimeoutError):
        return None
    # The camera's /status is a JSON dict carrying framesize; anything else
    # answering on port 80 (a router, a printer) is not our board.
    if b"framesize" in body:
        return ip
    return None


def find_camera(subnet: str | None = None, timeout: float = 0.6) -> str | None:
    """Scan a /24 for something whose /status looks like a CameraWebServer."""
    subnet = subnet or local_subnet()
    if subnet is None:
        return None
    hosts = [f"{subnet}.{i}" for i in range(1, 255)]
    print(f"[camera] scanning {subnet}.1-254 ...")
    with ThreadPoolExecutor(max_workers=64) as pool:
        for found in pool.map(lambda h: _probe_host(h, timeout), hosts):
            if found:
                print(f"[camera] found at {found}")
                return found
    return None


def _probe_framesize(cam: CameraControl, want: str) -> int | None:
    """Find the enum index that actually produces the requested size.

    Tries the last few indices that historically meant VGA/QVGA first, then
    sweeps. Each candidate is verified with a real /capture -- asking is not
    the same as getting.
    """
    target = FRAMESIZES[want]
    for val in PROBE_RANGE:
        if not cam.set_var("framesize", val):
            continue
        time.sleep(0.35)  # the sensor needs a moment to re-init
        got = cam.capture_size()
        if got == target:
            return val
    return None


def prepare_camera(
    ip: str,
    framesize: str = "vga",
    quality: int = 12,
    ae_level: int = -1,
    lock: bool = True,
    settle: float = 2.0,
) -> bool:
    """Put the sensor in a known state. Call BEFORE opening the stream.

    Order matters and is not arbitrary:
      1. resolution  -- CameraWebServer boots at QVGA and stays there
      2. quality     -- esp32-camera inverts the scale, lower = better
      3. ae_level    -- must be written before AEC is frozen or it does nothing
      4. lock        -- freeze AWB/AEC/AGC only after the image has settled
    """
    cam = CameraControl(ip)

    val = _probe_framesize(cam, framesize)
    if val is None:
        print(f"[camera] could not reach {framesize}, leaving resolution alone")
    else:
        print(f"[camera] framesize {framesize} = enum {val}")

    cam.set_var("quality", quality)
    cam.set_var("ae_level", ae_level)

    if lock:
        # Let AEC/AWB find a good image first, then freeze it. All five loops
        # are separate; leaving any of them on keeps the colours moving.
        time.sleep(settle)
        for var in ("awb", "awb_gain", "aec", "aec2", "agc"):
            cam.set_var(var, 0)
        print("[camera] AWB/AEC/AGC locked")

    return val is not None
