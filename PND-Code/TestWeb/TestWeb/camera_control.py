"""
CameraWebServer control endpoint
==================================
Sets the ESP32-CAM's resolution over HTTP so TestWeb runs at the size we
want without reflashing. CameraWebServer boots at QVGA (320x240) —
CameraWebServer.ino drops it there right after init for speed — while the
robot firmware runs VGA, so anything tuned in pixel units doesn't carry
over between them unless we match the two.

    http://<ip>/control?var=framesize&val=<enum index>
    http://<ip>/capture                                 -> one JPEG

The catch: `val` is a raw index into the framesize_t enum, and that enum
is NOT stable across esp32 core versions — the 4.0.0-alpha1 core here
inserted 128X128 and 320X320 into the middle, which shifts VGA from the
widely-quoted 8 to 10. So instead of trusting any single number, we try
the plausible indices and keep the one whose /capture actually comes back
at the resolution we asked for.
"""

import time
import urllib.request
from urllib.parse import urlparse

import cv2
import numpy as np

WEB_PORT = 80
HTTP_TIMEOUT = 5.0

# Target pixel sizes by name, plus the enum indices worth trying for each.
# Ordered newest-core-first; the probe below confirms which one is real.
RESOLUTIONS = {
    "qvga": ((320, 240), (6, 5)),
    "vga": ((640, 480), (10, 8)),
    "svga": ((800, 600), (11, 9)),
    "xga": ((1024, 768), (12, 10)),
}


def _base(ip):
    """Control endpoints live on port 80, but callers may hand us the port-81
    stream URL instead of a bare IP — so keep only the host."""
    host = ip
    if "://" in host:
        host = urlparse(host).hostname or host
    elif ":" in host:
        host = host.split(":", 1)[0]
    return f"http://{host}:{WEB_PORT}"


def capture_frame(ip, timeout=HTTP_TIMEOUT):
    """Grab a single still off /capture. Returns a BGR frame, or None."""
    try:
        with urllib.request.urlopen(f"{_base(ip)}/capture", timeout=timeout) as r:
            data = r.read()
    except Exception:
        return None
    return cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)


def _set_var(ip, var, val, timeout=HTTP_TIMEOUT):
    try:
        with urllib.request.urlopen(f"{_base(ip)}/control?var={var}&val={val}",
                                    timeout=timeout) as r:
            return r.status == 200
    except Exception:
        return False


def set_quality(ip, quality):
    """JPEG quality: 10 (best) .. 63 (worst). Lower = sharper but bigger frames."""
    return _set_var(ip, "quality", int(quality))


def set_resolution(ip, name):
    """Switch the camera to a named resolution and verify it took effect.

    Returns the (width, height) actually in force, or None if the camera
    couldn't be reached at all. A return value different from what was
    asked for means the sensor refused the size — the caller should carry
    on with what it got rather than fail."""
    name = name.lower()
    if name not in RESOLUTIONS:
        raise ValueError(f"unknown resolution '{name}' (have: {', '.join(RESOLUTIONS)})")

    (want_w, want_h), candidates = RESOLUTIONS[name]

    current = capture_frame(ip)
    if current is None:
        return None
    if (current.shape[1], current.shape[0]) == (want_w, want_h):
        return (want_w, want_h)

    for val in candidates:
        if not _set_var(ip, "framesize", val):
            continue
        # The sensor needs a moment to reconfigure; the first frame after a
        # size change is often still the old one or a partial.
        time.sleep(0.4)
        frame = capture_frame(ip)
        if frame is None:
            continue
        if (frame.shape[1], frame.shape[0]) == (want_w, want_h):
            return (want_w, want_h)

    # Nothing matched — report whatever the camera is actually giving us so
    # the caller can print an honest number instead of an assumed one.
    frame = capture_frame(ip)
    if frame is None:
        return None
    return (frame.shape[1], frame.shape[0])
