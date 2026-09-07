"""
Video source selection
=======================
One place that decides which live-camera receiver to use, so every tool
(cone_detection, hsv_calibrator, capture_esp32_samples) accepts the same
--source values and they stay consistent.

Two ESP32 firmwares are in play in this project and they stream in
completely different ways:

  --source esp32       sketch_jul13a.ino  -> JPEG over UDP (ports 44444/44442)
  --source webserver   CameraWebServer    -> MJPEG over HTTP (port 81, /stream)

Both hand back plain BGR frames, so the detection code doesn't care which
one is behind it. A full 'http://...' URL is also accepted and treated as
an MJPEG stream.
"""

STREAM_SOURCES = ("esp32", "webserver")


def is_stream_source(source):
    s = source.lower()
    return s in STREAM_SOURCES or s.startswith("http://") or s.startswith("https://")


def make_stream_source(source, esp_ip=None):
    """Build (but do not start) the receiver for a live-camera source.
    Returns None if `source` isn't a live-camera source — the caller
    should fall back to cv2.VideoCapture / cv2.imread for that."""
    s = source.lower()

    if s == "esp32":
        from esp32_video_source import ESP32VideoSource
        return ESP32VideoSource(esp_ip=esp_ip)

    if s == "webserver" or s.startswith("http://") or s.startswith("https://"):
        from mjpeg_video_source import MJPEGVideoSource
        target = source if s.startswith("http") else esp_ip
        if not target:
            raise ValueError("--source webserver needs --esp-ip (the ESP32-CAM's IP address)")
        return MJPEGVideoSource(target)

    return None
