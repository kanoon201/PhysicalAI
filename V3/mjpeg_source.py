"""Receive the MJPEG stream from CameraWebServer (port 81, /stream).

Interface kept deliberately small so anything built on top of it does not
care where frames come from:

    src = MjpegSource(ip); src.start()
    if src.frame_count != seen:      # <-- always gate on this, not on the frame
        frame = src.get_frame()      # BGR, straight from cv2.imdecode
    src.stop()
"""

from __future__ import annotations

import threading
import time
import urllib.request

import cv2
import numpy as np

SOI = b"\xff\xd8"
EOI = b"\xff\xd9"
CHUNK = 4096


class MjpegSource:
    def __init__(self, ip: str, port: int = 81, path: str = "/stream", timeout: float = 6.0):
        self.url = f"http://{ip}:{port}{path}"
        self.timeout = timeout

        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        # Counters are plain ints written only by the reader thread; readers
        # sample them without the lock on purpose (cheap, and monotonic).
        self.frame_count = 0
        self.bytes_received = 0
        self.reconnects = 0
        self.last_error: str | None = None

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="mjpeg", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def get_frame(self) -> np.ndarray | None:
        """Latest decoded frame. Returns the SAME frame forever if none is new
        -- that is why callers must compare frame_count instead."""
        with self._lock:
            return self._frame

    def wait_for_frame(self, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.frame_count:
                return True
            time.sleep(0.05)
        return False

    # -- reader ------------------------------------------------------------

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._read_stream()
            except Exception as exc:  # noqa: BLE001 - any failure means reconnect
                self.last_error = f"{type(exc).__name__}: {exc}"
            if self._stop.is_set():
                break
            self.reconnects += 1
            time.sleep(1.0)

    def _read_stream(self) -> None:
        req = urllib.request.Request(self.url, headers={"Connection": "close"})
        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            buf = bytearray()
            while not self._stop.is_set():
                # read1(), never read(). urlopen hands back a BufferedReader and
                # read(n) blocks until it has all n bytes; an MJPEG response has
                # no overall Content-Length, so a finished JPEG would sit unread
                # until the NEXT frame started arriving -- exactly one frame
                # interval of latency, which reads as stutter.
                data = resp.read1(CHUNK)
                if not data:
                    return  # server closed
                buf += data
                self.bytes_received += len(data)
                self._drain(buf)

    def _drain(self, buf: bytearray) -> None:
        """Pull every complete JPEG out of the buffer, keep the remainder."""
        while True:
            start = buf.find(SOI)
            if start < 0:
                # No frame start in sight: keep only a trailing byte in case a
                # marker is split across reads.
                del buf[: max(0, len(buf) - 1)]
                return
            end = buf.find(EOI, start + 2)
            if end < 0:
                del buf[:start]  # drop the multipart headers before the frame
                return
            jpeg = bytes(buf[start : end + 2])
            del buf[: end + 2]
            self._decode(jpeg)

    def _decode(self, jpeg: bytes) -> None:
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return  # truncated frame, just skip it
        with self._lock:
            self._frame = img
        self.frame_count += 1
