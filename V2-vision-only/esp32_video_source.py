"""
ESP32-CAM Video Source (standalone receiver)
==============================================
The webcam on your PC is a completely different sensor/lens/compression
pipeline than the ESP32-CAM — colors, sharpness, and noise characteristics
don't match, so nothing tested on webcam footage carries over to the
robot's real feed. This module receives frames DIRECTLY from the
ESP32-CAM's UDP stream (the same stream 2026_07_13.py uses), so anything
run against it sees exactly what the robot sees.

Protocol (matches sketch_jul13a.ino):
  - The ESP32 listens on DISCOVERY_PORT for ANY UDP packet and starts
    streaming video to whichever IP that packet came from. We send a
    small beacon periodically so it knows we exist.
  - Video arrives on VIDEO_PORT as JPEG frames fragmented into UDP
    chunks: an 8-byte header (4-byte little-endian seq, 2-byte
    total_chunks, 2-byte chunk_id) followed by the chunk payload.

Usage as a module:
    from esp32_video_source import ESP32VideoSource
    src = ESP32VideoSource(esp_ip="192.168.8.191")  # or None to broadcast
    src.start()
    frame = src.get_frame()   # BGR numpy array, or None until first frame arrives
    ...
    src.stop()

Usage standalone (just to check the feed comes through):
    python esp32_video_source.py --esp-ip 192.168.8.191
    python esp32_video_source.py            # broadcast discovery instead
"""

import socket
import time
import threading
import argparse
import cv2
import numpy as np

DISCOVERY_PORT = 44444
VIDEO_PORT = 44442
HOST_IP = "0.0.0.0"


class ESP32VideoSource:
    def __init__(self, esp_ip=None, beacon_interval=0.5):
        """
        esp_ip: IP of the ESP32-CAM if known (recommended — faster to
                connect and avoids flooding the LAN with broadcasts).
                Leave None to broadcast on the local subnet instead
                (slower to establish, works without knowing the IP).
        """
        self.esp_ip = esp_ip
        self.beacon_interval = beacon_interval
        self._running = False
        self._latest_frame = None
        self._frame_count = 0
        self._lock = threading.Lock()
        self._threads = []

    def start(self):
        self._running = True
        t1 = threading.Thread(target=self._beacon_loop, daemon=True)
        t2 = threading.Thread(target=self._recv_loop, daemon=True)
        t1.start()
        t2.start()
        self._threads = [t1, t2]

    def stop(self):
        self._running = False
        for t in self._threads:
            t.join(timeout=1.0)

    def get_frame(self):
        """Returns the latest decoded BGR frame, or None if nothing received yet."""
        with self._lock:
            return None if self._latest_frame is None else self._latest_frame.copy()

    @property
    def frame_count(self):
        return self._frame_count

    def _beacon_loop(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            msg = b"PC_HERE,delivery:100"
            while self._running:
                try:
                    if self.esp_ip:
                        s.sendto(msg, (self.esp_ip, DISCOVERY_PORT))
                    else:
                        s.sendto(msg, ('<broadcast>', DISCOVERY_PORT))
                except Exception:
                    pass
                time.sleep(self.beacon_interval)

    def _recv_loop(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2097152)
        except Exception:
            pass
        srv.bind((HOST_IP, VIDEO_PORT))
        srv.settimeout(0.5)

        frames = {}
        last_processed_seq = 0

        while self._running:
            try:
                data, addr = srv.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                continue

            if len(data) < 8:
                continue

            seq = int.from_bytes(data[0:4], 'little')
            total = int.from_bytes(data[4:6], 'little')
            chunk_id = int.from_bytes(data[6:8], 'little')
            payload = data[8:]

            if seq < last_processed_seq and (last_processed_seq - seq < 5000):
                continue

            if seq not in frames:
                if frames:
                    for k in list(frames.keys()):
                        if k < seq:
                            del frames[k]
                frames[seq] = {'total': total, 'chunks': {}}

            frames[seq]['chunks'][chunk_id] = payload

            if len(frames[seq]['chunks']) == frames[seq]['total']:
                entry = frames.pop(seq)
                try:
                    full_frame = b"".join(entry['chunks'][i] for i in range(entry['total']))
                except KeyError:
                    continue

                if full_frame.startswith(b'\xff\xd8') and full_frame.endswith(b'\xff\xd9'):
                    np_arr = np.frombuffer(full_frame, dtype=np.uint8)
                    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    if frame is not None:
                        with self._lock:
                            self._latest_frame = frame
                            self._frame_count += 1
                        last_processed_seq = seq

        srv.close()


def main():
    parser = argparse.ArgumentParser(description="Preview the raw ESP32-CAM feed (no controller UI, just the video)")
    parser.add_argument("--esp-ip", default=None, help="ESP32-CAM IP (skip to broadcast-discover instead)")
    args = parser.parse_args()

    src = ESP32VideoSource(esp_ip=args.esp_ip)
    src.start()
    print("Waiting for frames... (make sure the ESP32-CAM is powered on and on the same WiFi)")
    print("Press 'q' to quit.")

    last_seen_count = -1
    try:
        while True:
            frame = src.get_frame()
            if frame is not None:
                if src.frame_count != last_seen_count:
                    last_seen_count = src.frame_count
                cv2.imshow("ESP32-CAM Live Feed", frame)
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break
    finally:
        src.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
