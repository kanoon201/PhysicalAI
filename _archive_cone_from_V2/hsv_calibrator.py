"""
HSV Calibration Tool
=====================
Interactive trackbar tool to find the right HSV threshold range for
your actual cone color under your actual lighting. Do this BEFORE
trusting cone_detection.py — a threshold tuned on someone else's
cone/lighting will not transfer to yours.

Usage:
    python hsv_calibrator.py --source 0            # webcam
    python hsv_calibrator.py --source cone.jpg      # a still photo
    python hsv_calibrator.py --source clip.mp4      # a recorded video
    python hsv_calibrator.py --source webserver --esp-ip 192.168.8.191  # CameraWebServer feed
    python hsv_calibrator.py --source esp32 --esp-ip 192.168.8.191      # sketch_jul13a feed

Tune against the ESP32-CAM feed, not the webcam — the thresholds have to
match the sensor and JPEG compression the robot actually sees.

Controls:
    Adjust the 6 trackbars until the "Mask" window shows ONLY the
    cones as solid white, everything else black.
    's' = save current values to cone_config.json
    'q' = quit without saving
"""

import cv2
import numpy as np
import argparse
from cone_detection import CONFIG_PATH, load_config, save_config
from video_source import is_stream_source, make_stream_source


def nothing(_):
    pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="0", help="Camera index, video file, image file, 'esp32' (sketch_jul13a UDP feed) or 'webserver' (CameraWebServer MJPEG feed)")
    parser.add_argument("--esp-ip", default=None, help="ESP32-CAM IP address. Required for --source webserver; optional for --source esp32 (omit to broadcast-discover instead)")
    args = parser.parse_args()

    cfg = load_config()

    cv2.namedWindow("Calibrator")
    cv2.createTrackbar("H min", "Calibrator", cfg["hsv_range1_lower"][0], 179, nothing)
    cv2.createTrackbar("S min", "Calibrator", cfg["hsv_range1_lower"][1], 255, nothing)
    cv2.createTrackbar("V min", "Calibrator", cfg["hsv_range1_lower"][2], 255, nothing)
    cv2.createTrackbar("H max", "Calibrator", cfg["hsv_range1_upper"][0], 179, nothing)
    cv2.createTrackbar("S max", "Calibrator", cfg["hsv_range1_upper"][1], 255, nothing)
    cv2.createTrackbar("V max", "Calibrator", cfg["hsv_range1_upper"][2], 255, nothing)

    source = args.source
    is_image = source.lower().endswith((".jpg", ".jpeg", ".png"))
    is_esp32 = is_stream_source(source)
    cap = None
    static_frame = None
    esp_src = None

    if is_esp32:
        esp_src = make_stream_source(source, esp_ip=args.esp_ip)
        esp_src.start()
        print("Connecting to ESP32-CAM... (waiting for first frame — make sure it's powered on)")
    elif is_image:
        static_frame = cv2.imread(source)
        if static_frame is None:
            print(f"Cannot read image: {source}")
            return
    else:
        cap_source = int(source) if source.isdigit() else source
        cap = cv2.VideoCapture(cap_source)
        if not cap.isOpened():
            print(f"Cannot open source: {source}")
            return

    print("Adjust trackbars until only the cones are white in the Mask window.")
    print("Press 's' to save config, 'q' to quit without saving.")

    while True:
        if esp_src is not None:
            frame = esp_src.get_frame()
            if frame is None:
                # Still waiting for the first frame from the ESP32 — don't
                # spin the CPU, and skip processing until we have one.
                if cv2.waitKey(100) & 0xFF == ord('q'):
                    break
                continue
        elif cap is not None:
            ret, frame = cap.read()
            if not ret:
                break
        else:
            frame = static_frame.copy()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h_min = cv2.getTrackbarPos("H min", "Calibrator")
        s_min = cv2.getTrackbarPos("S min", "Calibrator")
        v_min = cv2.getTrackbarPos("V min", "Calibrator")
        h_max = cv2.getTrackbarPos("H max", "Calibrator")
        s_max = cv2.getTrackbarPos("S max", "Calibrator")
        v_max = cv2.getTrackbarPos("V max", "Calibrator")

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow("Original", frame)
        cv2.imshow("Mask", mask)
        cv2.imshow("Result", result)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cfg["hsv_range1_lower"] = [int(h_min), int(s_min), int(v_min)]
            cfg["hsv_range1_upper"] = [int(h_max), int(s_max), int(v_max)]
            save_config(cfg)
            print(f"Saved to {CONFIG_PATH}: lower={cfg['hsv_range1_lower']} upper={cfg['hsv_range1_upper']}")

    if cap is not None:
        cap.release()
    if esp_src is not None:
        esp_src.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
