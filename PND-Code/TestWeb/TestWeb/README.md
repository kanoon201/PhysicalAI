# TestWeb — cone detection off the CameraWebServer feed

Self-contained copy of the cone-detection toolkit, set up to run against
the **CameraWebServer** sketch (MJPEG over HTTP) instead of the robot
firmware's UDP stream. Nothing here needs the robot to be assembled —
just an ESP32-CAM on the same WiFi.

## Files

| File | What it does |
| --- | --- |
| `run.py` | **Start here.** One command: finds the camera, sets it to VGA, streams it, and runs detection with live HSV trackbars. |
| `camera_control.py` | Sets resolution/quality over `/control`, verifying the change actually took. |
| `mjpeg_video_source.py` | Receives `http://<ip>:81/stream` from CameraWebServer. Threaded, keeps only the newest frame, auto-reconnects. |
| `video_source.py` | Picks the receiver for a given `--source` (`webserver`, `esp32`, or an `http://` URL). |
| `esp32_video_source.py` | Receives the UDP feed from `sketch_jul13a.ino` (kept so `--source esp32` still works here too). |
| `cone_detection.py` | HSV cone detection + lane-keeping steering (`ConeLaneKeeper`). |
| `hsv_calibrator.py` | Trackbar tool to find the HSV range for your cones. |
| `capture_esp32_samples.py` | Saves frames off the live feed as `.jpg` for offline testing. |

## Setup

Flash `CameraWebServer/CameraWebServer.ino` to the ESP32-CAM and power it
on. You don't need to read the IP off the Serial Monitor — `run.py` scans
the network for it.

```
pip install opencv-python numpy
```

## Run it

One command does everything — finds the camera, switches it to **VGA
(640×480)**, opens the stream, and shows the detection with live HSV
trackbars:

```
python run.py
```

VGA is the default because the robot firmware (`sketch_jul13a.ino`) runs
VGA. Matching them means `min_contour_area` — which is in raw pixels —
carries over unchanged when you move from testing to driving. Use
`--framesize qvga` for a faster, lower-res feed if you don't care about
that.

Press `s` to save the tuned HSV values to `cone_config.json`, `q` to quit.
Adjust the trackbars until the **Mask** window shows only the cones as
solid white; the **Cone Detection** window then shows the boxes, the
target point, and the `steer` / `throttle` the car would be commanded to
use. Nothing is sent to any motor from here.

If the scan picks the wrong device or you already know the IP:

```
python run.py --esp-ip 192.168.8.191
```

To run the same thing against the robot firmware's UDP feed instead:

```
python run.py --source esp32
```

## The individual tools

`run.py` covers the normal case; these are still here for when you want
one piece on its own.

```
python mjpeg_video_source.py --esp-ip 192.168.8.191                 # just check the feed
python hsv_calibrator.py --source webserver --esp-ip 192.168.8.191   # tuning only
python cone_detection.py --source webserver --esp-ip 192.168.8.191   # detection only
python cone_detection.py --source cone_photo.jpg                     # a still photo
```

Grab still frames to re-test offline without the camera:

```
python capture_esp32_samples.py --source webserver --esp-ip 192.168.8.191
python cone_detection.py --source esp32_samples/esp32_sample_xxx.jpg
```

## Things that will bite you

- **One client at a time.** The ESP32-CAM serves a single stream client.
  Close the browser tab showing the camera page or Python gets no frames.
- **CameraWebServer has no motor control.** This folder is for testing and
  tuning the vision only. To actually drive, flash `sketch_jul13a.ino` back
  and switch to `--source esp32` — the HSV values carry over unchanged,
  since it's the same sensor.
- **The resolution change is not permanent.** `run.py` sets VGA over HTTP
  every time it starts, so a camera reboot back to QVGA fixes itself. If
  you'd rather bake it in, change `FRAMESIZE_QVGA` to `FRAMESIZE_VGA` at
  `CameraWebServer.ino:96` and reflash.
- **Don't hardcode framesize enum numbers.** The `val` in
  `/control?var=framesize&val=N` is a raw index into `framesize_t`, and the
  esp32 core on this machine (4.0.0-alpha1) inserted `128X128` and
  `320X320` into that enum — VGA is 10 here, not the 8 quoted in most
  online examples. `camera_control.py` probes and verifies via `/capture`
  rather than trusting a number.
- **MJPEG over TCP lags a bit more** than the robot's UDP stream. Fine for
  tuning; use the UDP path when the loop is closed around the motors.
- This is a **copy** of the files in `../Detect`. Edits here do not
  propagate there, and vice versa — decide which folder is the real one
  before you start changing detection logic in both.
