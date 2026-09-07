# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An ESP32-CAM robot car. Firmware on an AI-Thinker ESP32-CAM streams JPEG video to a PC; Python on the PC displays it, finds **blue** traffic cones, works out the middle of the gap between them, and drives the car through it.

**`V2/` is the live folder and where new work belongs.** It holds the only complete driving pipeline in the repo. `backup/` is a deliberate vision-only copy of `V2/` (no driving code at all) and carries two fixes `V2/` still lacks — see the fix table. Everything else is history — see "The fork situation".

No build system, no test suite, no linter, no dependency manifest. Deps are `opencv-python`, `numpy` (and `pygame` for the legacy pre-V2 controllers only).

## Running things

**Always `cd` into the variant folder first.** `cone_detection.py` loads `cone_config.json` by relative path, and every module imports its siblings from the working directory. Running from the repo root silently picks up the wrong config or fails to import.

```bash
cd V2
python run.py                                   # find camera, stream, show video
python run.py --detect                          # + cone detection & lane center, tuning trackbars
python run.py --esp-ip <ip> --bench 12          # measure the link: no window, no detection
python run.py --esp-ip <ip> --quality 16 --ae-level -1   # the two framerate knobs
python run.py --flip both --framesize qvga

python esp32_link_check.py --esp-ip <ip>        # why is there no video at all?
python autodrive.py --dry-run                   # full loop incl. autopilot, sends nothing
python autodrive.py --esp-ip <ip>               # drives the car (starts DISARMED, SPACE to arm)

python cone_detection.py --source esp32_samples/x.jpg   # tune against a saved frame, no camera
python capture_esp32_samples.py --source webserver --esp-ip <ip>
```

`run.py` keys: `v`/`h` flip, `s` save frame, `w` write config, `q` quit.
`autodrive.py` keys: `SPACE` arm/disarm, `X` e-stop, `Q` quit, `W` write config.

`backup/` takes the same `run.py` commands minus anything that drives.

**Firmware**: Arduino IDE, board = AI-Thinker ESP32-CAM. The robot sketch allocates a 1 MB `tx_buffer` in PSRAM and reboots in a loop if that fails, so PSRAM has to be on — on this board it always is, and there is no PSRAM setting to tick. **WiFi credentials are not in the repo.** `V2/firmware/esp32cam_robot/` reads them from `wifi_secrets.h`, which is gitignored — copy `wifi_secrets.example.h` to `wifi_secrets.h` and edit it before flashing. The `#if __has_include` guard means a missing file still compiles, it just joins `YOUR_WIFI_SSID`. Every other `.ino` in the repo (the archive folders) has `YOUR_WIFI_SSID` / `YOUR_WIFI_PASSWORD` placeholders to edit in place.

**A firmware change can be compiled without a board**, which is worth doing before every handoff — the Arduino IDE ships `arduino-cli`:

```bash
CLI="$HOME/AppData/Local/Programs/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe"
"$CLI" compile --fqbn esp32:esp32:esp32cam V2/firmware/esp32cam_robot
"$CLI" compile --clean --warnings all --fqbn esp32:esp32:esp32cam V2/firmware/esp32cam_robot
```

esp32 core 3.3.11 is what is installed. **Do not add `:PSRAM=enabled` to the FQBN** — the `esp32cam` board defines no such option and the build dies with `invalid option 'PSRAM'`. The warning sweep is noisy with pre-existing `%u`/`uint32_t` format warnings and a deprecated `pin_sscb_sda`; read it for new entries, not for a clean sheet.

### Verification

There is no test runner. Everything is checked with throwaway scripts, and that is the expected pattern — it works well here and is worth continuing:

- **Vision**: synthesize frames with `cv2.drawContours` (a blue triangle on a grey floor) plus deliberate decoys — blue-but-above-the-ROI, blue-but-landscape, blue-but-tiny, an orange cone — and assert only the real one survives. Keep decoys from overlapping the real cones, or they merge into one contour and the test lies.
- **Protocol**: bind a UDP socket on `127.0.0.1:44441`, point `RobotLink` at localhost, and assert on the exact bytes.
- **Control law**: build `LaneResult` objects by hand and feed them to `Autopilot`.
- **Whole-program behaviour**: `run.main()` / `autodrive.main()` can be driven in a thread with `cv2.waitKey`, `cv2.imshow` and `make_stream_source` monkeypatched, plus a fake source producing frames at a chosen rate. This is how the key-handling and framerate bugs were pinned down.
- **Receivers**: a local HTTP server speaking multipart MJPEG, or a UDP sender speaking the chunk header, gives an exact known loss rate to measure against.
- **A timing fix needs the old behaviour in the same process to mean anything.** For `read1()`, the fake MJPEG server wrote a frame every 250 ms and the harness timed server-write → `frame_count` increment; the old bug was reproduced by handing back a response whose `read1` had been reassigned to `read`, and both variants were measured against the same server. 0–3 ms fixed vs 251–754 ms broken. Asserting only that the fixed path is fast would have passed just as well against a server that was never the bottleneck.
- **A guard gets tested in both directions.** `autodrive.py`'s firmware check was exercised five ways against a fake `/status` (robot board → let through; stock sketch → refused; unreachable host → refused; `--source esp32` → not consulted; `--dry-run` → not consulted), with `camera_control.WEB_PORT` pointed at the fake server and `ConeDetector` monkeypatched to raise, so the run stops the moment the guard passes. A guard asserted in one direction only is half-tested, and this one decides whether a car can be armed.

Two harness mistakes that produced convincing but false results: leaving a previous fake sender thread alive so two of them share sequence numbers (looks exactly like frame corruption), and asserting an expected delivery figure derived from the wrong formula. Isolate each case in its own process when the numbers matter.

## The two firmware families (the thing that confuses everything)

Two completely different sketches are in play, and which one is flashed decides what the PC code can do:

| | `CameraWebServer/` | `firmware/esp32cam_robot/` (the `sketch_jul13a` lineage) |
|---|---|---|
| Video | MJPEG over HTTP, port 81 `/stream` | **the same MJPEG by default**; UDP chunks on 44442 as a fallback |
| Settings | HTTP `/control?var=…&val=…`, `/status`, `/capture` | the same HTTP endpoints, **and** UDP command packets |
| Motors | **none at all** | yes |
| Receiver | `mjpeg_video_source.py` | `mjpeg_video_source.py`, or `esp32_video_source.py` on the fallback |

Since the MJPEG switchover the robot firmware serves `/stream` on 81 and `/control`, `/status`, `/capture` on 80, byte-identically to the stock sketch — so **`--source webserver` no longer tells you which sketch is flashed**, and `prepare_camera()`/`camera_control.py` now work against either. `--source esp32` still means the UDP chunk path, which only the robot firmware has.

That removed the old guard. `autodrive.py` used to refuse `--source webserver` outright, on the reasoning that MJPEG implied the motorless sketch; that reasoning is now wrong in the dangerous direction as well as the annoying one. The board is asked instead: the robot firmware reports `"robot":1` in `/status`, the stock sketch has no such field, and `camera_control.is_robot_firmware()` is what `autodrive.py` checks before it will arm.

Both receivers expose the same interface (`start()`, `get_frame()` → BGR, `frame_count`, `bytes_received`, `stop()`), and `video_source.make_stream_source()` picks between them, so everything above them is transport-agnostic.

**Why MJPEG became the default.** The UDP path has no retransmission: one lost chunk out of ~29 and `esp32_video_source.py` never completes the frame, so it drops it whole. TCP either delivers the JPEG byte-exact or stalls the socket. The UDP path is kept because it is the only one that can broadcast-discover the PC without being told an IP first, and it is one `/control?var=video_mode&val=1` (or UDP cmd 9) away.

## V2 pipeline

`video → cone_detection → lane_center → autopilot → robot_link → motors`

Each stage is a separate file, testable alone, and only the last two can move anything.

- **`camera_setup.py`** — everything that must happen *before* the stream opens: subnet scan, `prepare_camera()` (resolution → quality → `ae_level` → AWB/AEC/AGC lock, in that order), flip, and `set_robot_framesize()` for the UDP firmware. Shared by `run.py` and `autodrive.py` so they cannot drift apart on it, which they already did once.
- **`cone_detection.py`** — HSV threshold → morphology open/close → four filters (area, aspect, solidity, ROI) → `Cone` list, largest first. Blue, not orange: orange straddles the H=0/179 seam and needs two `inRange` passes, and the things that ruin an orange detector indoors (skin, cardboard, wooden floors) all live at H 5–25 on top of it. ROI is applied by **blanking the top of the mask, not cropping the frame**, so contour coordinates stay in full-frame space.
- **`lane_center.py`** — `LaneFinder.update(cones, w, h)` → `LaneResult(target_x, offset_norm, state, …)`. See its docstring for the four things that make this harder than `(left + right) / 2`; the short version is that splitting left/right at the frame center fails exactly when the car is off-center, so the split happens at the last known lane center and falls back to the widest gap between the cones.
- **`autopilot.py`** — the only file that decides motion. P only: the loop has ~150 ms of latency (WiFi + JPEG + decode), so an I term winds up and a D term amplifies the jitter `lane_center` just smoothed away. **If it hunts, lower `steer_gain` before adding terms.**
- **`robot_link.py`** — transport only, lifted from `Aj-Piya_code/2026_07_13.py`.
- **`esp32_link_check.py`** — beacons and listens on both robot ports at once, to separate "the ESP never heard us" from "the ESP answers but the video is blocked". Sends no drive commands.
- **`cone_config.json`** — one dict shared by the detector, `LaneFinder` and `Autopilot`; each fills in its own defaults via `setdefault`, so an older file still loads.

## Protocols

### Robot firmware: four ports

Both sides must change together.

| Port | Direction | Payload |
|---|---|---|
| 44444 | PC → ESP (broadcast) | `PC_HERE,delivery:<pct>` |
| 44444 | ESP → PC | `seq,chipID,RSSI:<n>,SSID:<s>,NAME:<s>` (1 Hz) |
| 44442 | ESP → PC | video chunks |
| 44441 | PC → ESP | `seq,cmd,v1,v2` (plain text) |

The ESP streams to whatever IP last sent it a packet on 44444 — any UDP packet works as a beacon.

**Video framing**: each JPEG frame is split into 1400-byte chunks with an 8-byte header — LE `uint32` seq, LE `uint16` total_chunks, LE `uint16` chunk_id. The receiver reassembles per-seq in a dict, discards frames older than the last completed one, and validates SOI/EOI (`\xff\xd8` … `\xff\xd9`) before decoding.

**Commands** (`v1`/`v2` repeat the cmd number on system commands as a crude checksum — the firmware refuses cmd 3 unless both are 3):

| cmd | meaning |
|---|---|
| 0 | drive — `v1`=steer, `v2`=throttle, 1000–2000 µs, 1500 = neutral |
| 1 | light — `v1` = 0..65535 PWM |
| 3 | `ESP.restart()` |
| 4/5/6/7 | QVGA / VGA / SVGA / XGA — each also re-runs `updateCameraSettings()` |
| 8 | video over MJPEG/HTTP (the default) |
| 9 | video over UDP chunks (the fallback) |

`ev()` maps −1..1 to 1000..2000; the firmware multiplies by `MAX_DUTY` (3.27675) for a 16-bit 50 Hz PWM, i.e. a standard servo/ESC pulse.

### Adaptive quality loop (UDP fallback only)

**This runs only while the board is in UDP video mode.** Over TCP the PC always measures ~100% delivery, and letting the loop act on that walks `current_quality` down to 10 — the biggest frames the sensor can make — and hands the radio more than it can carry. The firmware gates it on `video_udp_mode`; if you re-enable it for MJPEG, this is the failure you get.

Spans both sides and is easy to break by touching one half: the PC measures gaps in the video sequence into `stream_delivery_pct` (EMA over a beacon window), injects it into every beacon, and the firmware adjusts `current_quality` (lower = better; esp32-camera inverts the scale) after 3 consecutive lows or 5 consecutive highs.

**Both halves must be real.** The PC used to send a hardcoded `delivery:100`. On a lossy link that is not a harmless white lie but a signal pointing the wrong way: the firmware reads a perfect link, *lowers* compression, produces bigger frames, loses more chunks — and the PC keeps reporting 100%. Measured on an iPhone hotspot: 22% chunk loss, frames grown to 32 chunks, 1.2 usable fps.

Measure it as **completed frames / sequence span per window**, not as the mean of `1/gap` per frame — gaps of 1,1,2 average to 83% when only 3 of 4 frames actually arrived.

The video sequence number **must be a frame counter, not `millis()`** — a timestamp makes every gap look like ~50 lost frames and pins the link at max compression. The pre-V2 firmware variants all have this bug; `V2/firmware/esp32cam_robot/` does not.

## Traps that have already cost time

### PC loop and receivers

- **Process only on a new frame.** `get_frame()` returns the last picture forever. A loop that processes every iteration runs detection ~500×/s against ~20 real frames/s, saturates a core, starves the receive thread through the GIL, and on the lossy UDP path that shows up as **stuttering video**. Check `src.frame_count` first and `cv2.waitKey(5)` when it hasn't changed. On TCP/MJPEG the same bug shows as latency instead, which is why it hid for so long.
- **Anything derived from "is there a frame?" is always true**, for the same reason — a stale-video safety cutout keyed on `frame is not None` can never fire. Key it on `frame_count`.
- **Read the MJPEG socket with `read1()`, never `read()`.** `urlopen()` returns an `HTTPResponse` backed by a `BufferedReader`, and `BufferedReader.read(n)` blocks until it has all n bytes; an MJPEG response has no overall Content-Length, so that is the path taken. A finished JPEG then sits unnoticed until ~4 KB of the *next* frame arrives — measured at exactly one frame interval of delay, which bunches frames into pairs and reads as stutter. Fixed in `V2/` on 2026-09-06; against a 250 ms fake server the two paths measured 0–3 ms and 251–754 ms. This matters more than it used to: MJPEG is now the robot's default video path, not just the stock sketch's.
- **One `handle_key()`, called from both loop paths.** `waitKey` consumes the keystroke wherever it is called, and the idle path runs ~25× more often than the fresh-frame path at 8 fps. A handler that only exists on the fresh-frame path drops ~96% of keypresses — measured 0/20 before, 20/20 after. In `autodrive.py` that meant SPACE not arming and X not e-stopping.
- **A sequence number that jumps backwards means the ESP rebooted**, not reordering. It restarts its frame counter at 1, and the old high-water mark then discards every new frame until the counter catches up — minutes of "connected, no picture". Treat a backwards jump beyond `REORDER_WINDOW` as a resync.
- **The firmware zeroes the motors 150 ms after the last control packet.** So "keep going straight" must be re-sent continuously; `RobotLink.drive()` re-sends an unchanged command every 50 ms, and the caller must keep calling it even on iterations where no new frame arrived.
- **`RobotLink.stop()` bypasses the queue on purpose.** The sender collapses a batch of drive packets to the newest one (right for steering, since a queued backlog makes the car chase positions it has already left) — which would swallow all but one packet of a stop burst, and could transmit a stale full-throttle command *after* the stop.

### The robot firmware serving two transports

- **Only one thing may own the camera.** In MJPEG mode `cameraDrainTask` and `streamTXTask` return early and never call `esp_camera_fb_get()`. If they don't, they fight the `/stream` handler over two frame buffers and burn core 0 — the core the WiFi stack runs on — producing frames nobody sends.
- **The 120 s beacon reboot fires on an HTTP-only client.** `BEACON_TIMEOUT` restarts the board when no UDP beacon has arrived, and a PC watching MJPEG through `run.py` sends none — so the board rebooted every two minutes, on a fresh DHCP lease, for no visible reason. The stream handler now refreshes `lastServerContact` on every frame: an open stream *is* contact.
- **The sensor power-down also keys on the UDP beacon.** `cameraDrainTask` puts the OV2640 to sleep when `pcConnected` is false, which an HTTP-only session never sets. Gating the whole task on `video_udp_mode` handles this too — the cost is that MJPEG mode has no sensor power saving at all.
- **`--source` no longer implies a sketch, and the defaults moved with that.** `autodrive.py` and `capture_esp32_samples.py` now default to `webserver` alongside `run.py`. `autodrive.py` still needs `--esp-ip` to arm — control goes out over UDP regardless of how video arrives.

### Camera setup

- **`CameraWebServer` boots at QVGA** — the sketch drops it there right after init for speed — and stays there unless something asks for more. Skipping `prepare_camera()` gets you a soft 320×240 picture, and since `min_contour_area` is in absolute pixels the detector then stops seeing cones. Set the resolution **before** opening the stream: the camera serves one client, and the `/capture` that verifies the change cannot run while the stream holds the slot.
- **Lock AWB before tuning HSV.** Auto white balance works by changing the blue and red channel gains, which is exactly the axis a blue cone sits on. `camera_control.lock_image_settings()` turns off `awb`, `awb_gain`, `aec`, `aec2` and `agc` — all five, since they are separate loops — after a 2 s settle so it freezes a properly adjusted image.
- **`ae_level` must be written before the AEC lock**, or it is frozen out and does nothing. `-1` keeps the shutter short: left alone in a dim room AEC picks an exposure longer than the frame period, the OV2640 pads with dummy lines, and the *sensor's own* framerate collapses with the WiFi still idle.
- **`min_contour_area` is in absolute pixels**, so it is tied to whatever resolution the camera is sending. 1500 px is ~0.5 % of a VGA frame but ~2 % of QVGA; re-tune after changing `--framesize`.
- **Detection needs BGR straight from `cv2.imdecode`** — not the RGB/`rot90` surface the legacy pygame display path builds. RGB swaps red and blue, turning a blue cone into an orange one.
- **Don't hardcode framesize enum indices.** `val` in `/control?var=framesize&val=N` is a raw index into `framesize_t`, and the esp32 core here (4.0.0-alpha1) inserted `128X128` and `320X320`, so VGA is 10 and QVGA is 6, not the 8 quoted online. `camera_control.py` probes and verifies via `/capture`.
- **The ESP32-CAM serves one stream client.** A browser tab left open on the camera page means Python gets no frames at all. The control server (port 80) and the stream server (port 81) are separate httpd instances, so `/status` still answers while the stream runs — which is what makes live diagnosis possible.
- **`updateCameraSettings()` was never called from `initCamera()`.** All the deliberate sensor settings — `ae_level -1`, gain ceiling, lens/black-pixel correction, gamma — only applied once a framesize command or the adaptive loop happened to fire. Fixed in `V2/firmware/esp32cam_robot/`; reflash to get it.

### Diagnosing a slow or stuttering link

Average fps hides stutter: a stream delivering frames in pairs with long holes averages the same as a steady one. `run.py --bench` reports frame spacing (median / 95th / worst) and classifies the result, because the three faults need opposite fixes:

| shape | means | fix |
|---|---|---|
| slow, plenty of bytes | genuinely bandwidth-bound | `--quality 16-18`, smaller `--framesize` |
| slow, few bytes, **even** spacing | the camera is not producing frames | `--ae-level -1` first, then `--quality`; try `--no-lock` |
| slow, few bytes, **long holes** | radio link, TCP backing off | check `ping` per-line, antenna jumper, distance |

Compressing harder does nothing for the last two — the frames are not too big.

**The verdict line can point at the sensor when the fault is the radio.** Do not act on one `--bench` run. Three cheap measurements separate the two for certain, and all three were needed on the case below:

1. **Sweep the config and watch KB/s, not fps.** Run `--bench` at VGA q12, QVGA q20, QVGA q24, QVGA q32. If fps climbs while **KB/s stays flat**, the link is capped and every "improvement" is only smaller frames. A sensor that cannot produce frames shows the opposite: KB/s falls with the frame size while fps holds.
2. **Sweep `aec_value` by hand** (`aec`/`aec2`/`agc` → 0, then `aec_value` 490 → 30) and time `/capture` at each. If frame time does not move, the "long exposure pads dummy lines" theory is dead — regardless of what the verdict says.
3. **Compare `/status` against `/capture`.** `/status` is ~1.4 KB of JSON that costs the board nothing to produce. If `/status` returns in ~0.15 s while a 3 KB `/capture` takes ~1 s, the picture path is the slow one. If both crawl, it is the link.

**Then prove whose radio.** `ping` the router and the board from the same PC in the same minute. One measured case: PC→router 5 ms avg on a 286 Mbps 802.11ax link at 86% signal, PC→ESP32 **46 ms avg with spikes to 178 ms and 0% loss**, throughput pinned at **8–12 KB/s no matter what** (0.7 fps at VGA q12, 5.0 fps at QVGA q32, same bytes/s). Same room, same AP: the board's radio was the bottleneck, not the PC, not the sensor, not the Python. `WiFi.setSleep(false)` was already in the sketch, so power save was not it either.

At that point the fix is physical, in this order: the **antenna jumper** (see below), then the **power supply** (5 V ≥1 A, short thick cable — the radio browns out before anything else does), then **distance**, then the AP's 2.4 GHz channel. Replacing the board is the last step, not the first: put it within a metre of the router and re-bench — if KB/s jumps, the board is fine.

- **`ping` on Windows lies in its summary.** When the host is gone your own machine answers `Destination host unreachable`, and those count as "received, 0% loss". Read the per-reply lines. A healthy local WiFi link sits at 2–10 ms; one measured case ran min 10 / avg 72 / max 530 ms with 4% loss, which is enough for TCP to collapse to ~9 KB/s.
- **Reboot or radio?** Write a marker the firmware only clears on boot (`/control?var=special_effect&val=2`) and poll `/status` while streaming. Marker disappears (settings back to QVGA + auto) → the board is resetting, suspect power. Marker survives but fps stays low → the radio link.
- **AI-Thinker boards select PCB or u.FL antenna with a solder jumper.** Set to u.FL with nothing attached, the board still associates normally but throughput collapses.
- **The robot firmware reboots itself after 120 s with no beacon** and comes back on a different DHCP lease. An IP that worked ten minutes ago is not the IP now — re-check before concluding the board is dead.

## The fork situation

Divergent copies of the same programs, with no shared library — a fix must be applied per folder, and merging naively loses whichever branch you did not start from. This is the repo's main hazard; do not add another fork without a reason.

| Folder | What it is |
|---|---|
| `V3/` | a fresh video-only rewrite (`run.py`, `camera.py`, `mjpeg_source.py`), superseded. `project 101/` was seeded from it and is likewise dead — neither is a branch to merge from |
| `V2/` | **current work.** The pipeline above — the only folder that can drive |
| `backup/` | **verified 2026-09-06: a byte-identical copy of `V2/`**, not the vision-only variant this file used to claim. It still has `autodrive.py`, `autopilot.py`, `robot_link.py` and `firmware/`, so it CAN drive — do not run it beside an assembled car expecting it to be safe. It is a plain snapshot, and it predates the MJPEG switchover |
| `V2-vision-only/` | an older, pre-driving snapshot of `V2/`. Superseded by `backup/` and missing everything from `camera_setup.py` onwards |
| `_archive_cone_from_V2/` | the orange-cone detector `V2/` replaced |
| `Lats-version/` | pre-V2 best: pygame controller with card HUD, cached scaling; firmware with VGA q12, 20 MHz XCLK, `fb_count=2`, chunk-yield TX. `V2/firmware/esp32cam_robot/` is copied from here |
| `Cone_Dectact/` | old pygame controller with autonomous mode bolted in; old firmware (QQVGA q40, `millis()` seq) |
| `Test/` | pre-V2 tooling: the original `esp32_video_source.py`, capture, ESP32-aware calibrator |
| `Aj-Piya_code/` | the original baseline. `robot_link.py` is derived from its `2026_07_13.py` |
| `PND-Code/` | side experiment: FSRCNN x2 / Upscayl upscaling. Its `TestWeb/` is where the CameraWebServer tooling in `V2/` came from |
| `CameraWebServer/` | stock Arduino example, kept for reference — different WiFi credentials, and its `camera_pins.h` differs from the robot copies |
| `try/` | a re-extracted `V2-vision-only.zip` and its unpacked copy. Scratch, not a branch to merge from |

**The old fix table in this file was wrong.** It claimed `backup/` (and a since-deleted `project 101/`) carried fixes that `V2/` lacked. `diff -rq V2 backup` on 2026-09-06 showed the two folders identical apart from the firmware sketch: **neither copy had the `read1()` fix or the `--bench` frame-spacing work.** Verify a claim like that with `diff` before porting anything on the strength of it.

Current state of `V2/`:

| Fix | `V2/` |
|---|---|
| `read1()` MJPEG fix | **yes — written 2026-09-06**, measured 0–3 ms vs 251–754 ms for `read()` |
| `frame_count` gating + one `handle_key()` on both loop paths | yes |
| framesize enum probed and verified via `/capture` | yes |
| delivery measurement, reboot resync (UDP path only) | yes |
| `set_robot_framesize()` on `--source esp32` | yes |
| `--bench` frame spacing, 3-way verdict | **no** — `--bench` still reports average fps only |


Two `cone_config.json` files in the pre-V2 folders disagree: `Cone_Dectact/` holds H 5–24 (orange cone), `Test/` holds H 12–101 / S min 0, which matches nearly anything non-gray and looks like a saved intermediate state. Neither is relevant to `V2/`, which is blue (H 90–130).

## Working language

The user writes in Thai; reply in Thai. `V2/README.md`, `backup/README.md`, `V3/README.md` and `V2-vision-only/README.md` are in Thai. Keep filenames, identifiers and code comments in English.
