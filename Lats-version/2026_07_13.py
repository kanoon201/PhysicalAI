import pygame
import socket
import time
import threading
import cv2
import numpy as np
import sys
import queue

# ==========================================
# CONFIGURATION & TUNING
# ==========================================

# --- 1. NETWORK PORTS ---
HOST_IP = "0.0.0.0"          # Listen on all interfaces
VIDEO_PORT = 44442           # UDP Port for Video (from ESP32)
CONTROL_PORT = 44441         # UDP Port for Control
DISCOVERY_PORT = 44444       # UDP Port for Discovery

# --- 2. PERFORMANCE & TIMING ---
DISCOVERY_INTERVAL = 0.5     # Seconds between beacons
WATCHDOG_TIMEOUT = 1.5       # Reset connection if no data for 1.5s
COMMAND_TX_INTERVAL = 0.01   # Loop speed for command sender
TELEMETRY_RX_TIMEOUT = 0.5   # Socket timeout for telemetry
RECV_BUFFER_SIZE = 65536     # UDP Receive Buffer size
VIDEO_QUEUE_SIZE = 1         # Latest-only video queue
COMMAND_QUEUE_SIZE = 5       # Buffer for control packets
FPS_UPDATE_INTERVAL = 1.0    # Calculate FPS every 1s

# --- 3. GUI & VIDEO ---
WINDOW_RES = (1500, 950)     # Pygame window size
UI_MARGIN = 14               # Gap between layout blocks
SIDEBAR_W = 340              # Telemetry column on the right
HEADER_H = 56                # Top status strip
FOOTER_H = 34                # Keybind hint strip
PANEL_HEIGHT = 250           # (legacy, unused by the current layout)
MIN_JPG_SIZE = 1000          # Ignore tiny/corrupt frames

# --- PALETTE ---
C_BG      = (11, 13, 17)
C_PANEL   = (21, 25, 32)
C_TRACK   = (33, 39, 49)     # unfilled portion of meters
C_BORDER  = (44, 51, 63)
C_TEXT    = (233, 237, 243)
C_DIM     = (124, 134, 149)
C_GREEN   = (0, 219, 130)
C_AMBER   = (255, 186, 60)
C_RED     = (255, 88, 88)
C_BLUE    = (86, 170, 255)
C_VIOLET  = (167, 139, 250)
SEQ_LIMIT = 10000            # Wrap-around limit matching ESP32
SEQ_CLOSE_UPPER = 9000
SEQ_CLOSE_LOWER = 1000
HUD_ALPHA = 255              # Solid panel background

# --- 4. ROBOT CONFIG ---
MAX_LIGHT = 65535
PRESSTIME = 0.2              # Debounce for button presses
DEFAULT_ESP_IP = "192.168.8.191" # Robot default IP for cold starts

# --- GLOBALS ---
SCREEN_WIDTH, SCREEN_HEIGHT = WINDOW_RES
VIDEO_RES = (1024, 768)
TARGET_ESP_IP = None 
robot_connected = False
running = True
myimage = None
myimage_seq = 0              # bumped per decoded frame; used to cache display scaling
fps = 0
lastimg = time.time()
last_heartbeat = time.time()
command_queue = queue.Queue(maxsize=COMMAND_QUEUE_SIZE)
telemetry_data = {"RSSI": "0", "RobotIP": "N/A", "ChipID": "Unknown", "LastSeq": 4294967295, "KBPS": 0, "ProcLag": 0, "NetLag": 0, "BufSize": 0, "SrcRes": None}
raw_video_queue = queue.Queue(maxsize=VIDEO_QUEUE_SIZE)
show_hud = True
last_beacon_time = 0
last_telemetry_time = 0
last_tx_time = 0
current_control_seq = 0
control_synced = False 
last_light_sync = 0
last_light_val = -1
last_drive_msg = [0, 0]
last_drive_time = 0

# NEW: Delivery tracking for Adaptive Quality on ESP32
stream_delivery_pct = 100.0  

def send_command(target_ip, port, cmd, v1, v2, force=False):
    global current_control_seq, last_drive_msg, last_drive_time, last_light_val, last_light_sync
    
    now = time.time()
    should_send = force
    burst = 1
    
    if cmd == 0: # Drive Command
        if [v1, v2] != last_drive_msg or (now - last_drive_time > 0.05):
            should_send = True
            last_drive_msg = [v1, v2]
            last_drive_time = now
    elif cmd == 1: # Light Command
        if v1 != last_light_val or (now - last_light_sync > 1.0):
            should_send = True
            burst = 2
            last_light_val = v1
            last_light_sync = now
    else: # System Commands
        should_send = True
        burst = 2

    if should_send:
        try:
            for _ in range(burst):
                msg = f"{current_control_seq},{cmd},{v1},{v2}".encode()
                command_queue.put_nowait((target_ip, port, msg))
                current_control_seq = (current_control_seq + 1) % SEQ_LIMIT
        except queue.Full:
            pass

# --- HELPER FUNCTIONS ---
def get_rssi_quality(rssi_str):
    try:
        val = int(rssi_str)
        if val >= -50: return "Excellent (Strong signal)", (0, 255, 0)
        if val >= -70: return "Good (Stable)", (200, 255, 0)
        return "Weak (Lag possible)", (255, 50, 50)
    except (ValueError, TypeError):
        return "Unknown", (150, 150, 150)

def get_fps_quality(fps_val):
    if fps_val >= 20: return (0, 255, 0)      
    if fps_val >= 10: return (255, 255, 0)    
    return (255, 50, 50)                     

def ev(v):
    hw = (2000 - 1000) * 0.5
    return 1000 + int(hw + (v * hw))

# ==========================================
# THREAD 1: SMART DISCOVERY (BROADCAST)
# ==========================================
def discovery_beacon():
    global TARGET_ESP_IP, robot_connected, last_heartbeat, last_beacon_time, control_synced, stream_delivery_pct
    print(f" [Thread] Smart Beacon Started on Port {DISCOVERY_PORT}")
    
    while running:
        if robot_connected and (time.time() - last_heartbeat > WATCHDOG_TIMEOUT):
            print(f" [Watchdog] Connection lost for {WATCHDOG_TIMEOUT}s. Resetting...")
            robot_connected = False
            control_synced = False 
        
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                last_beacon_time = time.time()
                s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                
                # INJECT ADAPTIVE DELIVERY PERCENTAGE
                msg = f"PC_HERE,delivery:{int(stream_delivery_pct)}".encode()
                
                try: s.sendto(msg, ('<broadcast>', DISCOVERY_PORT))
                except: pass
                
                if TARGET_ESP_IP and TARGET_ESP_IP != DEFAULT_ESP_IP:
                    s.sendto(msg, (TARGET_ESP_IP, DISCOVERY_PORT))
                
                try:
                    temp_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    temp_s.connect(("8.8.8.8", 80))
                    local_ip = temp_s.getsockname()[0]
                    temp_s.close()
                    
                    prefix = ".".join(local_ip.split(".")[:-1])
                    for i in range(255): 
                        s.sendto(msg, (f"{prefix}.{i}", DISCOVERY_PORT))
                        time.sleep(0.02)
                except Exception: pass
                
        except: pass
        time.sleep(DISCOVERY_INTERVAL)

# ==========================================
# THREAD: VIDEO PROCESSING
# ==========================================
def video_processor():
    global myimage, myimage_seq, lastimg, fps, running, flip, flip2
    
    fps_counter = 0
    fps_timer = time.time()
    decode_times = []
    
    cv2.setUseOptimized(True)
    if hasattr(cv2, 'setNumThreads'):
        cv2.setNumThreads(4)

    while running:
        try:
            item = raw_video_queue.get(timeout=0.01)
            frame_data, arrival_time = item
            
            t_start = time.time()
            np_arr = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:   
                if flip: frame = cv2.flip(frame, 1)
                if flip2: frame = cv2.flip(frame, 0)
                
                # Report what the camera actually sent, before any scaling.
                telemetry_data['SrcRes'] = (frame.shape[1], frame.shape[0])

                # Everything below runs at NATIVE resolution. Scaling here meant
                # cvtColor/rot90/make_surface all chewed through the upscaled
                # array (~3x the pixels) on every single frame, in the decode
                # thread, directly adding latency. The display scale happens once
                # in the draw loop instead, and is cached between frames.
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = np.rot90(frame)

                myimage = pygame.surfarray.make_surface(frame)
                myimage_seq += 1
                lastimg = time.time()
                
                t_end = time.time()
                decode_times.append(t_end - t_start)
                if len(decode_times) > 30: decode_times.pop(0)
                
                telemetry_data['ProcLag'] = (sum(decode_times) / len(decode_times)) * 1000
                telemetry_data['NetLag'] = (t_start - arrival_time) * 1000
                
                fps_counter += 1
                if time.time() - fps_timer >= FPS_UPDATE_INTERVAL:
                    fps = fps_counter
                    fps_counter = 0
                    fps_timer = time.time()

        except queue.Empty: continue
        except Exception as e: 
            print(f" [VideoProc] Error: {e}")
            time.sleep(0.001)

# ==========================================
# THREAD 2: VIDEO SERVER (UDP Receiver)
# ==========================================
def video_server():
    global TARGET_ESP_IP, robot_connected, running, last_heartbeat, stream_delivery_pct

    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, 'SO_REUSEPORT'):
        try: srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except: pass
    
    try: srv.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2097152) 
    except: pass

    try:
        srv.bind((HOST_IP, VIDEO_PORT))
        srv.settimeout(0.5) 
    except Exception as e:
        print(f" [Video] Bind Error: {e}")
        return
    
    print(f" [Video] UDP LISTENING on port {VIDEO_PORT}")

    frames = {}
    last_processed_seq = 0
    was_disconnected = True

    while running: 
        try:
            data, addr = srv.recvfrom(2048)
            if not data: continue
            
            now = time.time()
            TARGET_ESP_IP = addr[0]
            
            if not robot_connected or was_disconnected:
                last_processed_seq = 0
                frames.clear()
                was_disconnected = False
                stream_delivery_pct = 100.0
                
                global current_control_seq, control_synced
                current_control_seq = 0
                control_synced = True

            robot_connected = True
            last_heartbeat = now
            
            if len(data) < 8: continue
            
            seq = int.from_bytes(data[0:4], 'little') 
            total = int.from_bytes(data[4:6], 'little')
            chunk_id = int.from_bytes(data[6:8], 'little')
            payload = data[8:]
            
            if seq < last_processed_seq and (last_processed_seq - seq < 5000):
                continue
                
            if seq not in frames:
                if len(frames) > 0:
                    current_keys = list(frames.keys())
                    for k in current_keys:
                        if k < seq: del frames[k]
                frames[seq] = {'total': total, 'chunks': {}, 'last_update': now}
            
            frames[seq]['chunks'][chunk_id] = payload
            frames[seq]['last_update'] = now
            
            if len(frames[seq]['chunks']) == total:
                chunks_dict = frames[seq]['chunks']
                try:
                    full_frame = b"".join([chunks_dict[i] for i in range(total)])
                    
                    if full_frame.startswith(b'\xff\xd8') and full_frame.endswith(b'\xff\xd9'):
                        if seq > last_processed_seq or (seq < 1000 and last_processed_seq > 4000000000):
                            
                            # --- ADAPTIVE QUALITY: CALCULATE DELIVERY ---
                            if last_processed_seq > 0:
                                gap = seq - last_processed_seq - 1
                                if gap <= 0:
                                    # Perfect delivery
                                    stream_delivery_pct = min(100.0, (stream_delivery_pct * 0.95) + 5.0)
                                else:
                                    # Dropped frames, penalize delivery score
                                    batch_delivery = (1.0 / (gap + 1)) * 100.0
                                    stream_delivery_pct = (stream_delivery_pct * 0.8) + (batch_delivery * 0.2)
                            
                            last_processed_seq = seq
                            try:
                                while not raw_video_queue.empty():
                                    raw_video_queue.get_nowait()
                            except: pass
                            raw_video_queue.put_nowait((full_frame, now))
                            
                            telemetry_data['KBPS'] = (len(full_frame) * fps) / 1024 if fps > 0 else 0
                            telemetry_data['BufSize'] = len(frames)
                except KeyError:
                    pass
                
                del frames[seq]

        except socket.timeout:
            if robot_connected and (time.time() - last_heartbeat > WATCHDOG_TIMEOUT):
                print(f" [Video] No video for {WATCHDOG_TIMEOUT}s. Marking as disconnected.")
                robot_connected = False
                was_disconnected = True 
            continue
        except Exception:
            if running: time.sleep(0.001)

    srv.close()
    print(" [Video] UDP Server Stopped.")

# ==========================================
# THREAD 3: CONTROL SENDER
# ==========================================
def control_sender():
    global last_tx_time
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while running:
        try:
            item = command_queue.get(timeout=COMMAND_TX_INTERVAL)
            
            batch = [item]
            while not command_queue.empty():
                try: batch.append(command_queue.get_nowait())
                except queue.Empty: break
            
            final_send = []
            latest_drive = None
            
            for tip, p, data in batch:
                try:
                    cmd_type = int(data.decode('utf-8').split(',')[1])
                    if cmd_type == 0:
                        latest_drive = (tip, p, data) 
                    else:
                        final_send.append((tip, p, data)) 
                except:
                    final_send.append((tip, p, data)) 
            
            if latest_drive:
                final_send.append(latest_drive)
            
            for tip, p, data in final_send:
                udp_sock.sendto(data, (tip, p))
                last_tx_time = time.time()
            
            for _ in range(len(batch)):
                command_queue.task_done()
                
        except queue.Empty: pass
        except Exception: 
            if running: time.sleep(0.01)

# ==========================================
# THREAD 4: TELEMETRY RECEIVER
# ==========================================
def telemetry_receiver():
    global telemetry_data, last_heartbeat, last_telemetry_time, robot_connected
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, 'SO_REUSEPORT'):
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        s.bind((HOST_IP, DISCOVERY_PORT))
        s.settimeout(TELEMETRY_RX_TIMEOUT)
        print(f" [Thread] Telemetry Receiver Started on Port {DISCOVERY_PORT}")
        while running:
            try:
                data, addr = s.recvfrom(1024)
                if data:
                    msg = data.decode('utf-8', errors='ignore')
                    if msg.startswith("PC_HERE"): continue
                    
                    last_heartbeat = time.time()
                    robot_connected = True 
                    last_telemetry_time = time.time()
                    
                    if TARGET_ESP_IP and addr[0] != TARGET_ESP_IP:
                        continue
                    
                    try:
                        parts = msg.split(',')
                        if len(parts) >= 2:
                            seq = int(parts[0])
                            chipid = parts[1]
                            
                            last_seq = telemetry_data["LastSeq"]
                            if seq > last_seq or (last_seq > SEQ_CLOSE_UPPER and seq < SEQ_CLOSE_LOWER):
                                telemetry_data["LastSeq"] = seq
                                telemetry_data["ChipID"] = chipid
                                telemetry_data["RobotIP"] = addr[0]
                                
                                if len(parts) >= 3 and "RSSI:" in parts[2]:
                                    telemetry_data["RSSI"] = parts[2].split("RSSI:")[1]
                                elif "RSSI:" in msg:
                                    telemetry_data["RSSI"] = msg.split("RSSI:")[1]
                        else:
                            telemetry_data["RobotIP"] = addr[0]
                            if "RSSI:" in msg:
                                telemetry_data["RSSI"] = msg.split("RSSI:")[1]
                    except: pass
            except socket.timeout: continue
            except Exception: time.sleep(0.1)

# ==========================================
# DRAWING PRIMITIVES
# ==========================================
def rrect(surf, rect, color, radius=10, width=0):
    pygame.draw.rect(surf, color, rect, width, border_radius=radius)


def draw_card(surf, rect):
    """Panel background + hairline border."""
    rrect(surf, rect, C_PANEL, 10)
    rrect(surf, rect, C_BORDER, 10, 1)


def draw_meter(surf, x, y, w, h, frac, color):
    """Horizontal 0..1 fill bar."""
    frac = max(0.0, min(1.0, frac))
    rrect(surf, (x, y, w, h), C_TRACK, h // 2)
    if frac > 0:
        fill_w = max(h, int(w * frac))
        rrect(surf, (x, y, fill_w, h), color, h // 2)


def draw_bipolar(surf, x, y, w, h, val, color):
    """Center-origin bar for steering. val in -1..1."""
    val = max(-1.0, min(1.0, val))
    rrect(surf, (x, y, w, h), C_TRACK, h // 2)
    cx = x + w // 2
    if abs(val) > 0.01:
        span = int((w // 2) * abs(val))
        bx = cx if val > 0 else cx - span
        rrect(surf, (bx, y, max(3, span), h), color, h // 2)
    pygame.draw.line(surf, C_DIM, (cx, y - 2), (cx, y + h + 2), 1)


def draw_signal(surf, x, y, rssi_str):
    """Cellular-style 4-bar signal strength."""
    try:
        rssi = int(rssi_str)
    except (ValueError, TypeError):
        rssi = 0
    # 0 is the "no reading yet" default, not a perfect signal. Real WiFi RSSI
    # is always negative, so treat >= 0 as unknown.
    if rssi >= 0:     lit, col = 0, C_DIM
    elif rssi >= -55: lit, col = 4, C_GREEN
    elif rssi >= -67: lit, col = 3, C_GREEN
    elif rssi >= -78: lit, col = 2, C_AMBER
    elif rssi >= -88: lit, col = 1, C_RED
    else:             lit, col = 0, C_RED
    for i in range(4):
        bh = 6 + i * 4
        bx = x + i * 8
        by = y + (18 - bh)
        pygame.draw.rect(surf, col if i < lit else C_TRACK,
                         (bx, by, 5, bh), border_radius=2)
    return col


def draw_stat(surf, font_lbl, font_val, x, y, label, value, color, w):
    """Label above a large value, used in the sidebar grid."""
    surf.blit(font_lbl.render(label, True, C_DIM), (x, y))
    surf.blit(font_val.render(value, True, color), (x, y + 15))


def draw_pill(surf, font, x, y, text, color):
    """Small rounded status chip. Returns its width."""
    tw = font.size(text)[0]
    w = tw + 18
    rrect(surf, (x, y, w, 22), color, 11)
    surf.blit(font.render(text, True, C_BG), (x + 9, y + 4))
    return w


# ==========================================
# MAIN GUI
# ==========================================
def main():
    global running, flip, flip2, show_hud, VIDEO_RES, SCREEN_WIDTH, SCREEN_HEIGHT, last_light_sync, current_control_seq, last_drive_time, last_drive_msg, last_light_val, robot_connected, TARGET_ESP_IP, control_synced
    
    flip = False
    flip2 = False

    STREAM_AREA_HEIGHT = SCREEN_HEIGHT - HEADER_H - FOOTER_H - (2 * UI_MARGIN)
    STREAM_AREA_WIDTH = SCREEN_WIDTH - SIDEBAR_W - (3 * UI_MARGIN)

    aspect = 4/3
    final_h = STREAM_AREA_HEIGHT
    final_w = int(final_h * aspect)
    if final_w > STREAM_AREA_WIDTH:
        final_w = STREAM_AREA_WIDTH
        final_h = int(final_w / aspect)
    VIDEO_RES = (final_w, final_h)

    threading.Thread(target=discovery_beacon, daemon=True).start()
    threading.Thread(target=video_server, daemon=True).start()
    threading.Thread(target=video_processor, daemon=True).start()
    threading.Thread(target=control_sender, daemon=True).start()
    threading.Thread(target=telemetry_receiver, daemon=True).start()

    pygame.init()
    screen = pygame.display.set_mode(WINDOW_RES, pygame.RESIZABLE)
    pygame.display.set_caption("ESP32-CAM WiFi Car Controller")
    clock = pygame.time.Clock()
    
    MONO = "menlo,monaco,dejavusansmono,consolas,couriernew"
    font = pygame.font.SysFont(None, 32, bold=False)          # kept for compat
    font_small = pygame.font.SysFont(MONO, 13)
    font_lbl = pygame.font.SysFont(None, 17, bold=True)        # uppercase labels
    font_val = pygame.font.SysFont(MONO, 25, bold=True)        # big numbers
    font_hdr = pygame.font.SysFont(None, 25, bold=True)        # header text
    font_body = pygame.font.SysFont(MONO, 15)

    prev_msg = [ev(0), ev(0)]
    previous_press = 0
    scaled_cache = (None, None)   # ((frame_seq, target_size), scaled_surface)
    
    factor = 0.35
    maxfactor = 1.0
    TURN = 1
    TURNFACTOR = 1.0
    TURNTRIM = 0
    SPEEDTRIM = 0
    light = 0
    
    while running:
        SCREEN_WIDTH, SCREEN_HEIGHT = screen.get_size()
        # HUD off => video takes the whole window.
        sidebar_w = SIDEBAR_W if show_hud else 0
        top_h = HEADER_H if show_hud else 0
        bot_h = FOOTER_H if show_hud else 0
        gaps = 3 if show_hud else 2

        STREAM_AREA_HEIGHT = SCREEN_HEIGHT - top_h - bot_h - (2 * UI_MARGIN)
        STREAM_AREA_WIDTH = SCREEN_WIDTH - sidebar_w - (gaps * UI_MARGIN)

        aspect = 4/3
        final_h = max(100, STREAM_AREA_HEIGHT)
        final_w = int(final_h * aspect)
        if final_w > STREAM_AREA_WIDTH:
            final_w = max(100, STREAM_AREA_WIDTH)
            final_h = int(final_w / aspect)
        VIDEO_RES = (final_w, final_h)

        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            elif event.type == pygame.VIDEORESIZE:
                screen = pygame.display.set_mode(event.size, pygame.RESIZABLE)

        currentpress = time.time()
        sx, sy = 0, 0
        message = [0, 0]
        
        keys = pygame.key.get_pressed()
        
        if pygame.joystick.get_count() > 0:
            joy = pygame.joystick.Joystick(0); joy.init()
            sx = joy.get_axis(2)   
            sy = -joy.get_axis(1)  
            l_val = (joy.get_axis(4)+1)*0.5
            r_val = (joy.get_axis(5)+1)*0.5
            sy += l_val 
            sy += -r_val  
            if joy.get_button(9):  sx = -0.4
            if joy.get_button(10):  sx = 0.4
            if joy.get_button(11) == 1: sy = 1
            if joy.get_button(12) == 1: sy = -1
            if joy.get_button(13) == 1: sx = -1
            if joy.get_button(14) == 1: sx = 1
            if joy.get_button(3): light = min(MAX_LIGHT, light+1000)
            if joy.get_button(2): light = max(0, light-1000)
            if joy.get_button(0) or joy.get_button(1):
                factor = 0.3; TURNTRIM = 0; SPEEDTRIM = 0
            if joy.get_button(4) and currentpress-previous_press > (PRESSTIME*0.5):
                previous_press = currentpress; factor = min(1, factor+0.05)
            if joy.get_button(6) and currentpress-previous_press > (PRESSTIME*0.5):
                previous_press = currentpress; factor = max(0, factor-0.05)

        if keys[pygame.K_UP]: sy = 1
        elif keys[pygame.K_DOWN]: sy = -1
        if keys[pygame.K_LEFT]: sx = -1
        elif keys[pygame.K_RIGHT]: sx = 1

        if keys[pygame.K_MINUS] and currentpress-previous_press > PRESSTIME: 
            previous_press = currentpress; factor = max(0, factor-0.05)
        if keys[pygame.K_EQUALS] and currentpress-previous_press > PRESSTIME: 
            previous_press = currentpress; factor = min(maxfactor, factor+0.05)
        
        if keys[pygame.K_5] and currentpress-previous_press > PRESSTIME:
            previous_press = currentpress; TURN *= -1
        if keys[pygame.K_6]: TURNTRIM -= 0.001
        if keys[pygame.K_7]: TURNTRIM += 0.001
        if keys[pygame.K_8]: SPEEDTRIM -= 0.001
        if keys[pygame.K_9]: SPEEDTRIM += 0.001
        if keys[pygame.K_0]: factor = 0.3; TURNTRIM = 0; SPEEDTRIM = 0

        if keys[pygame.K_f] and currentpress-previous_press > PRESSTIME:
            previous_press = currentpress; flip = not flip
        if keys[pygame.K_g] and currentpress-previous_press > PRESSTIME:
            previous_press = currentpress; flip2 = not flip2
        if keys[pygame.K_h] and currentpress-previous_press > PRESSTIME:
            previous_press = currentpress; show_hud = not show_hud
            
        if keys[pygame.K_l]: light = min(MAX_LIGHT, light+2000)
        if keys[pygame.K_k]: light = max(0, light-2000)

        sy = (sy * factor) + SPEEDTRIM
        sx = (sx * TURNFACTOR) + TURNTRIM
        final_turn = sx * TURN 
        
        sy = round(sy, 3)
        final_turn = round(final_turn, 3)
        outmessage = [ev(final_turn), ev(sy)]

        active_ip = telemetry_data.get("RobotIP")
        target_ip = TARGET_ESP_IP if TARGET_ESP_IP else (active_ip if active_ip not in ["N/A", None] else DEFAULT_ESP_IP)
        
        valid_target = target_ip and target_ip not in ["N/A", "SEARCHING..."]
        
        if valid_target:
            send_command(target_ip, CONTROL_PORT, 1, int(light), int(light))
            
            if keys[pygame.K_r] and currentpress-previous_press > PRESSTIME:
                 previous_press = currentpress
                 print(" [System] Reboot Command Sent. Resetting Link.")
                 send_command(target_ip, CONTROL_PORT, 3, 3, 3)
                 robot_connected = False
                 control_synced = False
            if keys[pygame.K_q] and currentpress-previous_press > PRESSTIME:
                 previous_press = currentpress
                 send_command(target_ip, CONTROL_PORT, 4, 4, 4)
            if keys[pygame.K_w] and currentpress-previous_press > PRESSTIME:
                 previous_press = currentpress
                 send_command(target_ip, CONTROL_PORT, 5, 5, 5)
            if keys[pygame.K_e] and currentpress-previous_press > PRESSTIME:
                 previous_press = currentpress
                 send_command(target_ip, CONTROL_PORT, 6, 6, 6)
            if keys[pygame.K_t] and currentpress-previous_press > PRESSTIME:
                 previous_press = currentpress
                 send_command(target_ip, CONTROL_PORT, 7, 7, 7)

            send_command(target_ip, CONTROL_PORT, 0, outmessage[0], outmessage[1])

        screen.fill(C_BG)

        video_x = UI_MARGIN
        video_y = (HEADER_H + UI_MARGIN) if show_hud else UI_MARGIN

        # --- VIDEO ---
        stage = pygame.Rect(video_x, video_y, STREAM_AREA_WIDTH, STREAM_AREA_HEIGHT)
        draw_card(screen, stage)
        if myimage:
            # Fit the native frame into the stage, then cache the scaled copy:
            # frames arrive at ~15fps but the loop draws at 60, so without this
            # we would rescale the same image four times over for nothing.
            src = myimage
            sw_, sh_ = src.get_size()
            fit = min(stage.w / sw_, stage.h / sh_)
            tgt = (max(1, int(sw_ * fit)), max(1, int(sh_ * fit)))
            key = (myimage_seq, tgt)
            if scaled_cache[0] != key:
                scaled_cache = (key, pygame.transform.smoothscale(src, tgt)
                                if tgt != (sw_, sh_) else src)
            shown = scaled_cache[1]
            x_pos = stage.x + (stage.w - shown.get_width()) // 2
            y_pos = stage.y + (stage.h - shown.get_height()) // 2
            screen.blit(shown, (x_pos, y_pos))
        else:
            msg = "WAITING FOR VIDEO" if robot_connected else "SEARCHING FOR ROBOT"
            surf = font_hdr.render(msg, True, C_DIM)
            screen.blit(surf, (stage.centerx - surf.get_width() // 2,
                               stage.centery - surf.get_height() // 2))

        if not show_hud:
            pygame.display.flip()
            clock.tick(60)
            continue

        total_latency = (time.time() - lastimg) * 1000
        net_lag = telemetry_data.get('NetLag', 0)
        proc_lag = telemetry_data.get('ProcLag', 0)
        rssi_txt, _ = get_rssi_quality(telemetry_data['RSSI'])
        fps_col = get_fps_quality(fps)
        lag_col = C_GREEN if total_latency < 120 else (C_AMBER if total_latency < 250 else C_RED)
        del_col = C_GREEN if stream_delivery_pct > 80 else (C_AMBER if stream_delivery_pct > 50 else C_RED)
        now_t = time.time()

        # ==================== HEADER ====================
        hdr = pygame.Rect(UI_MARGIN, UI_MARGIN, SCREEN_WIDTH - 2 * UI_MARGIN, HEADER_H - UI_MARGIN)
        draw_card(screen, hdr)
        hx = hdr.x + 16
        hy = hdr.y + (hdr.h - 22) // 2

        hx += draw_pill(screen, font_lbl, hx, hy,
                        "LINKED" if robot_connected else "SEARCHING",
                        C_GREEN if robot_connected else C_RED) + 16

        ident = (f"{telemetry_data['RobotIP']}  ·  {telemetry_data['ChipID']}"
                 if robot_connected else f"probing {target_ip}")
        screen.blit(font_body.render(ident, True, C_TEXT), (hx, hy + 4))
        hx += font_body.size(ident)[0] + 20

        # What the sensor actually sent -> what we scaled it to. This is how you
        # confirm a resolution preset really took effect on the ESP32.
        src = telemetry_data.get('SrcRes')
        if src:
            scale = "upscaled" if VIDEO_RES[0] > src[0] else "downscaled"
            res_txt = f"SRC {src[0]}x{src[1]}  ({scale} to {VIDEO_RES[0]}x{VIDEO_RES[1]})"
            screen.blit(font_body.render(res_txt, True, C_DIM), (hx, hy + 4))

        # Activity indicators: beacon TX / telemetry RX / control TX
        act_x = hdr.right - 300
        for lbl, t_last, col in (("BCN", last_beacon_time, C_TEXT),
                                 ("TLM", last_telemetry_time, C_BLUE),
                                 ("CTL", last_tx_time, C_AMBER)):
            live = (now_t - t_last) < 0.15
            pygame.draw.circle(screen, col if live else C_TRACK, (act_x, hy + 11), 4)
            screen.blit(font_small.render(lbl, True, col if live else C_DIM), (act_x + 9, hy + 5))
            act_x += 52

        sig_col = draw_signal(screen, hdr.right - 130, hy, telemetry_data['RSSI'])
        try:
            sig_txt = (f"{int(telemetry_data['RSSI'])} dBm"
                       if int(telemetry_data['RSSI']) < 0 else "-- dBm")
        except (ValueError, TypeError):
            sig_txt = "-- dBm"
        screen.blit(font_body.render(sig_txt, True, sig_col), (hdr.right - 92, hy + 4))

        # ==================== SIDEBAR ====================
        sx = SCREEN_WIDTH - SIDEBAR_W - UI_MARGIN
        sy = HEADER_H + UI_MARGIN
        sw = SIDEBAR_W
        col_w = (sw - 48) // 2

        # --- Card: link quality ---
        card = pygame.Rect(sx, sy, sw, 124)
        draw_card(screen, card)
        screen.blit(font_lbl.render("LINK", True, C_DIM), (card.x + 16, card.y + 12))
        gx, gy = card.x + 16, card.y + 34
        draw_stat(screen, font_lbl, font_val, gx, gy, "FPS", f"{fps:02d}", fps_col, col_w)
        draw_stat(screen, font_lbl, font_val, gx + col_w + 16, gy, "LATENCY",
                  f"{total_latency:.0f}ms", lag_col, col_w)
        gy += 48
        kbps_txt = f"{telemetry_data.get('KBPS', 0):.0f}"
        draw_stat(screen, font_lbl, font_val, gx, gy, "BITRATE", kbps_txt, C_VIOLET, col_w)
        # Offset by the measured width so 4-digit bitrates don't collide.
        screen.blit(font_small.render("KB/s", True, C_DIM),
                    (gx + font_val.size(kbps_txt)[0] + 4, gy + 22))
        draw_stat(screen, font_lbl, font_val, gx + col_w + 16, gy, "DELIVERY",
                  f"{stream_delivery_pct:.0f}%", del_col, col_w)
        sy += card.h + UI_MARGIN

        # --- Card: latency breakdown ---
        card = pygame.Rect(sx, sy, sw, 128)
        draw_card(screen, card)
        screen.blit(font_lbl.render("PIPELINE", True, C_DIM), (card.x + 16, card.y + 12))
        by = card.y + 38
        for lbl, val, unit, frac, col in (
                ("Network transit", net_lag, "ms", net_lag / 200.0, C_BLUE),
                ("PC decode", proc_lag, "ms", proc_lag / 50.0, C_VIOLET),
                ("Reassembly buf", telemetry_data.get('BufSize', 0), "",
                 telemetry_data.get('BufSize', 0) / 8.0, C_AMBER)):
            screen.blit(font_small.render(lbl, True, C_DIM), (card.x + 16, by))
            vtxt = f"{val:.0f}{unit}" if unit else f"{int(val)}"
            vs = font_small.render(vtxt, True, C_TEXT)
            screen.blit(vs, (card.right - 16 - vs.get_width(), by))
            draw_meter(screen, card.x + 16, by + 19, sw - 32, 4, frac, col)
            by += 30
        sy += card.h + UI_MARGIN

        # --- Card: drive output ---
        card = pygame.Rect(sx, sy, sw, 186)
        draw_card(screen, card)
        screen.blit(font_lbl.render("DRIVE", True, C_DIM), (card.x + 16, card.y + 12))
        steer_n = (outmessage[0] - 1500) / 500.0
        throt_n = (outmessage[1] - 1500) / 500.0
        by = card.y + 36

        screen.blit(font_small.render("THROTTLE", True, C_DIM), (card.x + 16, by))
        ts = font_small.render(f"{throt_n:+.2f}", True, C_TEXT)
        screen.blit(ts, (card.right - 16 - ts.get_width(), by))
        draw_bipolar(screen, card.x + 16, by + 16, sw - 32, 9, throt_n, C_GREEN)
        by += 40

        screen.blit(font_small.render("STEERING", True, C_DIM), (card.x + 16, by))
        ss = font_small.render(f"{steer_n:+.2f}", True, C_TEXT)
        screen.blit(ss, (card.right - 16 - ss.get_width(), by))
        draw_bipolar(screen, card.x + 16, by + 16, sw - 32, 9, steer_n, C_BLUE)
        by += 40

        screen.blit(font_small.render("SPEED LIMIT", True, C_DIM), (card.x + 16, by))
        fs = font_small.render(f"{factor*100:.0f}%", True, C_TEXT)
        screen.blit(fs, (card.right - 16 - fs.get_width(), by))
        draw_meter(screen, card.x + 16, by + 16, sw - 32, 6, factor, C_AMBER)
        by += 34

        screen.blit(font_small.render("LIGHT", True, C_DIM), (card.x + 16, by))
        ls = font_small.render(f"{int(light/MAX_LIGHT*100)}%", True, C_TEXT)
        screen.blit(ls, (card.right - 16 - ls.get_width(), by))
        draw_meter(screen, card.x + 16, by + 16, sw - 32, 6, light / MAX_LIGHT, C_TEXT)
        sy += card.h + UI_MARGIN

        # --- Card: trim & mode ---
        card = pygame.Rect(sx, sy, sw, 92)
        draw_card(screen, card)
        screen.blit(font_lbl.render("TRIM & MODE", True, C_DIM), (card.x + 16, card.y + 12))
        rows = [
            ("Steer trim", f"{TURNTRIM:+.3f}"),
            ("Speed trim", f"{SPEEDTRIM:+.3f}"),
            ("Steer dir", "NORMAL" if TURN == 1 else "INVERTED"),
        ]
        by = card.y + 34
        for lbl, val in rows:
            screen.blit(font_small.render(lbl, True, C_DIM), (card.x + 16, by))
            vs = font_small.render(val, True, C_TEXT)
            screen.blit(vs, (card.right - 16 - vs.get_width(), by))
            by += 18

        # ==================== FOOTER ====================
        fy = SCREEN_HEIGHT - FOOTER_H + 4
        hints = ("DRIVE arrows/stick   SPEED -/=   LIGHT L/K   RES Q·W·E·T   "
                 "FLIP F/G   TRIM 6/7·8/9   INVERT 5   RESET 0   REBOOT R   HUD H")
        screen.blit(font_small.render(hints, True, C_DIM), (UI_MARGIN + 4, fy))

        pygame.display.flip()
        clock.tick(60)

    running = False
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()