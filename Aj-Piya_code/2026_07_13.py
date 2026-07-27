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
WINDOW_RES = (1400, 1050)    # Pygame window size
UI_MARGIN = 20               # Margin for layout parts
PANEL_HEIGHT = 250           # Height for bottom control panel
MIN_JPG_SIZE = 1000          # Ignore tiny/corrupt frames
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
fps = 0
lastimg = time.time()
last_heartbeat = time.time()
command_queue = queue.Queue(maxsize=COMMAND_QUEUE_SIZE)
telemetry_data = {"RSSI": "0", "RobotIP": "N/A", "ChipID": "Unknown", "LastSeq": 4294967295, "KBPS": 0, "ProcLag": 0, "NetLag": 0, "BufSize": 0}
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
    global myimage, lastimg, fps, running, flip, flip2
    
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
                
                if frame.shape[1] != VIDEO_RES[0] or frame.shape[0] != VIDEO_RES[1]:
                    frame = cv2.resize(frame, VIDEO_RES)
                
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = np.rot90(frame)
                
                myimage = pygame.surfarray.make_surface(frame)
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
# MAIN GUI
# ==========================================
def main():
    global running, flip, flip2, show_hud, VIDEO_RES, SCREEN_WIDTH, SCREEN_HEIGHT, last_light_sync, current_control_seq, last_drive_time, last_drive_msg, last_light_val, robot_connected, TARGET_ESP_IP, control_synced
    
    flip = False
    flip2 = False

    STREAM_AREA_HEIGHT = SCREEN_HEIGHT - PANEL_HEIGHT - (3 * UI_MARGIN)
    STREAM_AREA_WIDTH = SCREEN_WIDTH - (2 * UI_MARGIN)
    
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
    
    font = pygame.font.SysFont(None, 32, bold=False)
    font_small = pygame.font.SysFont("monospace", 16, bold=True)

    prev_msg = [ev(0), ev(0)]
    previous_press = 0
    
    factor = 0.35
    maxfactor = 1.0
    TURN = 1
    TURNFACTOR = 1.0
    TURNTRIM = 0
    SPEEDTRIM = 0
    light = 0
    
    while running:
        SCREEN_WIDTH, SCREEN_HEIGHT = screen.get_size()
        STREAM_AREA_HEIGHT = SCREEN_HEIGHT - PANEL_HEIGHT - (3 * UI_MARGIN)
        STREAM_AREA_WIDTH = SCREEN_WIDTH - (2 * UI_MARGIN)
        
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

            send_command(target_ip, CONTROL_PORT, 0, outmessage[0], outmessage[1])

        screen.fill((0, 0, 0)) 
        
        if myimage:
            x_pos = UI_MARGIN + (STREAM_AREA_WIDTH - myimage.get_width()) // 2
            y_pos = UI_MARGIN + (STREAM_AREA_HEIGHT - myimage.get_height()) // 2
            screen.blit(myimage, (x_pos, y_pos))
            pygame.draw.rect(screen, (40, 40, 50), (x_pos-2, y_pos-2, myimage.get_width()+4, myimage.get_height()+4), 1)
        
        if not show_hud:
            pygame.display.flip()
            clock.tick(60)
            continue

        panel_y = SCREEN_HEIGHT - PANEL_HEIGHT - UI_MARGIN
        hud_bg = pygame.Surface((SCREEN_WIDTH - 2*UI_MARGIN, PANEL_HEIGHT), pygame.SRCALPHA)
        hud_bg.fill((15, 15, 20, HUD_ALPHA)) 
        screen.blit(hud_bg, (UI_MARGIN, panel_y))
        
        hud_y = panel_y + 20 
        tcolor = (0, 255, 120) if robot_connected else (255, 50, 50)
        total_latency = (time.time() - lastimg) * 1000
        net_lag = telemetry_data.get('NetLag', 0)
        proc_lag = telemetry_data.get('ProcLag', 0)
        rssi_txt, rssi_col = get_rssi_quality(telemetry_data['RSSI'])
        fps_col = get_fps_quality(fps)
        
        x_off = UI_MARGIN + 20
        
        if time.time() - last_beacon_time < 0.02:
            screen.blit(font.render("[TX-B]", True, (255, 255, 255)), (x_off, hud_y))
        x_off += 80 
        
        status_lbl = "LINKED" if robot_connected else "SEARCHING"
        if not robot_connected:
            status_lbl += f" (@{target_ip})"
        screen.blit(font.render(f"STATUS : {status_lbl}", True, tcolor), (x_off, hud_y))
        
        x_off = UI_MARGIN + 530
        screen.blit(font.render(f"ROBOT : {telemetry_data['RobotIP']} [{telemetry_data['ChipID']}]", True, (200, 200, 200)), (x_off, hud_y))
        
        x_off = UI_MARGIN + 920
        signal_lbl = "SIG:" if robot_connected else "OFF:"
        screen.blit(font.render(f"{signal_lbl} {telemetry_data['RSSI']}dBm ({rssi_txt})", True, rssi_col), (x_off, hud_y))

        hud_y += 35
        x_off = UI_MARGIN + 20
        
        if time.time() - last_telemetry_time < 0.02:
            screen.blit(font.render("[RX]", True, (0, 255, 200)), (x_off, hud_y))
        x_off += 80 
        
        metrics_top = [
            (f"FPS: {fps:02}", fps_col),
            (f"  VIDEO: {'OK' if myimage else 'WAITING'}", (255, 255, 255) if myimage else (255, 50, 50)),
            (f"  BITRATE: {telemetry_data.get('KBPS', 0):.1f} KB/s", (180, 255, 180)),
            (f"  TOTAL LAG: {total_latency:03.0f}ms", (255, 255, 255) if total_latency < 200 else (255, 100, 100)),
            (f"  DELIVERY: {stream_delivery_pct:.0f}%", (100, 200, 255))
        ]
        for txt, color in metrics_top:
            surf = font.render(txt, True, color)
            screen.blit(surf, (x_off, hud_y)); x_off += surf.get_width() + 10

        hud_y += 30
        x_off = UI_MARGIN + 20
        
        if time.time() - last_tx_time < 0.02:
            screen.blit(font.render("[TX-C]", True, (255, 220, 0)), (x_off, hud_y))
        x_off += 80 
        
        metrics_bot = [
            (f"NETWORK TRANSIT: {net_lag:.0f}ms", (180, 180, 180)),
            (f"  PC DECODE TIME: {proc_lag:.1f}ms", (180, 180, 180)),
            (f"  REASSEMBLY BUF: {int(telemetry_data.get('BufSize', 0))} frames", (200, 150, 100))
        ]
        for txt, color in metrics_bot:
            surf = font.render(txt, True, color)
            screen.blit(surf, (x_off, hud_y)); x_off += surf.get_width() + 10

        hud_y += 30
        steer_status = "NORMAL" if TURN == 1 else "INVERT"
        drive_msg = f"DRIVE  : Factor {factor*100:>3.0f}% | Steer {steer_status} | Trim T:{TURNTRIM:+.3f} S:{SPEEDTRIM:+.3f}"
        screen.blit(font.render(drive_msg, True, (100, 200, 255)), (UI_MARGIN + 20, hud_y))

        hud_y += 30
        mtr_msg = f"OUTPUT : SERVO {outmessage[0]:04} | DC {outmessage[1]:04} | LIGHT {int(light):05}"
        screen.blit(font.render(mtr_msg, True, (255, 200, 100)), (UI_MARGIN + 20, hud_y))

        hud_y += 30
        h1 = "DRIVE: Arrows / Stick | SPEED: - / = | LIGHT: L / K | RESET: 0"
        h2 = "VIDEO: Q (Low) | W (High) | F/G (Flip) | R (Reboot) | H (HUD)"
        h3 = "TRIM: 6/7 (Steer) 8/9 (Speed) | 5 (Invert Steer)"
        screen.blit(font_small.render(h1, True, (120, 120, 140)), (UI_MARGIN + 20, hud_y))
        screen.blit(font_small.render(h2, True, (120, 120, 140)), (UI_MARGIN + 20, hud_y + 18))
        screen.blit(font_small.render(h3, True, (120, 120, 140)), (UI_MARGIN + 20, hud_y + 36))

        pygame.display.flip()
        clock.tick(60)

    running = False
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()