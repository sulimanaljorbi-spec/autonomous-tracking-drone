"""
Jetson Orin Nano — Experiment 3 Complete
Runs TWO tasks depending on argument:

  python3 jetson_exp3.py upload   ← run on ground before flight
  python3 jetson_exp3.py fly      ← run during flight (vision tracker)

Usage:
  1. On ground (SSH into Jetson):
       edit mission.json with real GPS coordinates
       python3 jetson_exp3.py upload
       (uploads waypoints → Jetson disconnected → drone takes off)

  2. During flight:
       python3 jetson_exp3.py fly
       (vision tracking + live telemetry overlay)
"""

import sys
import cv2
import serial
import struct
import time
import math
import json
import os
import threading
from ultralytics import YOLO

# ─────────────────────────────────────────────────────────────────
# Protocol Constants
# ─────────────────────────────────────────────────────────────────
HEADER_1        = 0xAA
HEADER_2        = 0x55
MSG_HEARTBEAT   = 0x01
MSG_TRACK_CMD   = 0x02
MSG_ATTITUDE    = 0x10
MSG_GPS         = 0x11
MSG_BATTERY     = 0x12
MSG_WP_UPLOAD   = 0x20
MSG_MISSION_CMD = 0x21
MSG_NAV_STATUS  = 0x30
MSG_GEO_STATUS  = 0x31

NAV_STATES = {
    0: 'IDLE',    1: 'TAKEOFF', 2: 'NAVIGATE',
    3: 'HOVER',   4: 'ADVANCE', 5: 'RTH',
    6: 'LAND',
}
GEO_STATES = {0: 'OK', 1: 'WARNING', 2: 'BREACH'}

ACTION_MAP = {
    'flythrough': 0,
    'hover':      1,
    'land':       2,
    'rth':        3,
}


# ─────────────────────────────────────────────────────────────────
# STM32 Link — handles all send/receive
# ─────────────────────────────────────────────────────────────────
class STM32Link:
    def __init__(self, port='/dev/ttyTHS1', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.1)

        # Telemetry (updated by receive thread)
        self.attitude   = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'alt': 0.0}
        self.gps        = {'lat': 0.0, 'lon': 0.0, 'speed': 0.0}
        self.battery    = {'voltage': 0.0, 'current': 0.0}
        self.nav_state  = 0
        self.wp_idx     = 0
        self.geo_status = 0

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    # ── Internal send ─────────────────────────────────────────────
    def _send(self, msg_id, payload=b''):
        length   = len(payload)
        checksum = msg_id ^ length
        for b in payload:
            checksum ^= b
        self.ser.write(
            bytes([HEADER_1, HEADER_2, msg_id, length])
            + payload
            + bytes([checksum & 0xFF])
        )

    # ── Exp2 messages ─────────────────────────────────────────────
    def send_heartbeat(self):
        self._send(MSG_HEARTBEAT, bytes([1]))

    def send_track_cmd(self, offset_x, offset_y, confidence):
        conf_int = int(max(0.0, min(1.0, confidence)) * 100)
        self._send(MSG_TRACK_CMD,
                   struct.pack('>hhH', int(offset_x), int(offset_y), conf_int))

    def send_hover(self):
        self.send_track_cmd(0, 0, 0.0)

    # ── Exp3 messages ─────────────────────────────────────────────
    def send_waypoint(self, lat, lon, alt, speed, action, hover_time):
        """Upload one waypoint — matches MSG_WP_UPLOAD handler on STM32."""
        payload = struct.pack('>ffBfBH',
                              float(lat), float(lon),
                              int(alt),   float(speed),
                              int(action), int(hover_time))
        self._send(MSG_WP_UPLOAD, payload)

    def send_mission_start(self):
        self._send(MSG_MISSION_CMD, bytes([0x01]))

    def send_mission_abort(self):
        self._send(MSG_MISSION_CMD, bytes([0x00]))

    # ── Receive loop ──────────────────────────────────────────────
    def _rx_loop(self):
        S_H1=0; S_H2=1; S_ID=2; S_LEN=3; S_PL=4; S_CHK=5
        state=S_H1; msg_id=0; length=0; payload=bytearray(); chk=0
        while True:
            try:
                raw = self.ser.read(1)
                if not raw: continue
                b = raw[0]
                if   state == S_H1: state = S_H2 if b == HEADER_1 else S_H1
                elif state == S_H2: state = S_ID if b == HEADER_2 else S_H1
                elif state == S_ID:
                    msg_id = b; chk = b; state = S_LEN
                elif state == S_LEN:
                    length = b; chk ^= b; payload = bytearray()
                    state  = S_PL if length > 0 else S_CHK
                elif state == S_PL:
                    payload.append(b); chk ^= b
                    if len(payload) >= length: state = S_CHK
                elif state == S_CHK:
                    if b == (chk & 0xFF): self._handle(msg_id, bytes(payload))
                    state = S_H1
            except Exception:
                state = S_H1

    def _handle(self, msg_id, payload):
        try:
            if msg_id == MSG_ATTITUDE and len(payload) >= 8:
                r, p, y, a = struct.unpack('>hhhh', payload[:8])
                self.attitude = {
                    'roll': r/100.0, 'pitch': p/100.0,
                    'yaw':  y/100.0, 'alt':   a/100.0,
                }
            elif msg_id == MSG_GPS and len(payload) >= 10:
                lat, lon, spd = struct.unpack('>iih', payload[:10])
                self.gps = {'lat': lat/1e6, 'lon': lon/1e6, 'speed': spd/100.0}
            elif msg_id == MSG_BATTERY and len(payload) >= 4:
                v, i = struct.unpack('>hh', payload[:4])
                self.battery = {'voltage': v/100.0, 'current': i/100.0}
            elif msg_id == MSG_NAV_STATUS and len(payload) >= 2:
                self.nav_state = payload[0]
                self.wp_idx    = payload[1]
            elif msg_id == MSG_GEO_STATUS and len(payload) >= 1:
                self.geo_status = payload[0]
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────
# Centroid Tracker
# ─────────────────────────────────────────────────────────────────
class CentroidTracker:
    def __init__(self, max_lost=15):
        self.next_id  = 0
        self.tracks   = {}
        self.max_lost = max_lost

    def update(self, detections):
        if not detections:
            lost = [tid for tid, (_, _, l) in self.tracks.items()
                    if l + 1 > self.max_lost]
            for tid in lost: del self.tracks[tid]
            self.tracks = {tid: (cx, cy, l+1)
                           for tid, (cx, cy, l) in self.tracks.items()
                           if tid not in lost}
            return {}

        matched   = {}
        used_dets = set()

        for tid, (tx, ty, _) in self.tracks.items():
            best_dist, best_idx = float('inf'), -1
            for i, d in enumerate(detections):
                if i in used_dets: continue
                dx, dy = d['center']
                dist = math.sqrt((tx-dx)**2 + (ty-dy)**2)
                if dist < best_dist:
                    best_dist, best_idx = dist, i
            if best_idx >= 0 and best_dist < 150:
                matched[tid] = detections[best_idx]
                cx, cy = detections[best_idx]['center']
                self.tracks[tid] = (cx, cy, 0)
                used_dets.add(best_idx)

        for i, det in enumerate(detections):
            if i not in used_dets:
                cx, cy = det['center']
                self.tracks[self.next_id] = (cx, cy, 0)
                matched[self.next_id] = det
                self.next_id += 1

        for tid in list(self.tracks):
            if tid not in matched:
                cx, cy, l = self.tracks[tid]
                if l + 1 > self.max_lost: del self.tracks[tid]
                else: self.tracks[tid] = (cx, cy, l+1)

        return matched


# ─────────────────────────────────────────────────────────────────
# GStreamer Pipeline
# ─────────────────────────────────────────────────────────────────
def gstreamer_pipeline():
    return (
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=1280, height=720, "
        "format=NV12, framerate=60/1 ! "
        "nvvidconv ! "
        "video/x-raw, width=640, height=360, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink drop=true sync=false"
    )


# ─────────────────────────────────────────────────────────────────
# MODE 1 — Mission Upload (run on ground before flight)
# ─────────────────────────────────────────────────────────────────
def run_upload():
    print("=" * 50)
    print("  MISSION UPLOAD MODE")
    print("  Run this on the ground before takeoff")
    print("=" * 50)

    if not os.path.exists('mission.json'):
        print("\nERROR: mission.json not found in current directory")
        print("Create it first with your GPS waypoints.")
        return

    with open('mission.json') as f:
        data = json.load(f)

    waypoints = data.get('waypoints', [])
    if not waypoints:
        print("ERROR: No waypoints found in mission.json")
        return

    print(f"\nFound {len(waypoints)} waypoints in mission.json")
    for i, wp in enumerate(waypoints):
        print(f"  WP{i+1}: ({wp['lat']:.6f}, {wp['lon']:.6f}) "
              f"alt={wp['alt']}m  action={wp.get('action','flythrough')}")

    print("\nConnecting to STM32...")
    try:
        link = STM32Link('/dev/ttyTHS1', 115200)
        print("Connected!")
    except Exception as e:
        print(f"UART Error: {e}")
        return

    print(f"\nUploading {len(waypoints)} waypoints...")
    for i, wp in enumerate(waypoints):
        action     = ACTION_MAP.get(wp.get('action', 'flythrough'), 0)
        hover_time = wp.get('hover_time', 0)
        speed      = wp.get('speed', 3.0)
        link.send_waypoint(wp['lat'], wp['lon'], wp['alt'],
                           speed, action, hover_time)
        print(f"  WP{i+1} sent")
        time.sleep(0.1)

    print("\nAll waypoints uploaded successfully!")
    print("\nOptions:")
    print("  [S] Send START command now (drone will take off when armed)")
    print("  [N] Upload only — start later with AUX1 switch")
    choice = input("\nChoice (S/N): ").strip().lower()

    if choice == 's':
        link.send_mission_start()
        print("Mission START sent!")
        print("Arm the drone and it will take off automatically.")
    else:
        print("Mission uploaded. Flip AUX1 switch to start after takeoff.")

    print("\nUpload complete. You can now disconnect SSH.")
    print("Run 'python3 jetson_exp3.py fly' to start vision tracking.")


# ─────────────────────────────────────────────────────────────────
# MODE 2 — Vision Tracking (run during flight)
# ─────────────────────────────────────────────────────────────────
def run_fly():
    print("=" * 50)
    print("  FLIGHT MODE — Vision Tracker + Telemetry")
    print("=" * 50)

    # UART
    print("\nConnecting UART...")
    try:
        link = STM32Link('/dev/ttyTHS1', 115200)
        print("UART Connected!")
    except Exception as e:
        print(f"UART Error: {e}")
        return

    # YOLO
    print("Loading YOLO...")
    try:
        model = YOLO('yolov8n.engine')
        print("TensorRT engine loaded!")
    except Exception:
        print("No .engine, loading .pt...")
        model = YOLO('yolov8n.pt')
        print("YOLO .pt loaded.")

    # Tracker
    tracker         = CentroidTracker(max_lost=15)
    target_track_id = None

    # Camera
    print("Opening Camera...")
    cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Camera failed!")
        return
    print("Camera Ready!  Press Q to quit.\n")

    FRAME_W  = 640;  FRAME_H = 360
    frame_cx = FRAME_W // 2;  frame_cy = FRAME_H // 2

    fps_count = 0;  fps_val = 0.0
    t_fps = time.time();  t_hb = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            # Heartbeat
            if time.time() - t_hb >= 0.5:
                link.send_heartbeat()
                t_hb = time.time()

            # YOLO detection
            results    = model(frame, verbose=False, classes=[0])
            detections = []
            if len(results[0].boxes) > 0:
                for i in range(len(results[0].boxes)):
                    x1,y1,x2,y2 = map(int, results[0].boxes.xyxy[i].cpu().numpy())
                    conf = float(results[0].boxes.conf[i].cpu().numpy())
                    cx   = (x1+x2)//2;  cy = (y1+y2)//2
                    detections.append({
                        'bbox': (x1,y1,x2,y2), 'center': (cx,cy),
                        'confidence': conf
                    })

            # Centroid tracker
            tracked = tracker.update(detections)
            if target_track_id is None and tracked:
                target_track_id = list(tracked.keys())[0]
                print(f"Locked onto ID:{target_track_id}")

            offset_x = 0;  offset_y = 0;  confidence = 0.0;  detected = False

            if target_track_id is not None and target_track_id in tracked:
                det  = tracked[target_track_id]
                x1,y1,x2,y2 = det['bbox']
                cx,cy = det['center']
                confidence = det['confidence']
                offset_x = cx - frame_cx
                offset_y = cy - frame_cy
                detected = True
                cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.circle(frame, (cx,cy), 6, (0,0,255), -1)
                cv2.line(frame, (frame_cx,frame_cy),
                                (frame_cx+offset_x,frame_cy), (0,255,255), 2)
                cv2.putText(frame,
                    f"ID:{target_track_id} Off({offset_x:+d},{offset_y:+d})"
                    f" {confidence:.2f}",
                    (x1, max(y1-8,12)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (0,255,0), 1)
            else:
                if target_track_id is not None:
                    target_track_id = None
                if tracked:
                    target_track_id = list(tracked.keys())[0]

            # Draw other tracks in blue
            for tid, det in tracked.items():
                if tid != target_track_id:
                    x1,y1,x2,y2 = det['bbox']
                    cv2.rectangle(frame,(x1,y1),(x2,y2),(255,100,0),1)

            # Send to STM32
            if detected:
                link.send_track_cmd(offset_x, offset_y, confidence)
            else:
                link.send_hover()

            # Crosshair
            cv2.line(frame,(frame_cx-20,frame_cy),(frame_cx+20,frame_cy),(255,255,0),1)
            cv2.line(frame,(frame_cx,frame_cy-20),(frame_cx,frame_cy+20),(255,255,0),1)

            # FPS
            fps_count += 1
            if time.time()-t_fps >= 1.0:
                fps_val = fps_count/(time.time()-t_fps)
                fps_count = 0; t_fps = time.time()
            cv2.putText(frame, f"FPS:{fps_val:.1f}", (10,25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,255), 2)

            # Tracking status
            status = f"TRACKING ID:{target_track_id}" if detected else "SEARCHING..."
            cv2.putText(frame, status, (10,55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0,255,0) if detected else (0,140,255), 2)

            # NAV state overlay
            nav  = NAV_STATES.get(link.nav_state, '?')
            geo  = GEO_STATES.get(link.geo_status, '?')
            geo_color = (0,255,0) if link.geo_status==0 else \
                        (0,165,255) if link.geo_status==1 else (0,0,255)
            cv2.putText(frame,
                f"NAV:{nav} WP:{link.wp_idx}  GEO:{geo}",
                (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.55, geo_color, 2)

            # Battery overlay
            bat   = link.battery
            bcolor = (0,255,0)
            if bat['voltage'] > 0:
                if bat['voltage'] < 10.2:   bcolor = (0,0,255)
                elif bat['voltage'] < 10.8:  bcolor = (0,165,255)
            cv2.putText(frame,
                f"BAT:{bat['voltage']:.1f}V",
                (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.55, bcolor, 2)

            # Attitude overlay
            att = link.attitude
            cv2.putText(frame,
                f"R:{att['roll']:.1f} P:{att['pitch']:.1f} "
                f"Y:{att['yaw']:.1f} Alt:{att['alt']:.1f}m",
                (10, FRAME_H-10), cv2.FONT_HERSHEY_SIMPLEX,
                0.4, (200,200,200), 1)

            # GPS overlay
            gps = link.gps
            cv2.putText(frame,
                f"GPS:{gps['lat']:.5f},{gps['lon']:.5f}",
                (10, FRAME_H-28), cv2.FONT_HERSHEY_SIMPLEX,
                0.35, (180,180,180), 1)

            cv2.imshow("Drone Vision — Exp3", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Quitting...")
                break
            elif key == ord(' '):
                target_track_id = None
                print("Target lock reset.")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        link.send_hover()
        cap.release()
        cv2.destroyAllWindows()
        print("Done.")


# ─────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 jetson_exp3.py upload   (before flight — upload mission)")
        print("  python3 jetson_exp3.py fly       (during flight — vision tracker)")
        sys.exit(1)

    mode = sys.argv[1].lower()
    if mode == 'upload':
        run_upload()
    elif mode == 'fly':
        run_fly()
    else:
        print(f"Unknown mode: {mode}")
        print("Use 'upload' or 'fly'")
