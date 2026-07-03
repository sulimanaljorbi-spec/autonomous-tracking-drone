"""
Jetson Vision Tracker — Experiment 2 COMPLETE
- Full protocol communication with STM32
- TensorRT YOLOv8 inference
- Centroid Tracker (locks onto one person across frames)
- Heartbeat, TRACK_CMD, telemetry receive
"""

import cv2
import serial
import struct
import time
import math
import threading
from ultralytics import YOLO

# ─────────────────────────────────────────────────────────────────
# Protocol Constants
# ─────────────────────────────────────────────────────────────────
HEADER_1      = 0xAA
HEADER_2      = 0x55
MSG_HEARTBEAT = 0x01
MSG_TRACK_CMD = 0x02
MSG_ATTITUDE  = 0x10
MSG_GPS       = 0x11
MSG_BATTERY   = 0x12


# ─────────────────────────────────────────────────────────────────
# Centroid Tracker
# ─────────────────────────────────────────────────────────────────
class CentroidTracker:
    """
    Tracks objects across frames by matching nearest centroids.
    Locks onto one target and maintains its ID even if briefly lost.
    """
    def __init__(self, max_lost=15):
        self.next_id  = 0
        self.tracks   = {}   # id -> (cx, cy, lost_count)
        self.max_lost = max_lost

    def update(self, detections):
        """
        detections: list of dicts with 'center' and 'bbox' and 'confidence'
        returns: dict of {track_id: detection}
        """
        if not detections:
            lost_ids = []
            for tid in self.tracks:
                cx, cy, lost = self.tracks[tid]
                self.tracks[tid] = (cx, cy, lost + 1)
                if lost + 1 > self.max_lost:
                    lost_ids.append(tid)
            for tid in lost_ids:
                del self.tracks[tid]
            return {}

        det_centers = [d['center'] for d in detections]
        matched     = {}
        used_dets   = set()

        for tid, (tx, ty, _) in self.tracks.items():
            best_dist = float('inf')
            best_idx  = -1
            for i, (dx, dy) in enumerate(det_centers):
                if i in used_dets:
                    continue
                dist = math.sqrt((tx - dx)**2 + (ty - dy)**2)
                if dist < best_dist:
                    best_dist = dist
                    best_idx  = i
            if best_idx >= 0 and best_dist < 150:
                matched[tid] = detections[best_idx]
                cx, cy = det_centers[best_idx]
                self.tracks[tid] = (cx, cy, 0)
                used_dets.add(best_idx)

        # New tracks for unmatched detections
        for i, det in enumerate(detections):
            if i not in used_dets:
                cx, cy = det['center']
                self.tracks[self.next_id] = (cx, cy, 0)
                matched[self.next_id] = det
                self.next_id += 1

        # Increment lost count for unmatched tracks
        for tid in list(self.tracks):
            if tid not in matched:
                cx, cy, lost = self.tracks[tid]
                self.tracks[tid] = (cx, cy, lost + 1)
                if lost + 1 > self.max_lost:
                    del self.tracks[tid]

        return matched


# ─────────────────────────────────────────────────────────────────
# STM32 Communication Link
# ─────────────────────────────────────────────────────────────────
class STM32Link:
    def __init__(self, port='/dev/ttyTHS1', baud=115200):
        self.ser      = serial.Serial(port, baud, timeout=0.1)
        self.attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'alt': 0.0}
        self.gps      = {'lat': 0.0, 'lon': 0.0, 'speed': 0.0}
        self.battery  = {'voltage': 0.0, 'current': 0.0}
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def _send(self, msg_id, payload=b''):
        length   = len(payload)
        checksum = msg_id ^ length
        for b in payload:
            checksum ^= b
        packet = (
            bytes([HEADER_1, HEADER_2, msg_id, length])
            + payload
            + bytes([checksum & 0xFF])
        )
        self.ser.write(packet)

    def send_heartbeat(self, status=1):
        self._send(MSG_HEARTBEAT, bytes([status & 0xFF]))

    def send_track_cmd(self, offset_x, offset_y, confidence):
        conf_int = int(max(0.0, min(1.0, confidence)) * 100)
        payload  = struct.pack('>hhH', int(offset_x), int(offset_y), conf_int)
        self._send(MSG_TRACK_CMD, payload)

    def send_hover(self):
        self.send_track_cmd(0, 0, 0.0)

    def _rx_loop(self):
        S_H1=0; S_H2=1; S_ID=2; S_LEN=3; S_PL=4; S_CHK=5
        state=S_H1; msg_id=0; length=0; payload=bytearray(); chk=0
        while True:
            try:
                raw = self.ser.read(1)
                if not raw:
                    continue
                b = raw[0]
                if state == S_H1:
                    if b == HEADER_1: state = S_H2
                elif state == S_H2:
                    state = S_ID if b == HEADER_2 else S_H1
                elif state == S_ID:
                    msg_id = b; chk = b; state = S_LEN
                elif state == S_LEN:
                    length = b; chk ^= b; payload = bytearray()
                    state = S_PL if length > 0 else S_CHK
                elif state == S_PL:
                    payload.append(b); chk ^= b
                    if len(payload) >= length: state = S_CHK
                elif state == S_CHK:
                    if b == (chk & 0xFF):
                        self._handle_msg(msg_id, bytes(payload))
                    state = S_H1
            except Exception:
                state = S_H1

    def _handle_msg(self, msg_id, payload):
        try:
            if msg_id == MSG_ATTITUDE and len(payload) >= 8:
                r, p, y, a = struct.unpack('>hhhh', payload[:8])
                self.attitude = {
                    'roll':  r / 100.0,
                    'pitch': p / 100.0,
                    'yaw':   y / 100.0,
                    'alt':   a / 100.0,
                }
            elif msg_id == MSG_GPS and len(payload) >= 10:
                lat, lon, spd = struct.unpack('>iih', payload[:10])
                self.gps = {
                    'lat':   lat / 1e6,
                    'lon':   lon / 1e6,
                    'speed': spd / 100.0,
                }
            elif msg_id == MSG_BATTERY and len(payload) >= 4:
                v, i = struct.unpack('>hh', payload[:4])
                self.battery = {
                    'voltage': v / 100.0,
                    'current': i / 100.0,
                }
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────
# GStreamer Pipeline
# ─────────────────────────────────────────────────────────────────
def gstreamer_pipeline(capture_w=1280, capture_h=720,
                        out_w=640, out_h=360, framerate=60):
    return (
        f"nvarguscamerasrc sensor-id=0 ! "
        f"video/x-raw(memory:NVMM), width={capture_w}, height={capture_h}, "
        f"format=NV12, framerate={framerate}/1 ! "
        f"nvvidconv ! "
        f"video/x-raw, width={out_w}, height={out_h}, format=BGRx ! "
        f"videoconvert ! video/x-raw, format=BGR ! "
        f"appsink drop=true sync=false"
    )


# ─────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────
def main():

    # ── UART ──
    print("Connecting UART...")
    try:
        link = STM32Link('/dev/ttyTHS1', 115200)
        print("UART Connected!")
    except Exception as e:
        print(f"UART Error: {e}")
        return

    # ── YOLO — use .engine if available, fall back to .pt ──
    print("Loading YOLO...")
    try:
        model = YOLO('yolov8n.engine')
        print("YOLO TensorRT engine loaded!")
    except Exception:
        print("TensorRT engine not found, loading .pt model...")
        model = YOLO('yolov8n.pt')
        print("YOLO .pt model loaded (slower — export to .engine for better FPS)")

    # ── Tracker ──
    tracker         = CentroidTracker(max_lost=15)
    target_track_id = None   # ID of the person we are following

    # ── Camera ──
    print("Opening Camera...")
    cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Camera failed to open!")
        return
    print("Camera Ready!  Press Q to quit.\n")

    FRAME_W  = 640
    FRAME_H  = 360
    frame_cx = FRAME_W // 2   # 320
    frame_cy = FRAME_H // 2   # 180

    fps_count = 0
    fps_val   = 0.0
    t_fps     = time.time()
    t_hb      = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Frame grab failed, retrying...")
                continue

            # ── Heartbeat every 500 ms ──
            if time.time() - t_hb >= 0.5:
                link.send_heartbeat(1)
                t_hb = time.time()

            # ── YOLO Detection (person class = 0) ──
            results    = model(frame, verbose=False, classes=[0])
            detections = []

            if len(results[0].boxes) > 0:
                boxes_xyxy = results[0].boxes.xyxy.cpu().numpy()
                boxes_conf = results[0].boxes.conf.cpu().numpy()

                for i in range(len(boxes_xyxy)):
                    x1, y1, x2, y2 = map(int, boxes_xyxy[i])
                    conf = float(boxes_conf[i])
                    cx   = (x1 + x2) // 2
                    cy   = (y1 + y2) // 2
                    detections.append({
                        'bbox':       (x1, y1, x2, y2),
                        'center':     (cx, cy),
                        'confidence': conf,
                    })

            # ── Centroid Tracker Update ──
            tracked = tracker.update(detections)

            # Lock onto first detected person if no target yet
            if target_track_id is None and tracked:
                target_track_id = list(tracked.keys())[0]
                print(f"Locked onto Track ID: {target_track_id}")

            # ── Compute offset and send to STM32 ──
            offset_x   = 0
            offset_y   = 0
            confidence  = 0.0
            detected   = False

            if target_track_id is not None and target_track_id in tracked:
                det  = tracked[target_track_id]
                x1, y1, x2, y2 = det['bbox']
                cx, cy          = det['center']
                confidence      = det['confidence']

                # Pixel offset from frame center
                offset_x = cx - frame_cx
                offset_y = cy - frame_cy
                detected = True

                # Draw green box and center dot
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                cv2.putText(frame,
                    f"ID:{target_track_id}  Off({offset_x:+d},{offset_y:+d})  {confidence:.2f}",
                    (x1, max(y1 - 8, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

                print(f"TRACKING ID:{target_track_id}  "
                      f"offset=({offset_x:+d},{offset_y:+d})  "
                      f"conf={confidence:.2f}")

            else:
                # Target lost — reset so we can re-lock
                if target_track_id is not None:
                    print(f"Target ID:{target_track_id} lost — searching...")
                    target_track_id = None
                    # Re-lock on next available track
                    if tracked:
                        target_track_id = list(tracked.keys())[0]
                else:
                    print("SEARCHING...")

            # Draw all other detected persons in blue
            for tid, det in tracked.items():
                if tid != target_track_id:
                    x1, y1, x2, y2 = det['bbox']
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 100, 0), 1)
                    cv2.putText(frame, f"ID:{tid}",
                        (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 100, 0), 1)

            # ── Send TRACK_CMD or hover ──
            if detected:
                link.send_track_cmd(offset_x, offset_y, confidence)
            else:
                link.send_hover()

            # ── Crosshair at frame center ──
            cv2.line(frame, (frame_cx - 20, frame_cy),
                             (frame_cx + 20, frame_cy), (255, 255, 0), 1)
            cv2.line(frame, (frame_cx, frame_cy - 20),
                             (frame_cx, frame_cy + 20), (255, 255, 0), 1)

            # ── FPS ──
            fps_count += 1
            elapsed = time.time() - t_fps
            if elapsed >= 1.0:
                fps_val   = fps_count / elapsed
                fps_count = 0
                t_fps     = time.time()
            cv2.putText(frame, f"FPS:{fps_val:.1f}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

            # ── Status ──
            status = f"TRACKING ID:{target_track_id}" if detected else "SEARCHING..."
            color  = (0, 255, 0) if detected else (0, 140, 255)
            cv2.putText(frame, status, (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

            # ── STM32 telemetry overlay ──
            att = link.attitude
            cv2.putText(frame,
                f"STM32  R:{att['roll']:.1f}  "
                f"P:{att['pitch']:.1f}  "
                f"Y:{att['yaw']:.1f}",
                (10, FRAME_H - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

            cv2.imshow("Drone Vision Tracker", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Quitting...")
                break

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        link.send_hover()
        cap.release()
        cv2.destroyAllWindows()
        print("Cleanup done.")


if __name__ == '__main__':
    main()
