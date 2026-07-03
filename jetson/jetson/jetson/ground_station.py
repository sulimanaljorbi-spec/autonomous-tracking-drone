"""
Jetson Ground Station — Experiment 3
- Upload waypoint missions from JSON to STM32
- Live telemetry display (attitude, GPS, battery, nav state)
- Geofence status monitor
- Mission abort / RTH command
"""

import serial
import struct
import time
import json
import threading
import os

# ─────────────────────────────────────────────────────────────────
# Protocol Constants (must match STM32 exactly)
# ─────────────────────────────────────────────────────────────────
HEADER_1       = 0xAA
HEADER_2       = 0x55
MSG_HEARTBEAT  = 0x01
MSG_TRACK_CMD  = 0x02
MSG_ATTITUDE   = 0x10
MSG_GPS        = 0x11
MSG_BATTERY    = 0x12
MSG_WP_UPLOAD  = 0x20
MSG_MISSION_CMD = 0x21
MSG_NAV_STATUS = 0x30
MSG_GEO_STATUS = 0x31

NAV_STATES = {
    0: 'IDLE',
    1: 'TAKEOFF',
    2: 'NAVIGATE',
    3: 'HOVER',
    4: 'ADVANCE',
    5: 'RTH',
    6: 'LAND',
}

GEO_STATES = {
    0: 'OK',
    1: 'WARNING',
    2: 'BREACH',
}


# ─────────────────────────────────────────────────────────────────
# STM32 Link (same protocol as Exp2, extended for Exp3)
# ─────────────────────────────────────────────────────────────────
class STM32Link:
    def __init__(self, port='/dev/ttyTHS1', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.1)

        # Live telemetry
        self.attitude   = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'alt': 0.0}
        self.gps        = {'lat': 0.0, 'lon': 0.0, 'speed': 0.0}
        self.battery    = {'voltage': 0.0, 'current': 0.0}
        self.nav_state  = 0
        self.wp_idx     = 0
        self.geo_status = 0

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    # ── Send ─────────────────────────────────────────────────────
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

    def send_heartbeat(self):
        self._send(MSG_HEARTBEAT, bytes([1]))

    def send_waypoint(self, lat, lon, alt, speed, action, hover_time):
        """
        Upload one waypoint to STM32.
        Payload: lat(4f) lon(4f) alt(1B) speed(4f) action(1B) hover_time(2H) = 16 bytes
        """
        payload = struct.pack('>ffBfBH',
                              float(lat),
                              float(lon),
                              int(alt),
                              float(speed),
                              int(action),
                              int(hover_time))
        self._send(MSG_WP_UPLOAD, payload)

    def send_mission_start(self):
        """Tell STM32 to start the uploaded mission."""
        self._send(MSG_MISSION_CMD, bytes([0x01]))

    def send_mission_abort(self):
        """Abort mission — STM32 will trigger RTH."""
        self._send(MSG_MISSION_CMD, bytes([0x00]))

    def send_rth(self):
        """Direct RTH command."""
        self.send_mission_abort()

    # ── Receive loop ─────────────────────────────────────────────
    def _rx_loop(self):
        S_H1=0; S_H2=1; S_ID=2; S_LEN=3; S_PL=4; S_CHK=5
        state=S_H1; msg_id=0; length=0; payload=bytearray(); chk=0
        while True:
            try:
                raw = self.ser.read(1)
                if not raw: continue
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
                        self._handle(msg_id, bytes(payload))
                    state = S_H1
            except Exception:
                state = S_H1

    def _handle(self, msg_id, payload):
        try:
            if msg_id == MSG_ATTITUDE and len(payload) >= 8:
                r, p, y, a = struct.unpack('>hhhh', payload[:8])
                self.attitude = {
                    'roll':  r / 100.0, 'pitch': p / 100.0,
                    'yaw':   y / 100.0, 'alt':   a / 100.0,
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
                self.battery = {'voltage': v / 100.0, 'current': i / 100.0}
            elif msg_id == MSG_NAV_STATUS and len(payload) >= 2:
                self.nav_state = payload[0]
                self.wp_idx    = payload[1]
            elif msg_id == MSG_GEO_STATUS and len(payload) >= 1:
                self.geo_status = payload[0]
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────
# Mission Planner
# ─────────────────────────────────────────────────────────────────
class MissionPlanner:
    """Load waypoints from JSON and upload to STM32."""

    ACTION_MAP = {
        'flythrough': 0,
        'hover':      1,
        'land':       2,
        'rth':        3,
    }

    def __init__(self, link):
        self.link = link

    def upload_from_file(self, filename):
        if not os.path.exists(filename):
            print(f"ERROR: {filename} not found")
            return False

        with open(filename) as f:
            data = json.load(f)

        waypoints = data.get('waypoints', [])
        if not waypoints:
            print("ERROR: No waypoints in mission file")
            return False

        print(f"\nUploading {len(waypoints)} waypoints...")
        for i, wp in enumerate(waypoints):
            action     = self.ACTION_MAP.get(wp.get('action', 'flythrough'), 0)
            hover_time = wp.get('hover_time', 0)
            speed      = wp.get('speed', 3.0)

            self.link.send_waypoint(
                lat        = wp['lat'],
                lon        = wp['lon'],
                alt        = wp['alt'],
                speed      = speed,
                action     = action,
                hover_time = hover_time,
            )
            print(f"  WP{i+1}: ({wp['lat']:.6f}, {wp['lon']:.6f}) "
                  f"alt={wp['alt']}m  action={wp.get('action','flythrough')}  "
                  f"hover={hover_time}s")
            time.sleep(0.05)   # small gap between uploads

        print("All waypoints uploaded.")
        return True

    def start_mission(self):
        self.link.send_mission_start()
        print("Mission START command sent.")

    def abort_mission(self):
        self.link.send_mission_abort()
        print("Mission ABORT / RTH command sent.")


# ─────────────────────────────────────────────────────────────────
# Telemetry Display
# ─────────────────────────────────────────────────────────────────
def display_telemetry(link):
    """Print live telemetry to terminal."""
    os.system('clear')
    att = link.attitude
    gps = link.gps
    bat = link.battery
    nav = NAV_STATES.get(link.nav_state, 'UNKNOWN')
    geo = GEO_STATES.get(link.geo_status, 'UNKNOWN')

    print("═" * 55)
    print("  DRONE GROUND STATION — Experiment 3")
    print("═" * 55)
    print(f"  NAV STATE : {nav:12s}  WP Index : {link.wp_idx}")
    print(f"  GEOFENCE  : {geo}")
    print("─" * 55)
    print(f"  ATTITUDE  Roll:{att['roll']:7.2f}°  "
          f"Pitch:{att['pitch']:7.2f}°  "
          f"Yaw:{att['yaw']:7.2f}°")
    print(f"  ALTITUDE  {att['alt']:.2f} m")
    print("─" * 55)
    print(f"  GPS       Lat:{gps['lat']:.6f}  Lon:{gps['lon']:.6f}")
    print(f"            Speed: {gps['speed']:.1f} m/s")
    print("─" * 55)
    print(f"  BATTERY   {bat['voltage']:.2f} V  {bat['current']:.1f} A")

    # Battery warning
    if bat['voltage'] > 0:
        if bat['voltage'] < 10.2:
            print("  ⚠️  CRITICAL BATTERY — LANDING")
        elif bat['voltage'] < 10.8:
            print("  ⚠️  LOW BATTERY — RTH")

    # Geofence warning
    if link.geo_status == 1:
        print("  ⚠️  GEOFENCE WARNING — near boundary")
    elif link.geo_status == 2:
        print("  🚨  GEOFENCE BREACH — RTH triggered")

    print("─" * 55)
    print("  Commands: [S]tart  [A]bort/RTH  [Q]uit")
    print("═" * 55)


# ─────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────
def main():
    print("Connecting to STM32...")
    try:
        link = STM32Link('/dev/ttyTHS1', 115200)
        print("Connected!")
    except Exception as e:
        print(f"UART Error: {e}")
        return

    planner = MissionPlanner(link)

    # Load and upload mission
    mission_file = 'mission.json'
    if os.path.exists(mission_file):
        print(f"\nFound {mission_file} — uploading...")
        if planner.upload_from_file(mission_file):
            print("\nMission ready. Press S to start, Q to quit.")
    else:
        print(f"\nNo {mission_file} found.")
        print("Create mission.json and restart, or use commands below.")

    t_hb      = time.time()
    t_display = time.time()

    try:
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        tty.setraw(fd)
        while True:
            # Heartbeat
            if time.time() - t_hb >= 0.5:
                link.send_heartbeat()
                t_hb = time.time()

            # Display update
            if time.time() - t_display >= 0.5:
                display_telemetry(link)
                t_display = time.time()

            # Non-blocking key check
            import select
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1).lower()
                if ch == 'q':
                    print("\nQuitting...")
                    break
                elif ch == 's':
                    planner.start_mission()
                elif ch == 'a':
                    planner.abort_mission()
                elif ch == 'r':
                    link.send_rth()
                    print("RTH command sent.")

            time.sleep(0.05)

    except ImportError:
        # Fallback for environments without tty
        print("\nRunning in simple mode (no keyboard input).")
        print("Edit mission.json and restart to change mission.")
        while True:
            if time.time() - t_hb >= 0.5:
                link.send_heartbeat(); t_hb = time.time()
            if time.time() - t_display >= 1.0:
                display_telemetry(link); t_display = time.time()
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except Exception:
            pass


if __name__ == '__main__':
    main()
