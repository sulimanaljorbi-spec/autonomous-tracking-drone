"""
Drone Mission Simulator - Experiment 3
Simulates the full drone mission on screen with no hardware needed.

Shows:
- Drone moving between waypoints on a 2D map
- Navigation state machine transitions
- Geofence boundary
- Battery draining
- RTH triggered automatically
- Live telemetry panel

Run: python3 drone_simulator.py
Controls:
  SPACE = start mission
  R     = trigger RTH manually
  B     = trigger low battery warning
  G     = trigger geofence breach
  +/-   = zoom map in/out
  Q     = quit
"""

import cv2
import math
import time
import numpy as np

# ─────────────────────────────────────────────────────────────────
# Mission Waypoints
# ─────────────────────────────────────────────────────────────────
HOME_LAT = 24.713600
HOME_LON = 46.675300

WAYPOINTS = [
    {'lat': 24.714200, 'lon': 46.674500, 'alt': 10,
     'action': 'hover',      'hover_time': 5, 'speed': 3.0, 'accept_radius': 2.0},
    {'lat': 24.714800, 'lon': 46.676500, 'alt': 15,
     'action': 'flythrough', 'hover_time': 0, 'speed': 5.0, 'accept_radius': 3.0},
    {'lat': 24.713200, 'lon': 46.677000, 'alt': 10,
     'action': 'hover',      'hover_time': 3, 'speed': 3.0, 'accept_radius': 2.0},
    {'lat': 24.713600, 'lon': 46.675300, 'alt': 5,
     'action': 'land',       'hover_time': 0, 'speed': 2.0, 'accept_radius': 1.5},
]

GEOFENCE_RADIUS = 150.0
RTH_ALTITUDE    = 30.0
LAND_THRESHOLD  = 0.5

MAP_W = 700
MAP_H = 600

# ─────────────────────────────────────────────────────────────────
# GPS Math
# ─────────────────────────────────────────────────────────────────
R_EARTH = 6371000.0

def gps_distance(lat1, lon1, lat2, lon2):
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat/2)**2
         + math.cos(math.radians(lat1))
         * math.cos(math.radians(lat2))
         * math.sin(dlon/2)**2)
    return R_EARTH * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def gps_bearing(lat1, lon1, lat2, lon2):
    dlon  = math.radians(lon2 - lon1)
    lat1r = math.radians(lat1)
    lat2r = math.radians(lat2)
    y = math.sin(dlon) * math.cos(lat2r)
    x = math.cos(lat1r)*math.sin(lat2r) - math.sin(lat1r)*math.cos(lat2r)*math.cos(dlon)
    return math.degrees(math.atan2(y, x)) % 360

def move_gps(lat, lon, bearing_deg, distance_m):
    bearing = math.radians(bearing_deg)
    lat_r   = math.radians(lat)
    lon_r   = math.radians(lon)
    d_r     = distance_m / R_EARTH
    new_lat = math.asin(math.sin(lat_r)*math.cos(d_r)
                        + math.cos(lat_r)*math.sin(d_r)*math.cos(bearing))
    new_lon = lon_r + math.atan2(
        math.sin(bearing)*math.sin(d_r)*math.cos(lat_r),
        math.cos(d_r) - math.sin(lat_r)*math.sin(new_lat))
    return math.degrees(new_lat), math.degrees(new_lon)

def latlon_to_px(lat, lon, ref_lat, ref_lon, scale):
    dy = gps_distance(ref_lat, ref_lon, lat,     ref_lon)
    dx = gps_distance(ref_lat, ref_lon, ref_lat, lon)
    if lat < ref_lat: dy = -dy
    if lon < ref_lon: dx = -dx
    return int(MAP_W//2 + dx*scale), int(MAP_H//2 - dy*scale)


# ─────────────────────────────────────────────────────────────────
# Drone State
# ─────────────────────────────────────────────────────────────────
class DroneState:
    def __init__(self):
        self.lat     = HOME_LAT
        self.lon     = HOME_LON
        self.alt     = 0.0
        self.yaw     = 0.0
        self.roll    = 0.0
        self.pitch   = 0.0
        self.speed   = 0.0
        self.battery = 12.6

        self.nav_state      = 'IDLE'
        self.wp_idx         = 0
        self.hover_start    = None
        self.rth_phase      = 0
        self.mission_active = False
        self.geo_status     = 'OK'

        self.home_lat = HOME_LAT
        self.home_lon = HOME_LON
        self.home_alt = 0.0

        self.log     = []
        self.t_start = time.time()

    def log_event(self, msg):
        t = time.time() - self.t_start
        entry = "[%6.1fs] %s" % (t, msg)
        self.log.append(entry)
        if len(self.log) > 12:
            self.log.pop(0)
        print(msg)

    def start_mission(self):
        if self.nav_state == 'IDLE':
            self.home_lat       = self.lat
            self.home_lon       = self.lon
            self.home_alt       = self.alt
            self.wp_idx         = 0
            self.mission_active = True
            self.nav_state      = 'TAKEOFF'
            self.log_event("Mission START - taking off")

    def trigger_rth(self, reason="Manual"):
        self.nav_state      = 'RTH'
        self.rth_phase      = 0
        self.mission_active = False
        self.log_event("RTH triggered: %s" % reason)

    def trigger_low_battery(self):
        self.battery = 10.5
        self.log_event("LOW BATTERY - RTH triggered")
        self.trigger_rth("Low battery")

    def trigger_geofence_breach(self):
        self.geo_status = 'BREACH'
        self.log_event("GEOFENCE BREACH - RTH triggered")
        self.trigger_rth("Geofence breach")

    def update(self, dt):

        # Battery drain
        self.battery -= 0.001 * dt
        self.battery  = max(self.battery, 9.0)

        # Battery failsafe
        if self.battery < 10.2 and self.nav_state not in ('RTH', 'LAND', 'IDLE'):
            self.log_event("CRITICAL BATTERY - LAND NOW")
            self.nav_state = 'LAND'
        elif self.battery < 10.8 and self.nav_state not in ('RTH', 'LAND', 'IDLE'):
            self.log_event("LOW BATTERY - RTH")
            self.trigger_rth("Battery warning")

        # Geofence check
        dist_home = gps_distance(self.lat, self.lon, self.home_lat, self.home_lon)
        if dist_home > GEOFENCE_RADIUS:
            if self.geo_status != 'BREACH':
                self.log_event("GEOFENCE BREACH - RTH")
                self.trigger_rth("Geofence")
            self.geo_status = 'BREACH'
        elif dist_home > GEOFENCE_RADIUS * 0.8:
            self.geo_status = 'WARNING'
        else:
            self.geo_status = 'OK'

        wp = WAYPOINTS[self.wp_idx] if self.wp_idx < len(WAYPOINTS) else None

        # ── Navigation State Machine ──────────────────────────────
        if self.nav_state == 'IDLE':
            pass

        elif self.nav_state == 'TAKEOFF':
            target_alt = WAYPOINTS[0]['alt']
            self.alt  += 3.0 * dt
            self.pitch = -5.0
            if self.alt >= target_alt:
                self.alt       = target_alt
                self.pitch     = 0.0
                self.nav_state = 'NAVIGATE'
                self.log_event("Takeoff complete - navigating to WP1")

        elif self.nav_state == 'NAVIGATE' and wp:
            dist    = gps_distance(self.lat, self.lon, wp['lat'], wp['lon'])
            bearing = gps_bearing(self.lat, self.lon, wp['lat'], wp['lon'])
            self.yaw   = bearing
            spd        = max(min(wp['speed'], dist * 0.5), 0.5)
            self.speed = spd
            self.pitch = -8.0
            self.lat, self.lon = move_gps(self.lat, self.lon, bearing, spd * dt)

            # Altitude adjust
            diff = wp['alt'] - self.alt
            if abs(diff) > 0.2:
                self.alt += math.copysign(min(1.0*dt, abs(diff)), diff)

            if dist < wp['accept_radius']:
                self.pitch = 0.0
                self.speed = 0.0
                action     = wp['action']
                self.log_event("Arrived WP%d (%s)" % (self.wp_idx+1, action))
                if action == 'hover':
                    self.nav_state   = 'HOVER'
                    self.hover_start = time.time()
                elif action == 'land':
                    self.nav_state = 'LAND'
                elif action == 'rth':
                    self.trigger_rth("Waypoint action")
                else:
                    self.nav_state = 'ADVANCE'

        elif self.nav_state == 'HOVER' and wp:
            self.speed   = 0.0
            elapsed      = time.time() - self.hover_start
            remaining    = wp['hover_time'] - elapsed
            self.log_event("HOVER WP%d - %.0fs left" % (self.wp_idx+1, max(remaining,0)))
            if remaining <= 0:
                self.log_event("Hover complete WP%d" % (self.wp_idx+1))
                self.nav_state = 'ADVANCE'

        elif self.nav_state == 'ADVANCE':
            self.wp_idx += 1
            if self.wp_idx >= len(WAYPOINTS):
                self.log_event("Mission complete - RTH")
                self.trigger_rth("Mission complete")
            else:
                self.log_event("Advancing to WP%d" % (self.wp_idx+1))
                self.nav_state = 'NAVIGATE'

        elif self.nav_state == 'RTH':
            if self.rth_phase == 0:
                self.alt   += 3.0 * dt
                self.pitch  = -5.0
                if self.alt >= RTH_ALTITUDE:
                    self.alt      = RTH_ALTITUDE
                    self.rth_phase = 1
                    self.log_event("RTH: climb done - flying home")

            elif self.rth_phase == 1:
                dist    = gps_distance(self.lat, self.lon, self.home_lat, self.home_lon)
                bearing = gps_bearing(self.lat, self.lon, self.home_lat, self.home_lon)
                self.yaw   = bearing
                self.speed = min(3.0, dist * 0.3)
                self.pitch = -8.0
                self.lat, self.lon = move_gps(self.lat, self.lon, bearing, self.speed*dt)
                if dist < 2.0:
                    self.pitch    = 0.0
                    self.rth_phase = 2
                    self.log_event("RTH: over home - descending")

            elif self.rth_phase == 2:
                self.alt  -= 2.0 * dt
                self.speed = 0.0
                self.pitch = 0.0
                if self.alt <= LAND_THRESHOLD:
                    self.alt            = 0.0
                    self.nav_state      = 'IDLE'
                    self.mission_active = False
                    self.log_event("RTH complete - landed safely")

        elif self.nav_state == 'LAND':
            self.alt  -= 1.5 * dt
            self.speed = 0.0
            self.pitch = 0.0
            if self.alt <= LAND_THRESHOLD:
                self.alt            = 0.0
                self.nav_state      = 'IDLE'
                self.mission_active = False
                self.log_event("Landed.")

        # Simulate roll wobble when flying
        if self.nav_state in ('NAVIGATE', 'RTH'):
            self.roll = math.sin(time.time() * 0.8) * 3.0
        else:
            self.roll = math.sin(time.time() * 0.3) * 1.0


# ─────────────────────────────────────────────────────────────────
# Draw Map Panel
# ─────────────────────────────────────────────────────────────────
def draw_map(drone, scale):
    canvas   = np.zeros((MAP_H, MAP_W, 3), dtype=np.uint8)
    canvas[:] = (20, 20, 20)
    ref_lat   = HOME_LAT
    ref_lon   = HOME_LON

    # Grid
    for i in range(-6, 7):
        x = MAP_W//2 + int(i * 50 * scale)
        y = MAP_H//2 + int(i * 50 * scale)
        cv2.line(canvas, (x, 0),     (x, MAP_H), (32, 32, 32), 1)
        cv2.line(canvas, (0, y),     (MAP_W, y), (32, 32, 32), 1)
    for i in range(-6, 7):
        x = MAP_W//2 + int(i * 50 * scale)
        if 0 < x < MAP_W:
            cv2.putText(canvas, "%dm" % (i*50), (x-12, MAP_H-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.25, (70,70,70), 1)

    home_px, home_py = latlon_to_px(HOME_LAT, HOME_LON, ref_lat, ref_lon, scale)

    # Geofence circles
    geo_px  = int(GEOFENCE_RADIUS * scale)
    warn_px = int(GEOFENCE_RADIUS * 0.8 * scale)
    geo_col = (0,80,0) if drone.geo_status=='OK' else \
              (0,120,200) if drone.geo_status=='WARNING' else (0,0,200)
    cv2.circle(canvas, (home_px, home_py), geo_px,  geo_col, 1)
    cv2.circle(canvas, (home_px, home_py), warn_px, (0,55,90), 1)
    cv2.putText(canvas, "Geofence %dm" % int(GEOFENCE_RADIUS),
                (home_px + geo_px + 4, home_py),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, geo_col, 1)

    # Waypoint path lines
    wp_pixels = [latlon_to_px(wp['lat'], wp['lon'], ref_lat, ref_lon, scale)
                 for wp in WAYPOINTS]
    for i in range(len(wp_pixels)-1):
        cv2.line(canvas, wp_pixels[i], wp_pixels[i+1], (55,55,55), 1)
    # Close path back to home
    cv2.line(canvas, wp_pixels[-1], (home_px, home_py), (40,40,40), 1)

    # Waypoints
    for i, (px, py) in enumerate(wp_pixels):
        active = (i == drone.wp_idx and
                  drone.nav_state not in ('IDLE','RTH','LAND','ADVANCE'))
        color  = (0,255,255) if active else (0,180,180)
        fill   = -1 if active else 2
        cv2.circle(canvas, (px, py), 10, color, fill)
        cv2.putText(canvas, "WP%d" % (i+1), (px+13, py+4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
        cv2.putText(canvas, WAYPOINTS[i]['action'].upper(), (px+13, py+18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.28, (100,100,100), 1)

    # Home marker
    cv2.drawMarker(canvas, (home_px, home_py), (0,255,0),
                   cv2.MARKER_CROSS, 22, 2)
    cv2.putText(canvas, "HOME", (home_px+8, home_py-8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0,255,0), 1)

    # Drone trail / position
    dpx, dpy = latlon_to_px(drone.lat, drone.lon, ref_lat, ref_lon, scale)

    # Heading arrow
    head_rad = math.radians(drone.yaw)
    ax = int(dpx + 22 * math.sin(head_rad))
    ay = int(dpy - 22 * math.cos(head_rad))
    cv2.arrowedLine(canvas, (dpx, dpy), (ax, ay), (0,200,255), 2, tipLength=0.4)

    # Drone body
    drone_color = (0,200,255)
    if drone.nav_state == 'RTH':  drone_color = (0,140,255)
    if drone.nav_state == 'LAND': drone_color = (0,80,200)
    cv2.circle(canvas, (dpx, dpy),  8, drone_color, -1)
    cv2.circle(canvas, (dpx, dpy), 13, drone_color, 1)

    # Drone label
    cv2.putText(canvas, drone.nav_state, (dpx+15, dpy-5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, drone_color, 1)

    # Altitude bar (right side)
    bx = MAP_W - 28; by0 = 40; bh = MAP_H - 80; bw = 14
    max_alt = 40.0
    frac    = min(max(drone.alt / max_alt, 0.0), 1.0)
    fill_h  = int(frac * bh)
    cv2.rectangle(canvas, (bx, by0),          (bx+bw, by0+bh), (35,35,35), -1)
    cv2.rectangle(canvas, (bx, by0+bh-fill_h),(bx+bw, by0+bh), (0,200,100), -1)
    cv2.rectangle(canvas, (bx, by0),          (bx+bw, by0+bh), (70,70,70),  1)
    cv2.putText(canvas, "%.0fm" % drone.alt,
                (bx-8, by0+bh+14), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (170,170,170), 1)
    cv2.putText(canvas, "ALT", (bx-1, by0-5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.28, (100,100,100), 1)

    return canvas


# ─────────────────────────────────────────────────────────────────
# Draw Telemetry Panel
# ─────────────────────────────────────────────────────────────────
def draw_telemetry(drone):
    panel    = np.zeros((MAP_H, 390, 3), dtype=np.uint8)
    panel[:] = (14, 14, 14)

    def txt(msg, y, color=(200,200,200), scale=0.45, bold=False):
        cv2.putText(panel, str(msg), (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2 if bold else 1)

    def hline(y):
        cv2.line(panel, (0,y), (390,y), (45,45,45), 1)

    # Title
    txt("EXPERIMENT 3 - SIMULATOR", 24, (255,255,255), 0.52, True)
    hline(34)

    # NAV state
    NAV_COL = {
        'IDLE':     (110,110,110),
        'TAKEOFF':  (0,210,255),
        'NAVIGATE': (0,255,100),
        'HOVER':    (0,255,255),
        'ADVANCE':  (0,200,100),
        'RTH':      (0,150,255),
        'LAND':     (0,100,210),
    }
    nc = NAV_COL.get(drone.nav_state, (200,200,200))
    txt("NAV STATE", 58, (140,140,140), 0.37)
    txt(drone.nav_state, 82, nc, 0.72, True)
    if drone.wp_idx < len(WAYPOINTS):
        txt("Waypoint %d of %d  action: %s" % (
            drone.wp_idx+1, len(WAYPOINTS),
            WAYPOINTS[drone.wp_idx]['action']),
            102, (160,160,160), 0.37)
    hline(114)

    # GPS
    txt("GPS POSITION", 132, (140,140,140), 0.37)
    txt("Lat: %.6f" % drone.lat,  150, (190,215,190), 0.42)
    txt("Lon: %.6f" % drone.lon,  168, (190,215,190), 0.42)
    txt("Alt: %.1f m" % drone.alt, 186, (190,215,190), 0.42)
    dist_home = gps_distance(drone.lat, drone.lon, drone.home_lat, drone.home_lon)
    txt("Dist from home: %.1f m" % dist_home, 204, (165,165,165), 0.38)
    hline(216)

    # Attitude
    txt("ATTITUDE", 233, (140,140,140), 0.37)
    txt("Roll:  %+.1f deg" % drone.roll,  250, (190,190,215), 0.42)
    txt("Pitch: %+.1f deg" % drone.pitch, 268, (190,190,215), 0.42)
    txt("Yaw:   %.1f deg"  % drone.yaw,   286, (190,190,215), 0.42)
    txt("Speed: %.1f m/s"  % drone.speed, 304, (190,190,215), 0.42)
    hline(316)

    # Battery
    txt("BATTERY", 334, (140,140,140), 0.37)
    if drone.battery < 10.2:
        bc = (0,0,255);   bl = "CRITICAL - LAND"
    elif drone.battery < 10.8:
        bc = (0,140,255); bl = "LOW - RTH"
    else:
        bc = (0,220,0);   bl = "GOOD"
    txt("%.2f V  %s" % (drone.battery, bl), 354, bc, 0.47, True)

    # Battery bar
    bx=10; by=364; bw=355; bh=13
    cv2.rectangle(panel, (bx,by), (bx+bw,by+bh), (38,38,38), -1)
    frac = max(0.0, (drone.battery - 9.0) / (12.6 - 9.0))
    cv2.rectangle(panel, (bx,by), (bx+int(frac*bw),by+bh), bc, -1)
    cv2.rectangle(panel, (bx,by), (bx+bw,by+bh), (75,75,75), 1)
    # Threshold markers
    warn_x = int(((10.8-9.0)/(12.6-9.0)) * bw) + bx
    crit_x = int(((10.2-9.0)/(12.6-9.0)) * bw) + bx
    cv2.line(panel, (warn_x,by), (warn_x,by+bh), (0,140,255), 1)
    cv2.line(panel, (crit_x,by), (crit_x,by+bh), (0,0,255), 1)
    hline(386)

    # Geofence
    txt("GEOFENCE", 404, (140,140,140), 0.37)
    GC = {'OK':(0,220,0), 'WARNING':(0,140,255), 'BREACH':(0,0,255)}
    gc = GC.get(drone.geo_status, (200,200,200))
    txt("%s  (%.0fm / %.0fm radius)" % (
        drone.geo_status, dist_home, GEOFENCE_RADIUS),
        422, gc, 0.42)
    hline(434)

    # Event log
    txt("EVENT LOG", 450, (140,140,140), 0.36)
    for i, entry in enumerate(drone.log[-7:]):
        txt(entry, 468 + i*16, (150,150,150), 0.30)

    hline(MAP_H-38)
    txt("SPACE=Start  R=RTH  B=Battery  G=Geofence  +/-=Zoom  Q=Quit",
        MAP_H-20, (90,90,90), 0.30)

    return panel


# ─────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────
def main():
    drone  = DroneState()
    t_prev = time.time()
    scale  = 4.0

    print("Drone Mission Simulator - Experiment 3")
    print("Press SPACE to start the mission")
    print("Press Q to quit\n")

    while True:
        t_now  = time.time()
        dt     = min(t_now - t_prev, 0.1)
        t_prev = t_now

        drone.update(dt)

        map_frame   = draw_map(drone, scale)
        telem_frame = draw_telemetry(drone)
        combined    = np.hstack([map_frame, telem_frame])

        cv2.imshow("Drone Mission Simulator - Experiment 3", combined)

        key = cv2.waitKey(33) & 0xFF
        if   key == ord('q'):
            break
        elif key == ord(' '):
            drone.start_mission()
        elif key == ord('r'):
            drone.trigger_rth("Manual RC switch")
        elif key == ord('b'):
            drone.trigger_low_battery()
        elif key == ord('g'):
            drone.trigger_geofence_breach()
        elif key in (ord('+'), ord('=')):
            scale = min(scale * 1.2, 20.0)
        elif key == ord('-'):
            scale = max(scale / 1.2, 1.0)

    cv2.destroyAllWindows()
    print("Simulator closed.")


if __name__ == '__main__':
    main()
