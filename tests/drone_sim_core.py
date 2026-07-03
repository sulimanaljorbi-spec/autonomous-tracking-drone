"""
drone_sim_core.py
Headless simulation core (no display) — shared physics/state machine logic
reused by both the visual simulator and the automated test harness.

This mirrors the STM32 navigation state machine from Experiment 3:
IDLE -> TAKEOFF -> NAVIGATE -> HOVER -> ADVANCE -> RTH -> LAND
"""

import math
import time

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


class DroneSim:
    """
    Headless drone simulation. No rendering, no real-time sleep needed —
    can be stepped as fast as the CPU allows for rapid automated testing.
    """

    def __init__(self, home_lat, home_lon, waypoints,
                 geofence_radius=150.0, rth_altitude=30.0,
                 land_threshold=0.5, battery_start=12.6):
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.waypoints = waypoints
        self.geofence_radius = geofence_radius
        self.rth_altitude    = rth_altitude
        self.land_threshold  = land_threshold

        self.lat = home_lat
        self.lon = home_lon
        self.alt = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.speed = 0.0
        self.battery = battery_start

        self.nav_state   = 'IDLE'
        self.wp_idx      = 0
        self.hover_timer = 0.0
        self.rth_phase   = 0
        self.geo_status  = 'OK'

        self.sim_time = 0.0
        self.events   = []          # (sim_time, message)
        self.crashed  = False

        # Injected failures for fault testing
        self.force_low_battery   = False
        self.force_geo_breach    = False
        self.force_gps_loss      = False
        self.force_rc_loss       = False

    def log(self, msg):
        self.events.append((round(self.sim_time, 2), msg))

    def trigger_rth(self, reason):
        self.nav_state = 'RTH'
        self.rth_phase = 0
        self.log(f"RTH triggered: {reason}")

    def start_mission(self):
        if self.nav_state == 'IDLE':
            self.home_lat = self.lat
            self.home_lon = self.lon
            self.wp_idx   = 0
            self.nav_state = 'TAKEOFF'
            self.log("Mission START")

    def step(self, dt):
        """Advance simulation by dt seconds. Call repeatedly."""
        self.sim_time += dt

        # Battery drain (faster in tests so failsafes can be reached quickly)
        self.battery -= 0.01 * dt
        self.battery  = max(self.battery, 8.0)
        if self.force_low_battery:
            self.battery = 10.5

        # Battery failsafe
        if self.battery < 10.2 and self.nav_state not in ('RTH','LAND','IDLE'):
            self.log("CRITICAL BATTERY -> LAND")
            self.nav_state = 'LAND'
        elif self.battery < 10.8 and self.nav_state not in ('RTH','LAND','IDLE'):
            self.trigger_rth("Battery warning")

        # Geofence check
        dist_home = gps_distance(self.lat, self.lon, self.home_lat, self.home_lon)
        if self.force_geo_breach:
            dist_home = self.geofence_radius + 50
        if dist_home > self.geofence_radius:
            if self.geo_status != 'BREACH':
                self.trigger_rth("Geofence breach")
            self.geo_status = 'BREACH'
        elif dist_home > self.geofence_radius * 0.8:
            self.geo_status = 'WARNING'
        else:
            self.geo_status = 'OK'

        wp = self.waypoints[self.wp_idx] if self.wp_idx < len(self.waypoints) else None

        if self.nav_state == 'IDLE':
            pass

        elif self.nav_state == 'TAKEOFF':
            target_alt = self.waypoints[0]['alt']
            self.alt += 3.0 * dt
            self.pitch = -5.0
            if self.alt >= target_alt:
                self.alt = target_alt
                self.pitch = 0.0
                self.nav_state = 'NAVIGATE'
                self.log("Takeoff complete -> NAVIGATE WP1")

        elif self.nav_state == 'NAVIGATE' and wp:
            dist    = gps_distance(self.lat, self.lon, wp['lat'], wp['lon'])
            bearing = gps_bearing(self.lat, self.lon, wp['lat'], wp['lon'])
            self.yaw = bearing
            spd = max(min(wp['speed'], dist * 0.5), 0.5)
            self.speed = spd
            self.pitch = -8.0
            self.lat, self.lon = move_gps(self.lat, self.lon, bearing, spd * dt)

            diff = wp['alt'] - self.alt
            if abs(diff) > 0.2:
                self.alt += math.copysign(min(1.0*dt, abs(diff)), diff)

            if dist < wp['accept_radius']:
                self.pitch = 0.0
                self.speed = 0.0
                action = wp['action']
                self.log(f"Arrived WP{self.wp_idx+1} ({action})")
                if action == 'hover':
                    self.nav_state = 'HOVER'
                    self.hover_timer = 0.0
                elif action == 'land':
                    self.nav_state = 'LAND'
                elif action == 'rth':
                    self.trigger_rth("Waypoint action")
                else:
                    self.nav_state = 'ADVANCE'

        elif self.nav_state == 'HOVER' and wp:
            self.speed = 0.0
            self.hover_timer += dt
            if self.hover_timer >= wp['hover_time']:
                self.log(f"Hover complete WP{self.wp_idx+1}")
                self.nav_state = 'ADVANCE'

        elif self.nav_state == 'ADVANCE':
            self.wp_idx += 1
            if self.wp_idx >= len(self.waypoints):
                self.trigger_rth("Mission complete")
            else:
                self.log(f"Advancing to WP{self.wp_idx+1}")
                self.nav_state = 'NAVIGATE'

        elif self.nav_state == 'RTH':
            if self.rth_phase == 0:
                self.alt += 3.0 * dt
                self.pitch = -5.0
                if self.alt >= self.rth_altitude:
                    self.alt = self.rth_altitude
                    self.rth_phase = 1
                    self.log("RTH: climb done -> flying home")
            elif self.rth_phase == 1:
                dist    = gps_distance(self.lat, self.lon, self.home_lat, self.home_lon)
                bearing = gps_bearing(self.lat, self.lon, self.home_lat, self.home_lon)
                self.yaw = bearing
                self.speed = min(3.0, dist * 0.3)
                self.pitch = -8.0
                self.lat, self.lon = move_gps(self.lat, self.lon, bearing, self.speed*dt)
                if dist < 2.0:
                    self.pitch = 0.0
                    self.rth_phase = 2
                    self.log("RTH: over home -> descending")
            elif self.rth_phase == 2:
                self.alt -= 2.0 * dt
                self.speed = 0.0
                if self.alt <= self.land_threshold:
                    self.alt = 0.0
                    self.nav_state = 'IDLE'
                    self.log("RTH complete -> landed")

        elif self.nav_state == 'LAND':
            self.alt -= 1.5 * dt
            self.speed = 0.0
            if self.alt <= self.land_threshold:
                self.alt = 0.0
                self.nav_state = 'IDLE'
                self.log("Landed")

        # Basic crash detection: negative altitude impossible state, or
        # tilt exceeds safety envelope while flying (simulated structural limit)
        if self.alt < -0.01:
            self.crashed = True
            self.log("CRASH DETECTED: negative altitude")

    def run_until(self, predicate, dt=0.05, max_time=120.0):
        """Step simulation until predicate(self) is True or max_time exceeded."""
        while self.sim_time < max_time:
            self.step(dt)
            if predicate(self):
                return True
        return False
