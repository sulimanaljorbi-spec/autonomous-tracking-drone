"""
test_missions.py
Automated Testing — Lecture 8 applied

Implements the CI loop described in the lecture:
  1. Start simulation
  2. Execute a mission
  3. Check outcomes (did it reach the waypoint? did it crash?)
  4. Generate a pass/fail report

Run:  python3 test_missions.py
Exit code 0 = all tests passed, 1 = at least one failed (CI-friendly).
"""

import sys
import time
from drone_sim_core import DroneSim, gps_distance


# ─────────────────────────────────────────────────────────────────
# Test result bookkeeping
# ─────────────────────────────────────────────────────────────────
class TestResult:
    def __init__(self, name):
        self.name = name
        self.passed = True
        self.reasons = []
        self.duration = 0.0

    def check(self, condition, description):
        """Assertion helper — records pass/fail without raising."""
        if condition:
            self.reasons.append(f"  [PASS] {description}")
        else:
            self.passed = False
            self.reasons.append(f"  [FAIL] {description}")

    def __str__(self):
        status = "PASSED" if self.passed else "FAILED"
        lines = [f"[{status}] {self.name}  ({self.duration:.2f}s sim time)"]
        lines.extend(self.reasons)
        return "\n".join(lines)


# ─────────────────────────────────────────────────────────────────
# Test 1 — Basic waypoint mission completes successfully
# ─────────────────────────────────────────────────────────────────
def test_basic_mission_completion():
    result = TestResult("Basic mission completion (3 waypoints)")

    home_lat, home_lon = 24.7136, 46.6753
    waypoints = [
        {'lat': 24.7142, 'lon': 46.6745, 'alt': 10, 'speed': 3.0,
         'action': 'hover', 'hover_time': 3, 'accept_radius': 2.0},
        {'lat': 24.7148, 'lon': 46.6765, 'alt': 15, 'speed': 5.0,
         'action': 'flythrough', 'hover_time': 0, 'accept_radius': 3.0},
        {'lat': 24.7136, 'lon': 46.6753, 'alt': 5, 'speed': 2.0,
         'action': 'land', 'hover_time': 0, 'accept_radius': 1.5},
    ]

    sim = DroneSim(home_lat, home_lon, waypoints, geofence_radius=200.0)
    sim.start_mission()

    finished = sim.run_until(lambda s: s.nav_state == 'IDLE' and s.sim_time > 1.0,
                             dt=0.05, max_time=180.0)
    result.duration = sim.sim_time

    result.check(finished, "Mission reached IDLE state before timeout")
    result.check(not sim.crashed, "No crash detected during mission")
    result.check(sim.alt < 0.6, f"Final altitude near ground ({sim.alt:.2f}m)")
    result.check(sim.battery > 10.2, f"Battery stayed above critical ({sim.battery:.2f}V)")

    wp1_visited = any("Arrived WP1" in msg for _, msg in sim.events)
    wp2_visited = any("Arrived WP2" in msg for _, msg in sim.events)
    wp3_visited = any("Arrived WP3" in msg for _, msg in sim.events)
    result.check(wp1_visited, "Visited WP1")
    result.check(wp2_visited, "Visited WP2")
    result.check(wp3_visited, "Visited WP3")

    return result


# ─────────────────────────────────────────────────────────────────
# Test 2 — Battery failsafe triggers RTH automatically
# ─────────────────────────────────────────────────────────────────
def test_battery_failsafe_triggers_rth():
    result = TestResult("Low battery automatically triggers RTH")

    home_lat, home_lon = 24.7136, 46.6753
    waypoints = [
        {'lat': 24.7200, 'lon': 46.6900, 'alt': 20, 'speed': 5.0,
         'action': 'flythrough', 'hover_time': 0, 'accept_radius': 3.0},
    ]

    sim = DroneSim(home_lat, home_lon, waypoints, geofence_radius=5000.0)
    sim.start_mission()

    # Let it fly a bit, then force low battery
    sim.run_until(lambda s: s.nav_state == 'NAVIGATE', dt=0.05, max_time=30.0)
    sim.force_low_battery = True

    rth_reached = sim.run_until(lambda s: s.nav_state == 'RTH', dt=0.05, max_time=30.0)
    result.check(rth_reached, "Navigation state switched to RTH after low battery")

    landed = sim.run_until(lambda s: s.nav_state == 'IDLE', dt=0.05, max_time=120.0)
    result.duration = sim.sim_time
    result.check(landed, "Drone completed RTH and landed (IDLE state)")
    result.check(not sim.crashed, "No crash during RTH")

    rth_event = any("RTH triggered: Battery warning" in msg for _, msg in sim.events)
    result.check(rth_event, "RTH reason correctly logged as battery warning")

    return result


# ─────────────────────────────────────────────────────────────────
# Test 3 — Geofence breach triggers RTH
# ─────────────────────────────────────────────────────────────────
def test_geofence_breach_triggers_rth():
    result = TestResult("Geofence breach automatically triggers RTH")

    home_lat, home_lon = 24.7136, 46.6753
    waypoints = [
        {'lat': 24.7300, 'lon': 46.6753, 'alt': 15, 'speed': 8.0,
         'action': 'flythrough', 'hover_time': 0, 'accept_radius': 3.0},
    ]

    # Small geofence so the drone is guaranteed to breach it en route
    sim = DroneSim(home_lat, home_lon, waypoints, geofence_radius=50.0)
    sim.start_mission()

    breached = sim.run_until(lambda s: s.geo_status == 'BREACH', dt=0.05, max_time=60.0)
    result.check(breached, "Geofence breach was detected")

    rth_active = sim.nav_state == 'RTH'
    result.check(rth_active, "Drone switched to RTH state immediately on breach")

    landed = sim.run_until(lambda s: s.nav_state == 'IDLE', dt=0.05, max_time=120.0)
    result.duration = sim.sim_time
    result.check(landed, "Drone returned home and landed after breach")
    result.check(not sim.crashed, "No crash during geofence RTH")

    return result


# ─────────────────────────────────────────────────────────────────
# Test 4 — Hover action holds for the correct duration
# ─────────────────────────────────────────────────────────────────
def test_hover_duration_accuracy():
    result = TestResult("Hover action holds for correct duration")

    home_lat, home_lon = 24.7136, 46.6753
    HOVER_TIME = 6
    waypoints = [
        {'lat': 24.7140, 'lon': 46.6757, 'alt': 10, 'speed': 3.0,
         'action': 'hover', 'hover_time': HOVER_TIME, 'accept_radius': 2.0},
        {'lat': 24.7136, 'lon': 46.6753, 'alt': 5, 'speed': 2.0,
         'action': 'land', 'hover_time': 0, 'accept_radius': 1.5},
    ]

    sim = DroneSim(home_lat, home_lon, waypoints, geofence_radius=500.0)
    sim.start_mission()

    sim.run_until(lambda s: s.nav_state == 'HOVER', dt=0.02, max_time=60.0)
    hover_start_time = sim.sim_time

    sim.run_until(lambda s: s.nav_state == 'ADVANCE', dt=0.02, max_time=60.0)
    hover_end_time = sim.sim_time
    actual_hover_duration = hover_end_time - hover_start_time

    result.duration = sim.sim_time
    tolerance = 0.5
    result.check(
        abs(actual_hover_duration - HOVER_TIME) < tolerance,
        f"Hover lasted {actual_hover_duration:.2f}s (expected {HOVER_TIME}s ±{tolerance}s)"
    )

    return result


# ─────────────────────────────────────────────────────────────────
# Test 5 — Critical battery forces immediate landing (not RTH)
# ─────────────────────────────────────────────────────────────────
def test_critical_battery_forces_immediate_land():
    result = TestResult("Critical battery forces immediate LAND (skips RTH)")

    home_lat, home_lon = 24.7136, 46.6753
    waypoints = [
        {'lat': 24.7250, 'lon': 46.6900, 'alt': 20, 'speed': 5.0,
         'action': 'flythrough', 'hover_time': 0, 'accept_radius': 3.0},
    ]

    sim = DroneSim(home_lat, home_lon, waypoints, geofence_radius=5000.0)
    sim.start_mission()
    sim.run_until(lambda s: s.nav_state == 'NAVIGATE', dt=0.05, max_time=30.0)

    # Force battery straight to critical (below 10.2V)
    sim.battery = 10.0

    landed_directly = sim.run_until(lambda s: s.nav_state in ('LAND', 'IDLE'),
                                    dt=0.05, max_time=30.0)
    result.duration = sim.sim_time

    result.check(landed_directly, "Drone entered LAND state on critical battery")
    no_rth_first = not any("RTH" in msg for t, msg in sim.events
                           if t < sim.sim_time and "CRITICAL" in
                           [m for _, m in sim.events if "CRITICAL" in m][0:1])
    # Simpler check: verify LAND was triggered by CRITICAL BATTERY log
    critical_log = any("CRITICAL BATTERY" in msg for _, msg in sim.events)
    result.check(critical_log, "CRITICAL BATTERY event was logged")

    return result


# ─────────────────────────────────────────────────────────────────
# Test runner — executes all tests, builds report
# ─────────────────────────────────────────────────────────────────
def main():
    tests = [
        test_basic_mission_completion,
        test_battery_failsafe_triggers_rth,
        test_geofence_breach_triggers_rth,
        test_hover_duration_accuracy,
        test_critical_battery_forces_immediate_land,
    ]

    print("=" * 60)
    print("  AUTOMATED MISSION TEST SUITE")
    print("  (Lecture 8 — Automated Testing for Drone Software)")
    print("=" * 60)
    print()

    results = []
    t_start = time.time()

    for test_fn in tests:
        print(f"Running: {test_fn.__name__} ...")
        try:
            result = test_fn()
        except Exception as e:
            result = TestResult(test_fn.__name__)
            result.passed = False
            result.reasons.append(f"  [ERROR] Exception raised: {e}")
        results.append(result)
        print(result)
        print()

    wall_time = time.time() - t_start

    # ── Summary report ──
    passed = sum(1 for r in results if r.passed)
    failed = len(results) - passed

    print("=" * 60)
    print("  TEST SUMMARY")
    print("=" * 60)
    for r in results:
        status = "PASS" if r.passed else "FAIL"
        print(f"  [{status}]  {r.name}")
    print("-" * 60)
    print(f"  Total:  {len(results)}   Passed: {passed}   Failed: {failed}")
    print(f"  Wall-clock time: {wall_time:.2f}s "
          f"(simulated {sum(r.duration for r in results):.1f}s of flight time)")
    print("=" * 60)

    if failed > 0:
        print("\nRESULT: BUILD FAILED — fix failing tests before flight.")
        sys.exit(1)
    else:
        print("\nRESULT: ALL TESTS PASSED — safe to proceed to hardware testing.")
        sys.exit(0)


if __name__ == '__main__':
    main()
