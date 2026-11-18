import pytest
from src.aeb_controller import AEBController, AEBState

@pytest.fixture
def aeb():
    return AEBController()

# --- 1. UNIT & FAULT INJECTION TESTS ---

def test_fault_injection_invalid_sensor(aeb):
    """REQ-01: Invalid radar signal must transition the system to FAULT_DEGRADED."""
    state = aeb.update(ego_speed=20.0, distance=50.0, sensor_valid=False)
    assert state == AEBState.FAULT_DEGRADED

def test_fault_injection_negative_inputs(aeb):
    """REQ-01: Negative physical inputs must trigger FAULT_DEGRADED."""
    assert aeb.update(ego_speed=-5.0, distance=30.0) == AEBState.FAULT_DEGRADED
    assert aeb.update(ego_speed=15.0, distance=-10.0) == AEBState.FAULT_DEGRADED

def test_standstill_vehicle_no_division_by_zero(aeb):
    """REQ-02: Standstill vehicle (0 m/s) must remain in NORMAL state without ZeroDivisionError."""
    state = aeb.update(ego_speed=0.0, distance=5.0)
    assert state == AEBState.NORMAL

# --- 2. BOUNDARY VALUE ANALYSIS (BVA) ---

@pytest.mark.parametrize("speed_mps, distance_m, expected_state", [
    (10.0, 25.1, AEBState.NORMAL),             # TTC = 2.51s -> slightly above warning threshold
    (10.0, 25.0, AEBState.WARNING),            # TTC = 2.50s -> exact boundary for WARNING
    (10.0, 15.1, AEBState.WARNING),            # TTC = 1.51s -> slightly above braking threshold
    (10.0, 15.0, AEBState.EMERGENCY_BRAKING),  # TTC = 1.50s -> exact boundary for EMERGENCY_BRAKING
])
def test_ttc_boundary_values(aeb, speed_mps, distance_m, expected_state):
    """REQ-03/04/05: Boundary Value Analysis around TTC = 2.5s and TTC = 1.5s."""
    assert aeb.update(ego_speed=speed_mps, distance=distance_m) == expected_state

# --- 3. CLOSED-LOOP SIL SIMULATION ---

def run_sil_simulation(initial_speed_kmh: float, initial_distance_m: float, deceleration_mps2: float = 8.0, dt: float = 0.1):
    """Helper executing a closed-loop time-stepped braking physics simulation."""
    controller = AEBController()
    speed_mps = initial_speed_kmh / 3.6
    distance_m = initial_distance_m
    time_elapsed = 0.0

    while speed_mps > 0 and distance_m > 0 and time_elapsed < 15.0:
        state = controller.update(ego_speed=speed_mps, distance=distance_m)
        if state == AEBState.EMERGENCY_BRAKING:
            speed_mps = max(0.0, speed_mps - deceleration_mps2 * dt)
        distance_m -= speed_mps * dt
        time_elapsed += dt

    return distance_m, speed_mps, controller.state

def test_sil_urban_scenario_collision_avoided():
    """SIL-01: Urban scenario (54 km/h, obstacle at 30m) — vehicle must stop safely before impact."""
    remaining_dist, final_speed, final_state = run_sil_simulation(
        initial_speed_kmh=54.0,
        initial_distance_m=30.0
    )
    assert final_speed == 0.0
    assert final_state == AEBState.EMERGENCY_BRAKING
    assert remaining_dist > 0.5, f"Collision or insufficient safety margin! Remaining distance: {remaining_dist:.2f}m"

# --- 4. SOTIF ANALYSIS (ISO 21448) ---

@pytest.mark.xfail(reason="SOTIF Limit (ISO 21448): At 108 km/h, braking physics requires >45m, exceeding TTC=1.5s detection window.")
def test_sil_highway_sotif_limit():
    """SIL-02 (SOTIF): Highway scenario (108 km/h, obstacle at 60m) exposing ODD speed limitation."""
    remaining_dist, final_speed, _ = run_sil_simulation(
        initial_speed_kmh=108.0,
        initial_distance_m=60.0
    )
    assert remaining_dist > 0.5, f"SOTIF limitation: vehicle failed to stop before obstacle (dist={remaining_dist:.2f}m)"
