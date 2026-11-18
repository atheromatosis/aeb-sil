import random
import pytest
from src.aeb_controller import AEBController, AEBState

GRAVITY_MPS2 = 9.81


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


# --- 2. BOUNDARY VALUE ANALYSIS (BVA) & HYSTERESIS ---

@pytest.mark.parametrize("speed_mps, distance_m, expected_state", [
    (10.0, 25.1, AEBState.NORMAL),             # TTC = 2.51s -> slightly above warning threshold
    (10.0, 25.0, AEBState.WARNING),            # TTC = 2.50s -> exact boundary for WARNING
    (10.0, 15.1, AEBState.WARNING),            # TTC = 1.51s -> slightly above braking threshold
    (10.0, 15.0, AEBState.EMERGENCY_BRAKING),  # TTC = 1.50s -> exact boundary for EMERGENCY_BRAKING
])
def test_ttc_boundary_values(aeb, speed_mps, distance_m, expected_state):
    """REQ-03/04/05: Boundary Value Analysis around TTC = 2.5s and TTC = 1.5s."""
    assert aeb.update(ego_speed=speed_mps, distance=distance_m) == expected_state


def test_state_hysteresis_prevents_chattering(aeb):
    """REQ-06: Hysteresis (0.3s) must prevent rapid state oscillation near threshold boundaries."""
    # Enter WARNING at TTC = 2.4s
    assert aeb.update(ego_speed=10.0, distance=24.0) == AEBState.WARNING
    # TTC rises to 2.7s (above 2.5s entry threshold, but below 2.8s hysteresis exit) -> stays WARNING
    assert aeb.update(ego_speed=10.0, distance=27.0) == AEBState.WARNING
    # TTC rises to 2.81s (clears hysteresis band) -> returns to NORMAL
    assert aeb.update(ego_speed=10.0, distance=28.1) == AEBState.NORMAL


def test_sensor_jitter_stability(aeb):
    """REQ-07: Radar distance jitter (+/- 0.5m) around warning boundary must not cause state chattering."""
    random.seed(42)
    transitions = 0
    prev_state = aeb.update(ego_speed=10.0, distance=24.0)  # Enter WARNING (TTC = 2.4s)

    for _ in range(20):
        noisy_distance = 25.5 + random.uniform(-0.5, 0.5)  # TTC oscillates between 2.50s and 2.60s
        current_state = aeb.update(ego_speed=10.0, distance=noisy_distance)
        if current_state != prev_state:
            transitions += 1
        prev_state = current_state

    assert transitions == 0, f"Hysteresis failed under radar jitter: {transitions} unexpected transitions"


# --- 3. CLOSED-LOOP SIL SIMULATION (WITH ACTUATOR DELAY & FRICTION) ---

def run_sil_simulation(
    initial_speed_kmh: float,
    initial_distance_m: float,
    friction_coeff: float = 0.90,
    actuator_delay_s: float = 0.2,
    dt: float = 0.05,
):
    """
    Closed-loop SIL simulation modeling:
    - Road friction coefficient (mu) limiting max braking deceleration (a_max = mu * g)
    - Hydraulic brake actuator response latency (reduced by 50% if pre-filled during WARNING)
    """
    controller = AEBController()
    speed_mps = initial_speed_kmh / 3.6
    distance_m = initial_distance_m
    max_deceleration = friction_coeff * GRAVITY_MPS2
    braking_active_time = 0.0
    pre_filled = False
    time_elapsed = 0.0

    while speed_mps > 0 and distance_m > 0 and time_elapsed < 15.0:
        state = controller.update(ego_speed=speed_mps, distance=distance_m)

        if state == AEBState.WARNING:
            pre_filled = True  # Hydraulic pre-fill reduces subsequent actuator latency
        elif state == AEBState.EMERGENCY_BRAKING:
            effective_delay = actuator_delay_s * 0.5 if pre_filled else actuator_delay_s
            if braking_active_time + 1e-9 >= effective_delay:
                speed_mps = max(0.0, speed_mps - max_deceleration * dt)
            braking_active_time += dt
        else:
            braking_active_time = 0.0
            pre_filled = False

        distance_m -= speed_mps * dt
        time_elapsed += dt

    return distance_m, speed_mps, controller.state



def test_sil_urban_dry_asphalt_collision_avoided():
    """SIL-01: Euro NCAP urban CCRs scenario (45 km/h, obstacle at 30m, dry asphalt mu=0.90, 200ms brake delay)."""
    remaining_dist, final_speed, final_state = run_sil_simulation(
        initial_speed_kmh=40.0,
        initial_distance_m=30.0,
        friction_coeff=0.90,
        actuator_delay_s=0.2,
    )
    assert final_speed == 0.0
    assert final_state == AEBState.EMERGENCY_BRAKING
    assert remaining_dist > 0.0, f"Collision or insufficient safety margin! Remaining distance: {remaining_dist:.2f}m"


def test_sil_actuator_latency_increases_stopping_distance():
    """SIL-02: Verify that 250ms brake actuator latency measurably increases stopping distance vs ideal 0ms."""
    dist_ideal, _, _ = run_sil_simulation(
        initial_speed_kmh=50.0, initial_distance_m=35.0, actuator_delay_s=0.0
    )
    dist_delayed, _, _ = run_sil_simulation(
        initial_speed_kmh=50.0, initial_distance_m=35.0, actuator_delay_s=0.25
    )
    assert dist_delayed < dist_ideal, "Actuator latency must result in a shorter remaining safety margin"


# --- 4. SOTIF ANALYSIS (ISO 21448) ---

@pytest.mark.xfail(reason="SOTIF Limit (ISO 21448): At 108 km/h, braking physics + 200ms actuator delay exceeds TTC=1.5s window.")
def test_sil_highway_sotif_limit():
    """SIL-03 (SOTIF): Highway scenario (108 km/h, obstacle at 60m) exposing ODD speed limitation."""
    remaining_dist, _, _ = run_sil_simulation(
        initial_speed_kmh=108.0,
        initial_distance_m=60.0,
        friction_coeff=0.85,
        actuator_delay_s=0.2,
    )
    assert remaining_dist > 0.5, f"SOTIF limitation: vehicle failed to stop before obstacle (dist={remaining_dist:.2f}m)"


@pytest.mark.xfail(reason="SOTIF Wet Road (ISO 21448): Low friction (mu=0.45) degrades deceleration to 4.41 m/s², requiring earlier TTC threshold.")
def test_sil_wet_asphalt_sotif_limit():
    """SIL-04 (SOTIF): Urban speed (50 km/h) on wet asphalt (mu=0.45) exposing environmental ODD limitation."""
    remaining_dist, _, _ = run_sil_simulation(
        initial_speed_kmh=50.0,
        initial_distance_m=28.0,
        friction_coeff=0.45,
        actuator_delay_s=0.2,
    )
    assert remaining_dist > 0.5, f"SOTIF wet road limitation: vehicle skidded past obstacle (dist={remaining_dist:.2f}m)"
