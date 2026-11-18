import pytest
from src.aeb_controller import AEBController, AEBState


@pytest.fixture
def aeb():
    return AEBController()


def test_req01_sensor_fault_injection(aeb):
    """Weryfikacja reakcji na awarię sensora (Fault Injection)."""
    state = aeb.update(ego_speed=20.0, distance=10.0, sensor_valid=False)
    assert state == AEBState.FAULT_DEGRADED


def test_req02_standstill_zero_speed(aeb):
    """Weryfikacja braku błędu dzielenia przez zero przy postoju."""
    state = aeb.update(ego_speed=0.0, distance=5.0, sensor_valid=True)
    assert state == AEBState.NORMAL


@pytest.mark.parametrize("speed, distance, expected_state", [
    (10.0, 25.1, AEBState.NORMAL),             # TTC = 2.51s -> NORMAL
    (10.0, 25.0, AEBState.WARNING),            # TTC = 2.50s -> WARNING
    (10.0, 15.1, AEBState.WARNING),            # TTC = 1.51s -> WARNING
    (10.0, 15.0, AEBState.EMERGENCY_BRAKING),  # TTC = 1.50s -> BRAKING
])
def test_req03_to_req05_boundary_values(aeb, speed, distance, expected_state):
    """Testy wartości brzegowych (Boundary Value Analysis) dla progów TTC."""
    state = aeb.update(ego_speed=speed, distance=distance)
    assert state == expected_state


def test_sil_dynamic_emergency_braking_scenario(aeb):
    """Symulacja dynamiczna w pętli czasu (krok dt = 0.1s)."""
    dt = 0.1
    speed = 15.0            # m/s
    distance = 35.0         # m
    max_deceleration = 8.0  # m/s^2
    collision = False

    for _ in range(100):
        state = aeb.update(ego_speed=speed, distance=distance)

        if state == AEBState.EMERGENCY_BRAKING:
            speed = max(0.0, speed - max_deceleration * dt)

        distance -= speed * dt

        if distance <= 0:
            collision = True
            break

        if speed == 0:
            break

    assert not collision, f"Doszło do kolizji! Pozostały dystans: {distance:.2f}m"
    assert speed == 0.0, "Pojazd nie zatrzymał się całkowicie"
    assert distance > 0.5, f"Zatrzymanie zbyt blisko przeszkody: {distance:.2f}m"

@pytest.mark.xfail(reason="SOTIF limit: przy 108 km/h próg TTC=1.5s nie wystarcza do pełnego zatrzymania")
def test_sil_high_speed_collision_limit(aeb):
    """
    Test graniczny SOTIF (ISO 21448): Scenariusz autostradowy.
    Auto jedzie 30 m/s (108 km/h). Oczekujemy, że obecna konfiguracja progów
    nie pozwoli na wyhamowanie przed przeszkodą (znane ograniczenie systemu).
    """
    dt = 0.1
    speed = 30.0            # m/s (108 km/h - prędkość autostradowa)
    distance = 60.0         # m
    max_deceleration = 8.0  # m/s^2
    collision = False

    for _ in range(100):
        state = aeb.update(ego_speed=speed, distance=distance)

        if state == AEBState.EMERGENCY_BRAKING:
            speed = max(0.0, speed - max_deceleration * dt)

        distance -= speed * dt

        if distance <= 0:
            collision = True
            break

        if speed == 0:
            break

    assert not collision, f"Oczekiwana kolizja SOTIF przy wysokiej prędkości. Dystans: {distance:.2f}m"
