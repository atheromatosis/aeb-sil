from enum import Enum


class AEBState(Enum):
    NORMAL = "NORMAL"
    WARNING = "WARNING"
    EMERGENCY_BRAKING = "EMERGENCY_BRAKING"
    FAULT_DEGRADED = "FAULT_DEGRADED"


class AEBController:
    def __init__(self):
        self.state = AEBState.NORMAL
        self.ttc_warning_threshold = 2.5  # sekundy
        self.ttc_braking_threshold = 1.5  # sekundy

    def update(self, ego_speed: float, distance: float, sensor_valid: bool = True) -> AEBState:
        # REQ-01: Weryfikacja jakości sygnału z czujnika
        if not sensor_valid or distance < 0 or ego_speed < 0:
            self.state = AEBState.FAULT_DEGRADED
            return self.state

        # REQ-02: Zabezpieczenie przed dzieleniem przez zero, gdy auto stoi
        if ego_speed == 0:
            self.state = AEBState.NORMAL
            return self.state

        # Obliczenie Time-To-Collision (TTC)
        ttc = distance / ego_speed

        # REQ-03, REQ-04, REQ-05: Maszyna stanów
        if ttc <= self.ttc_braking_threshold:
            self.state = AEBState.EMERGENCY_BRAKING
        elif ttc <= self.ttc_warning_threshold:
            self.state = AEBState.WARNING
        else:
            self.state = AEBState.NORMAL

        return self.state
