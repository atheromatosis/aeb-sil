from enum import Enum


class AEBState(Enum):
    NORMAL = "NORMAL"
    WARNING = "WARNING"
    EMERGENCY_BRAKING = "EMERGENCY_BRAKING"
    FAULT_DEGRADED = "FAULT_DEGRADED"


class AEBController:
    def __init__(self):
        self.state = AEBState.NORMAL
        self.ttc_warning_threshold = 2.5
        self.ttc_braking_threshold = 1.5

    def update(self, ego_speed: float, distance: float, sensor_valid: bool = True) -> AEBState:
        if not sensor_valid or distance < 0 or ego_speed < 0:
            self.state = AEBState.FAULT_DEGRADED
            return self.state

        if ego_speed == 0:
            self.state = AEBState.NORMAL
            return self.state

        ttc = distance / ego_speed

        if ttc <= self.ttc_braking_threshold:
            self.state = AEBState.EMERGENCY_BRAKING
        elif ttc <= self.ttc_warning_threshold:
            self.state = AEBState.WARNING
        else:
            self.state = AEBState.NORMAL

        return self.state
