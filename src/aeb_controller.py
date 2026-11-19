from enum import Enum


class AEBState(Enum):
    NORMAL = "NORMAL"
    WARNING = "WARNING"
    EMERGENCY_BRAKING = "EMERGENCY_BRAKING"
    FAULT_DEGRADED = "FAULT_DEGRADED"


class AEBController:
    def __init__(self, hysteresis_margin: float = 0.3):
        self.state = AEBState.NORMAL
        self.ttc_warning_threshold = 2.5
        self.ttc_braking_threshold = 1.5
        self.hysteresis_margin = hysteresis_margin

    def update(self, ego_speed: float, distance: float, sensor_valid: bool = True) -> AEBState:
        # REQ-01: Sensor health & physical sanity check
        if not sensor_valid or distance < 0 or ego_speed < 0:
            self.state = AEBState.FAULT_DEGRADED
            return self.state

        # REQ-02: Standstill safety (prevent ZeroDivisionError)
        if ego_speed == 0:
            self.state = AEBState.NORMAL
            return self.state

        ttc = distance / ego_speed

        # State machine transitions with hysteresis to prevent chattering
        if self.state == AEBState.EMERGENCY_BRAKING:
            if ttc > self.ttc_braking_threshold + self.hysteresis_margin:
                self.state = (
                    AEBState.WARNING
                    if ttc <= self.ttc_warning_threshold
                    else AEBState.NORMAL
                )
        elif self.state == AEBState.WARNING:
            if ttc <= self.ttc_braking_threshold:
                self.state = AEBState.EMERGENCY_BRAKING
            elif ttc > self.ttc_warning_threshold + self.hysteresis_margin:
                self.state = AEBState.NORMAL
        else:  # NORMAL or recovering from FAULT_DEGRADED
            if ttc <= self.ttc_braking_threshold:
                self.state = AEBState.EMERGENCY_BRAKING
            elif ttc <= self.ttc_warning_threshold:
                self.state = AEBState.WARNING
            else:
                self.state = AEBState.NORMAL

        return self.state

#test