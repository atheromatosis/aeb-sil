![ADAS SIL Test Suite](https://github.com/atheromatosis/aeb-sil/actions/workflows/tests.yml/badge.svg)

# AEB System - Software-in-the-Loop (SIL) Test Framework

A Python-based Software-in-the-Loop (SIL) testing framework designed to verify the control logic and state machine of an **Autonomous Emergency Braking (AEB)** system.

## System Requirements (SUT)
* **REQ-01:** If the radar signal is invalid (`sensor_valid == False`) or input values are negative, the system shall transition to `FAULT_DEGRADED` state.
* **REQ-02:** If the vehicle is at a standstill (`ego_speed == 0 m/s`), the system shall remain in `NORMAL` state (preventing zero-division errors).
* **REQ-03:** If Time-To-Collision `TTC > 2.5s`, the system shall remain in `NORMAL` state.
* **REQ-04:** If `1.5s < TTC <= 2.5s`, the system shall transition to `WARNING` state.
* **REQ-05:** If `TTC <= 1.5s`, the system shall activate `EMERGENCY_BRAKING` state.

## Test Coverage & Methodology
* **Unit & State Machine Testing:** Verification of state transitions and robustness against invalid inputs (`Fault Injection`, zero-speed handling).
* **Boundary Value Analysis (BVA):** Precision testing around critical thresholds (`TTC = 2.5s` and `TTC = 1.5s`).
* **Dynamic SIL Simulation:** Closed-loop time simulation (`dt = 0.1s`) validating braking deceleration physics and collision avoidance KPIs (`remaining_distance > 0.5m`).
* **SOTIF Analysis (ISO 21448):** Dedicated `XFAIL` test case defining the Operational Design Domain (ODD) highway speed limit (`108 km/h`) under current TTC thresholds.

## Project Structure
```text
aeb-sil/
├── .github/workflows/
│   └── tests.yml           # CI/CD pipeline (GitHub Actions)
├── src/
│   └── aeb_controller.py   # System Under Test (AEB logic & state machine)
├── tests_aeb/
│   └── test_aeb.py         # Pytest test suite (Unit, BVA, SIL, SOTIF)
├── requirements.txt        # Project dependencies
└── README.md               # Documentation
```

## How to Run Tests & Generate HTML Report
HTML report generation (`report.html`) is configured by default in `pytest.ini`. To execute the suite:

```bash
pip install -r requirements.txt
python -m pytest

