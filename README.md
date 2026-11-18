# AEB System - Software-in-the-Loop (SIL) Test Framework

Projekt weryfikujący logikę systemu automatycznego hamowania awaryjnego (Autonomous Emergency Braking) w środowisku SIL (Software-in-the-Loop).

## Wymagania systemowe (Requirements)
* **REQ-01:** Jeśli sygnał z radaru jest nieprawidłowy (`sensor_valid == False`), system musi przejść w stan `FAULT_DEGRADED`.
* **REQ-02:** Jeśli prędkość pojazdu wynosi 0 m/s, system musi pozostać w stanie `NORMAL` (brak błędu dzielenia przez zero).
* **REQ-03:** Jeśli czas do kolizji `TTC > 2.5s`, system musi pozostać w stanie `NORMAL`.
* **REQ-04:** Jeśli `1.5s < TTC <= 2.5s`, system musi przejść w stan `WARNING`.
* **REQ-05:** Jeśli `TTC <= 1.5s`, system musi aktywować stan `EMERGENCY_BRAKING`.

## Pokrycie testowe (Test Coverage)
* **Testy jednostkowe (Unit):** Weryfikacja maszyny stanów i odporności na błędy (`Fault Injection`, dzielenie przez zero).
* **Analiza wartości brzegowych (BVA):** Testy progów `TTC = 2.5s` oraz `TTC = 1.5s`.
* **Symulacja dynamiczna (SIL):** Pętla czasowa `dt = 0.1s` weryfikująca drogę hamowania i brak kolizji.
* **Analiza SOTIF (ISO 21448):** Test `XFAIL` definiujący granicę prędkości autostradowej (108 km/h) dla zadanych progów TTC.

## Jak uruchomić testy i wygenerować raport
```bash
pip install -r requirements.txt
pytest -v --html=report.html --self-contained-html
