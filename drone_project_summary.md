# Project: DIY ESP32-S3 Super Mini Smartphone-Controlled Coreless Quadcopter

## Overview
A from-scratch lightweight quadcopter built for learning embedded systems, electronics, and flight control. No commercial flight controller — the ESP32-S3 handles sensor reading, PID stabilization, and motor output itself. Phone controls it over WiFi (ESP32 hosts its own network, no internet/router needed). Target all-up weight: ~60-65g.

## Hardware / BOM
- **Flight controller:** ESP32-S3 Super Mini (input voltage range confirmed 3.3V–6V per product page; has onboard battery charging via B+/B- pads, chargeable through its USB-C)
- **IMU:** MPU6050 (accelerometer + gyroscope), I2C
- **Battery:** 3.7V 600mAh 1S LiPo (602030), JST-PH 2.0 connector, connected to board's B+/B- pads
- **Motors:** 4x 8520 3.7V brushed coreless motors (2x CW, 2x CCW), 55mm props (0.9mm shaft hole)
- **Motor driver (×4, one per motor):** SI2302 N-channel logic-level MOSFET (SOT-23), ~220Ω gate resistor, 10kΩ pulldown resistor (gate to GND), flyback diode (1N5819 or SS14, cathode toward battery+), ~100nF ceramic capacitor across motor terminals
- **Frame:** Fusion 360 designed X-frame, prototyped from folded Coca-Cola aluminum can sheet (edges folded inward for rigidity; needs electrical tape/Kapton at any contact points with wiring since aluminum is conductive)
- Motor+ wires connect directly to the battery+ rail (NOT through the ESP32 board). MOSFET sources, ESP32 GND, battery-, and MPU6050 GND all share one common ground rail.
- A 470µF 16V electrolytic capacitor has been added across B+/B- to help smooth current spikes (added during current debugging, see Known Issue below).

## Pin Map (ESP32-S3 Super Mini)
| Function | GPIO |
|---|---|
| MPU6050 SDA | GPIO9 |
| MPU6050 SCL | GPIO8 |
| MPU6050 AD0 | GND (sets address 0x68) |
| Motor 1 (front-left) gate | GPIO11 |
| Motor 2 (front-right) gate | GPIO7 |
| Motor 3 (rear-right) gate | GPIO2 (moved off GPIO1 during earlier debugging) |
| Motor 4 (rear-left) gate | GPIO13 |
| Battery voltage monitor (reserved, not yet implemented) | GPIO4 |

## Control Scheme
- ESP32 runs as WiFi SoftAP ("Drone_ESP32" / "drone1234"), serves a control webpage at 192.168.4.1 over HTTP
- Page has two on-screen touch joysticks (Mode 2 layout: left = throttle+yaw, right = pitch+roll) plus an Arm/Disarm toggle button
- Control data sent from browser to ESP32 via WebSocket (`/ws`), ~20Hz, as CSV string: `armed,throttle,yaw,pitch,roll`
- Failsafe: if no WebSocket message received for 500ms, firmware forces disarm and cuts all motors immediately
- Libraries used: ESPAsyncWebServer + AsyncTCP (ESP32-S3 compatible forks, e.g. lacamera/ESP32Async — NOT the old unmaintained me-no-dev original)

## Firmware Status
Full firmware has been written (single .ino), includes:
- Raw I2C MPU6050 reading (no external IMU library) — accel + gyro registers read directly
- Complementary filter for pitch/roll (98% gyro / 2% accel), gyro Z rate used directly for yaw (no absolute heading — yaw will drift, expected/accepted since there's no magnetometer)
- PID structs for pitch, roll (angle mode) and yaw (rate mode) — gains are placeholder/untuned (kp/ki/kd values in code, need real tuning once flight-testing starts)
- X-frame motor mixing (front-left/front-right/rear-right/rear-left), with per-motor yaw sign defines (`YAW_M1`..`YAW_M4`) that can be flipped in code if yaw direction feels backwards, without rewiring
- Arm/disarm logic: motors stay at 0 until armed AND throttle > MIN_ARM_THROTTLE (15/255)
- **Throttle slew-rate limiting was added** (`THROTTLE_RAMP_RATE`, currently 3.0/loop at ~250Hz loop rate) so commanded throttle ramps smoothly even if the phone stick is moved instantly, intended to prevent current-inrush brownouts — **this fix did NOT resolve the current bug (see below)**

## Known Issue / Debugging Breakthrough
**Symptom:** WiFi AP connects reliably over phone. When armed, all 4 motors spin for a moment, then WiFi disconnects and the ESP32 resets.

**Root Causes Identified & Solved:**
1. **Critical Web UI Joystick Bug (Auto-center 50% Throttle):**
   - The web touch joystick calculated throttle with `throttle = Math.round(constrainVal((-dy + 1) * 127.5, 0, 255));`.
   - At center (`dy = 0`), throttle is **128 (50% power)**.
   - When the user released the touch stick, `reset()` moved the knob to center (`dy = 0`), snapping throttle to 128!
   - There was no zero-throttle arming interlock. Pressing "ARM" instantly commanded all 4 motors to 50% duty cycle, suddenly drawing 4–6 Amps from the 1S battery and collapsing the rail voltage.
   - **Fix Applied:** Re-engineered the left stick so throttle rests at the bottom (0%), snaps to 0% on release, requires throttle == 0 before arming is permitted, and displays live telemetry (`Throttle: 0%`).
2. **Aggressive Hardware Brownout Detector Reset:**
   - The ESP32-S3's hardware Brownout Detector resets the chip on microsecond voltage dips below ~2.8V.
   - **Fix Applied:** Disabled the hardware brownout reset in `setup()` via `WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);`.
3. **High ESP32-S3 Power Consumption (240 MHz & +20 dBm WiFi TX):**
   - High clock speed and maximum WiFi transmit power draw 400–500mA spikes during packet transfers.
   - **Fix Applied:** Reduced CPU frequency to 80 MHz (`setCpuFrequencyMhz(80)`) and lowered WiFi TX power to 8.5 dBm (`WiFi.setTxPower(WIFI_POWER_8_5dBm)`), saving ~60% MCU current and eliminating large RF bursts.
4. **PID Ground Vibration & Gyro Offsets:**
   - No gyro calibration meant stationary gyro offsets fed non-zero rates into PID.
   - Unattenuated D-term amplified motor vibrations on the frame, causing motors to kick violently.
   - **Fix Applied:** Added 200-sample stationary gyro zero-offset calibration in `setup()`, plus low-throttle PID ground attenuation.
5. **PWM Frequency & Switching Spikes:**
   - Reduced PWM from 20 kHz to 10 kHz to reduce switching losses on SI2302 MOSFETs and suppress capacitive spikes across motor terminals.

**Hardware Action Required:**
- **Reinstall the 470µF smoothing capacitor** across B+/B- (or Battery+ and GND). Running high-drain brushed coreless motors and an ESP32-S3 off a single 1S LiPo without a bulk capacitor guarantees inductive ripple and voltage dips. The reason the capacitor didn't work previously was because of the 50% throttle jump bug and the 240MHz/20dBm power spikes. With the code fixes applied, the capacitor can now do its job properly.
- If 100nF ceramic capacitors are installed directly across the motor terminals, consider removing or reducing them (to 1nF–10nF) if voltage sags continue at higher throttle, as 100nF acts as a momentary short circuit on fast PWM edges.

## Testing Status
- [x] Web controller throttle bug fixed (starts at 0%, rests at bottom, zero-throttle arm interlock added).
- [x] ESP32-S3 brownout detector disabled in firmware.
- [x] CPU frequency lowered to 80 MHz and WiFi TX power reduced to 8.5 dBm.
- [x] Gyro calibration at startup added.
- [x] Throttle slew rate smoothed and low-throttle ground PID attenuation added.
- [ ] User to re-solder 470µF capacitor to B+/B- and reflash firmware.
- [ ] Bench test props OFF starting from 0% throttle.
