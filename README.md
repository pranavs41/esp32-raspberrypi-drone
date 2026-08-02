# Scratch-Built ESP32 Quadcopter with Vision-Based Position Hold

Custom 5" quadcopter: flight controller firmware, radio protocol, and
control loops built from scratch on ESP32 — now gaining optical-flow
drift correction via a Raspberry Pi companion computer.

## Hardware
- FC: ESP32 (Arduino/C++), MPU6050 IMU, DShot300 → HGLRC Zeus 60A 4-in-1
- Motors: RS2205 2300KV on 4S
- Radio: custom bidirectional ESP-NOW link (50Hz control up, 10Hz telemetry down)
- Companion: Raspberry Pi 3B+ + OV5647 camera (optical flow @ 30fps)
- Pi ↔ FC: 115200 UART, checksummed 12-byte packet, 200ms failsafe

## Architecture
Cascaded angle-mode PID (angle → rate), complementary-filter attitude
estimation, anti-windup integral design (activation latch, conditional
integration, zero-crossing decay), offset/trim separation for attitude
reference vs. steady torque compensation.

## Status
- [x] Stable manual hover, tuned trim, heading hold
- [x] Live wireless telemetry for flight-log tuning
- [x] Optical flow pipeline (M2): 30fps, ±0.02px noise floor
- [ ] M3: flow → FC UART link *(in progress)*
- [ ] M4: velocity damping — vision drift-kill
- [ ] ToF integration: altitude scale + wall avoidance
