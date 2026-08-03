Scratch-Built ESP32 Quadcopter with Vision-Based Position Hold
Custom 5" quadcopter: flight controller firmware, radio protocol, and control loops built from scratch on ESP32 — now with optical-flow drift correction and lidar altitude hold via a Raspberry Pi companion computer.

Hardware

* FC: ESP32 (Arduino/C++), MPU6050 IMU, DShot300 → HGLRC Zeus 60A 4-in-1
* Motors: RS2205 2300KV on 4S
* Radio: custom bidirectional ESP-NOW link (50Hz control up, 10Hz telemetry down)
* Companion: Raspberry Pi 3B+ + OV5647 camera (optical flow @ 30fps)
* Rangefinder: VL53L1X ToF on the Pi's I²C bus, down-facing (~3m, medium mode)
* Glove (experimental): ESP32 + BNO085 9-DoF, pinch contacts + hall sensors
* Pi ↔ FC: 115200 UART, checksummed 12-byte packet, 200ms failsafe

Architecture
Cascaded angle-mode PID (angle → rate), complementary-filter attitude estimation, anti-windup integral design (activation latch, conditional integration, zero-crossing decay), offset/trim separation for attitude reference vs. steady torque compensation. Heading hold closes a P loop on integrated gyro Z; stick input slews the reference rather than fighting it. Altitude hold is P+I on lidar height driving throttleHold — engages on released stick, freezes rather than resets on data loss.

Glove control mapping
Tilt fwd/back = pitch, left/right = roll (8° deadzone, full at 40°). Ring held remaps left/right tilt to yaw. Index held ramps throttle up and freezes on release. Middle held lands. Pinky or fist kills. Arm stays on the TX switch, deliberately not on the glove.

Wire protocols
Four formats, all silent-failure on mismatch: TX→FC Packet (9B), FC→TX Telem (41B), Pi→FC framed+XOR (12B), Glove→TX GlovePacket (11B). Both FC and TX print sizeof(Telem) at boot and must agree.

Status

- [x] Stable manual hover, tuned trim, converged roll axis, heading hold
- [x] Live wireless telemetry for flight-log tuning
- [x] M2: optical flow pipeline — 30fps, ±0.02px noise floor
- [x] M3: flow → FC UART link
- [x] M4: velocity damping — vision drift-kill (gain-limited by loop delay)
- [x] ToF altitude hold + altitude-aware landing (landing path untested in flight)
- [ ] Glove transmitter
- [ ] Flow scaling by height
- [ ] Rotation compensation on flow
- [ ] Forward-facing ToF for wall detection

Known limits

* Vibration (vib ≈ 0.9–1.2 at hover) caps D-gain at zero and blurs flow frames
* Flow uncompensated for rotation; spikes ±20 px/frame during altitude change
* Flow unscaled by height, so effective gain falls as altitude rises
* Lidar is indoor-biased — sunlight cuts usable range substantially
