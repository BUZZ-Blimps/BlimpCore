# Catching Blimp — Source File Review

## What Each File Does (and how it contributes)

**catching_blimp_node.cpp** — ROS 2 entry point: `main()` inits ROS, creates and spins the `CatchingBlimp` node, then shuts down. No logic; just startup/shutdown.

**CatchingBlimp.cpp** — Core node: loads PID and accel calibration from params; inits hardware (IMU, lidar, motors, grabber); creates publishers/subscribers and timers; runs 100 Hz IMU loop (Madgwick, TF, yaw/roll/z PIDs, motor mixing), 50 Hz lidar (height with tilt correction), 2 Hz heartbeat, ~30 Hz state machine tick; handles basestation (mode, catch, shoot, kill, manual commands) and vision (targets, avoidance). Drives the blimp’s sensing, control, and actuation.

**CatchingBlimpStateMachine.cpp** — High-level behavior: manual vs autonomous; in auto, sequences search → approach → catch → caught → goal search → approach goal → scoring/shoot → scored. Maps basestation triggers (grab, shoot, goal color) and vision targets to forward/up/yaw/roll commands and grabber/shooter actions. Drives *what* the blimp does (search, catch, score).

**BangBang.cpp** — On/off controller: output is ±centeringCom or 0 from setpoint vs actual with a deadband. Used for goal height hold (e.g. centering at the goal).

**OPI_IMU.cpp** — Orange Pi IMU/baro driver over I2C: configures and reads LSM6DSL (accel, gyro), LIS3MDL (mag), BM388 (pressure, temp); converts to physical units and FLU frame; optional Z rotation; baro compensation. Feeds attitude and (optionally) altitude.

**Brushless.cpp** — Single brushless motor/ESC: PWM setup and thrust command in 1000–2000 range. Low-level thrust output.

**MotorControl_V2.cpp** — Motor mixing: maps body commands (forward, up, yaw rate, roll rate) to four Brushless ESCs with deadband and min/max; inits all four at 1500. Turns high-level commands into per-motor thrust.

**EMAFilter.cpp** — Exponential moving average: `y = alpha*current + (1-alpha)*last`. Used to smooth gyro rates, target position, height, etc.

**ZEstimator.cpp** — Altitude (z) Kalman filter: state [z, v, ?]; propagate from accel/attitude; partial or full updates from baro/lidar. Fuses sensors for vertical position/velocity (currently lidar is primary; baro path is optional).

**Madgwick_Filter.cpp** — Attitude from gyro + accel (and optionally mag): fuses angular rate and acceleration into a quaternion and Euler angles. Provides orientation for TF, tilt correction, and roll/yaw control.

**optical_ekf.cpp** — Optical-flow EKF (7-state): predicts/updates from flow and model params. Available for translational velocity or position estimation from optical flow (if used elsewhere).

**PID.cpp** — Generic PID with optional output limits and I-term clamping. Used for x, y, z, yaw rate, roll, roll rate.

**Servo.cpp** — Single servo: PWM setup, angle (0–180°) mapped to pulse width. Used by the grabber gate.

**tripleBallGrabber.cpp** — Ball grabber + shooter: servo (gate open/close) and brushless (shooter motor); open/close/shoot with rate limiting and state. Executes catch and score actions.

**math_helpers.cpp** — `constrain(x, lo, hi)` and `map(x, in_lo, in_hi, out_lo, out_hi)`. Used for clamping and scaling (e.g. servo, motor).

**TOF_Sense.cpp** — Time-of-flight lidar over UART: frame decode, distance and signal strength. Primary height measurement; feed into height filter and z_hat_.

**Gimbal.cpp, Kalman_Filter_Tran_Vel_Est.cpp, Optical_Flow.cpp** — Present in `src/` but not linked in `CMakeLists.txt` for the main node. Gimbal/Kalman/optical flow could support gimbal or velocity estimation in other targets or future use.

---

## Comments and Changes

**CatchingBlimp.cpp** — Added a short file header describing the node; section comments in the constructor (params, TF, hardware, actuators, publishers, QoS/subscribers, timers, timestamps); one-line comments above each timer callback and subscription callback describing rate and purpose; and inline notes for blimp name, tilt correction, heading layout, deadbands, startup zeroing, and motor mode branches. No logic was changed.

**EMAFilter.cpp** — Added a file header with the EMA formula and meaning of alpha; comments on both constructors (seeding behavior and default passthrough); brief comments on `reset()`, `setInitial()`, and `filter()` (including NaN pass-through); and tightened the inline NaN comment. No behavior changes.
