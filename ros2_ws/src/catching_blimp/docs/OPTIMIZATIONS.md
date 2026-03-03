# Catching Blimp — Code Optimization Audit (optimizations branch)

**Scope:** ROS2 workspace, `catching_blimp` package. Focus: Finite State Machine (FSM) subsystem; issues flagged across the codebase where relevant.

**Audit date:** 2025-02-16. No files were modified during Phase 1–2; implementation (Phase 4) is pending approval of Phase 3.

---

## PHASE 1: RECONNAISSANCE

### 1.1 Directory structure (catching_blimp package)

```
ros2_ws/src/catching_blimp/
├── CMakeLists.txt
├── package.xml
├── param/pid_config.yaml
├── calibration/*.yaml
├── launch/
│   ├── catchingblimp.launch.py
│   └── [1-6]_*.launch.py (per-blimp)
├── scripts/battery_monitor_node.py
├── docs/
│   ├── DOCUMENTATION.txt
│   ├── CODE_GRAVEYARD.txt
│   ├── SRC_REVIEW.md
│   └── OPTIMIZATION_AUDIT.md (this file)
├── include/
│   ├── CatchingBlimp.hpp          # FSM enums, state vars, class declaration
│   ├── PID.hpp, EMAFilter.hpp, BangBang.hpp
│   ├── OPI_IMU.hpp, Madgwick_Filter.hpp, TOF_Sense.hpp
│   ├── MotorControl_V2.hpp, Brushless.hpp, Servo.hpp
│   ├── tripleBallGrabber.hpp, ZEstimator.hpp, math_helpers.hpp
│   └── (optical_ekf, Optical_Flow, Kalman_Filter_Tran_Vel_Est, etc.)
└── src/
    ├── catching_blimp_node.cpp    # main, node spin
    ├── CatchingBlimp.cpp          # node ctor, timers, callbacks, update_target, motor apply
    ├── CatchingBlimpStateMachine.cpp  # FSM implementation (state_machine_*)
    ├── OPI_IMU.cpp, Madgwick_Filter.cpp, TOF_Sense.cpp
    ├── MotorControl_V2.cpp, Brushless.cpp, Servo.cpp
    ├── tripleBallGrabber.cpp, PID.cpp, EMAFilter.cpp, BangBang.cpp
    ├── ZEstimator.cpp, math_helpers.cpp
    └── (optical_ekf, Optical_Flow, Kalman_Filter_Tran_Vel_Est, Gimbal, etc.)
```

- **FSM implementation:** `CatchingBlimpStateMachine.cpp` (state enums, transition logic, per-state callbacks); `CatchingBlimp.hpp` (enum `autoState`, `blimpState`, `approachState`; `auto_state_`, timers).
- **Sensor / HAL:** `OPI_IMU.cpp`, `TOF_Sense.cpp`, `ZEstimator.cpp`, `Madgwick_Filter.cpp`; vision input via ROS subscription `targets_subscription_callback` in `CatchingBlimp.cpp`.
- **Actuators:** `MotorControl_V2.cpp`, `Brushless.cpp`, `Servo.cpp`, `tripleBallGrabber.cpp`.
- **RTOS / scheduler:** None. ROS2 timers and subscriptions; single-threaded executor (timers and callbacks run on same thread).
- **Build:** `CMakeLists.txt` (ament_cmake), `package.xml`. No Makefile or platformio.ini.
- **Tests/benchmarks:** No dedicated test or benchmark files under `catching_blimp`.

### 1.2 Target platform

- **MCU/SBC:** Orange Pi 5 (OPI 5) — 64-bit ARM (RK3588), runs Linux. References: `OPI_IMU.hpp`, `wiringPi`, pin defines in `CatchingBlimp.hpp` (e.g. `PIN_LEFT_UP`, `GATE_S`).
- **FPU/RAM/Flash:** SoC has FPU; system has ample RAM and flash. Not a deeply embedded MCU.
- **RTOS:** No RTOS. ROS2 Humble; node uses `rclcpp::create_wall_timer` (33 ms state machine, 10 ms IMU, 20 ms lidar, 500 ms heartbeat).
- **Compiler / flags:** CMake adds `-Wall -Wextra -Wpedantic` only. No explicit `-O2`/`-O3` in package; ament/colcon may add release flags at workspace level.

### 1.3 FSM architectural summary (plain English)

- **States:** The autonomous FSM has **10 nominal states** plus **no_state** (landing/standby):  
  `searching` → `approach` → `catching` → `caught` → (then either back to `searching` or) `goalSearch` → `approachGoal` → `scoringStart` → `shooting` → `scored` → back to `searching`.  
  So: **11 enum values** (`searching`, `approach`, `catching`, `caught`, `goalSearch`, `approachGoal`, `scoringStart`, `shooting`, `scored`, `no_state`).

- **Transition triggers:**  
  - **Polling (timer-driven):** The FSM runs every **33 ms** via `timer_state_machine`. Each tick: `state_machine_callback()` runs, which calls `update_target()` then branches on `control_mode_` (manual / autonomous / lost). In autonomous, a **switch on `auto_state_`** dispatches to one of 10 state handlers.  
  - Transitions are **condition-based** inside each handler: time elapsed (e.g. `TIME_TO_CATCH`, `MAX_APPROACH_TIME`), vision flags (`target_active_`, `target_.type`, `target_.bbox_area`), and counters (`catches_`, `TOTAL_ATTEMPTS`). There is **no separate event queue**; all conditions are re-evaluated every tick.

- **Sensor data into FSM:**  
  - **Vision:** Offboard vision publishes to `targets` topic → `targets_subscription_callback` updates `target_`, `target_history_`, `target_detected_`.  
  - **State machine tick:** At the start of each tick, `update_target()` runs (timeouts, `target_active_`, optional prediction). The FSM then reads `target_active_`, `target_.type`, `target_.bbox_area`, `z_hat_`, etc. So sensor data is **shared memory** (member variables) written by ROS callbacks and read by the timer callback; **no mutex** (single-threaded executor).

- **Topology:** The FSM is **flat**: one `auto_state_` and one switch. The `approachState` enum (far_approach, alignment, near_approach) and `approach_state_` exist in the header but are **never used** in transitions (sub-state logic was removed and is only in CODE_GRAVEYARD). So there is **no hierarchical state machine** in the current code.

---

## PHASE 2: STATIC ANALYSIS — ISSUES BY CATEGORY

### FSM-specific

| File | Line(s) | Type | Severity | Description |
|------|---------|------|----------|-------------|
| CatchingBlimpStateMachine.cpp | 37–56 | control_mode_ branch | Medium | Linear if/else-if for manual/autonomous/lost; could be switch for consistency and possible table-driven dispatch later. |
| CatchingBlimpStateMachine.cpp | 169–206, 206–277, etc. | Re-evaluate every tick | Medium | All transitions (e.g. ball detected, timeouts) re-evaluated every 33 ms; no event-driven transitions (acceptable for 30 Hz but worth noting). |
| CatchingBlimpStateMachine.cpp | 24 | update_target() every tick | Low | update_target() called every FSM tick; contains time checks and optional prediction — acceptable but couples FSM tick to vision/target logic. |
| CatchingBlimp.hpp / StateMachine | 264–267, 447 | approachState / approach_state_ | Medium | approachState enum and approach_state_ declared and initialized but never written; dead state/sub-state. |
| CatchingBlimpStateMachine.cpp | 206–277, 365–406 | Duplicate approach logic | Medium | state_machine_approach_callback and state_machine_approachGoal_callback duplicate PID/scaling/bbox logic; could be shared helper. |
| CatchingBlimpStateMachine.cpp | 161–166 | default / no_state | Low | default case and state_machine_default_callback() present; no_state used for land() and battery low — catch-all exists. |
| CatchingBlimpStateMachine.cpp | 94–125, 280–308, etc. | Entry/exit mixed with logic | Low | State entry (e.g. approach_start_time_ = now) and ongoing behavior mixed in same callback; no explicit on_entry/on_exit. |
| CatchingBlimp.hpp | 239–250 | Flat FSM, 10+ states | Low | 10 autonomous states + no_state; flat structure; could later group (e.g. “ball phase” vs “goal phase”) for HFSM if complexity grows. |

### Memory

| File | Line(s) | Type | Severity | Description |
|------|---------|------|----------|-------------|
| CatchingBlimpStateMachine.cpp | 64–65 | std::string in FSM tick | Medium | On every state change, auto_state_to_string() builds and returns std::string; RCLCPP_INFO uses .c_str(); allocation in hot path when state changes. |
| CatchingBlimp.cpp | 556–574 | targets_subscription_callback | Low | target_history_.push_back() and pop_front() in callback; deque bounded by TARGET_HISTORY_SIZE (10); not in FSM tick but in vision callback. |
| CatchingBlimp.cpp | 402–444 | auto_state_to_string | Low | Multiple temporary std::strings per call; could use string_view or static lookup table to avoid allocations when logging. |
| CatchingBlimp.cpp | 62 | blimp_name_ | Low | std::string(this->get_namespace()).substr(1) in constructor; one-time, acceptable. |
| CatchingBlimp.hpp | 429 | avoidance | Low | std::vector<double> avoidance(9) with initializer; fixed size; could be std::array to avoid heap. |

### CPU / Performance

| File | Line(s) | Type | Severity | Description |
|------|---------|------|----------|-------------|
| CatchingBlimpStateMachine.cpp | 17–18 | rand() % 10 | Low | searchDirection() uses rand() % 10; rand() not seeded in node; and % 10 then < 5 gives biased binary; use C++11 RNG or seed. |
| PID.cpp | 96 | abs(i_out) | Medium | abs() on double; in C++ abs(int) may be used (truncation); should use std::abs(i_out) or fabs(i_out) for correct integral clamping. |
| CatchingBlimpStateMachine.cpp | 226, 376 | math_helpers::map/constrain | Low | Called every tick in approach/goal approach; small inline-friendly helpers; consider inline in header for hot path. |
| math_helpers.cpp | 24–26 | map() division | Low | map() has (in_max - in_min) in denominator; no check for zero; could be NaN if in_min == in_max (defensive). |
| CatchingBlimp.cpp | 647 | update_target | Low | predictTargetPosition() uses division by dt; dt can be zero (line 713 check); already guarded. |
| MotorControl_V2.cpp | 53, 60 | deadband_/2.0, 500 | Low | Division by constant 2 and non-power-of-two 500 in motorCom(); not critical on SBC with FPU. |

### I/O & Communication

| File | Line(s) | Type | Severity | Description |
|------|---------|------|----------|-------------|
| CatchingBlimpStateMachine.cpp | 64–65, 197, 249, 263, 269, 300, 358 | RCLCPP_INFO / RCLCPP_WARN in FSM | Medium | Logging on state change and transitions; can block on slow console/serial; consider throttling or async log. |
| CatchingBlimp.cpp | 106 | delay(2000) | High | delay(2000) in constructor blocks node startup for 2 s (ESC arm); blocks entire process; consider non-blocking or startup state. |
| CatchingBlimp.cpp | 319–338 | lidar in timer | Low | lidar.TOF_read() in lidar_timer_callback (50 Hz); UART read; no indication of DMA; acceptable for 50 Hz. |
| OPI_IMU.cpp, TOF_Sense.cpp, Brushless.cpp | (see grep) | printf/fprintf | Low | Error-path printf/fprintf in HAL; not in FSM hot path but in init/failure paths. |
| CatchingBlimp.cpp | 554–574 | targets callback | Low | Vision data written to target_/target_history_ from subscription; read in same thread by state_machine_callback; no double-buffer (not required with single-threaded executor). |

### Concurrency & Timing (ROS2, no RTOS)

| File | Line(s) | Type | Severity | Description |
|------|---------|------|----------|-------------|
| (N/A) | — | FSM vs sensor priority | Low | All callbacks/timers same thread; ordering defined by ROS2 executor; no explicit priority; acceptable. |
| (N/A) | — | Shared data | Low | target_, target_active_, etc. accessed from subscription and timer; single-threaded so no mutex; document assumption. |
| CatchingBlimp.cpp | 106 | delay(2000) | High | Blocking delay in ctor prevents other callbacks from running during startup. |

### Code Quality (indirect optimization)

| File | Line(s) | Type | Severity | Description |
|------|---------|------|----------|-------------|
| CatchingBlimp.cpp | 204–208, 228–232, 286–289, 340, 600–611, etc. | Commented-out blocks | Low | Several commented RCLCPP_INFO, z_est_, debug publish; dead code in production path; already documented in CODE_GRAVEYARD or could be removed. |
| CatchingBlimp.hpp, StateMachine, MotorControl | Many #defines | Magic numbers | Medium | Many numeric constants (900.0, 20000.0, 8000.0, 100000.0, 15.0, etc.) inline in code; some named (bbox_align_min) locally; recommend named constants for key thresholds. |
| CatchingBlimp.cpp | 401–445, 646–693, 766–824 | Long functions | Low | auto_state_to_string ~45 lines; update_target ~48; load_pid_config ~60; could be split for readability and inlining of hot helpers. |
| CatchingBlimp.hpp | 401–402 | Type inconsistency | Medium | blimpColor, goalColor declared as int but assigned enum values (red, orange, yellow); should be blimpType and goalType for type safety. |
| CatchingBlimpStateMachine.cpp | 438 | Typo in string | Low | auto_state_to_string(scored) returns "score" not "scored"; inconsistent with enum name. |
| CatchingBlimpStateMachine.cpp | 286–288 | Redundant ZERO_MODE check | Low | if (ballGrabber.is_fully_open() && !ZERO_MODE) { if (!ZERO_MODE) ballGrabber.suck(); } — inner ZERO_MODE redundant. |
| PID.cpp | 96 | abs vs std::abs | Medium | abs(i_out) for double i_out; use std::abs to avoid integer truncation. |

---

## PHASE 3: PRIORITIZED OPTIMIZATION PLAN

Optimizations ranked by severity then by estimated effort (low-effort first). **Do not implement until approved.**

| Priority | File | Line(s) | Issue | Proposed Fix | Expected Impact |
|----------|------|---------|-------|--------------|-----------------|
| HIGH | CatchingBlimp.cpp | 106 | Blocking delay(2000) in constructor | Replace with non-blocking startup: set a "arming" flag and in first N timer callbacks skip FSM/motors or use a one-shot timer to clear arming after 2 s | Removes 2 s block at startup; node remains responsive |
| HIGH | PID.cpp | 96 | abs(i_out) may truncate double | Use std::abs(i_out) or fabs(i_out) for integral clamping | Correct behavior for integral windup limits |
| MEDIUM | CatchingBlimpStateMachine.cpp | 64–65 | std::string allocation on every state change | Cache state string in static table (array of const char* or string_view keyed by enum); only call RCLCPP_INFO with cached string | Reduces allocations in FSM tick on transition |
| MEDIUM | CatchingBlimp.hpp / StateMachine | 264–267, 447 | Dead approach_state_ / approachState | Remove approach_state_ member and approachState enum from FSM usage, or document "reserved for future"; optionally keep enum for API but remove unused member to avoid confusion | Clearer FSM surface; no behavioral change |
| MEDIUM | CatchingBlimp.hpp | 401–402 | blimpColor, goalColor as int | Change type to blimpType and goalType; update goal_color_subscription_callback to use enum type | Type safety; avoids enum/int mismatch |
| MEDIUM | CatchingBlimpStateMachine.cpp | 37–56 | if/else-if control_mode_ | Replace with switch(control_mode_) for manual/autonomous/lost | Consistency with auto_state_ switch; minor readability |
| MEDIUM | CatchingBlimpStateMachine.cpp | 206–277, 365–406 | Duplicate approach logic | Extract shared helper e.g. run_approach(bbox_align_min, bbox_align_max, close_com, closure_com, gate_trigger, catch_trigger, is_ball) and call from both approach and approachGoal | Less duplication; easier tuning |
| LOW | CatchingBlimpStateMachine.cpp | 17–18 | rand() unseeded, biased binary | Seed srand once (e.g. from get_clock()->now()) or use std::uniform_int_distribution with std::default_random_engine | Reproducible and unbiased search direction |
| LOW | CatchingBlimpStateMachine.cpp | 438 | "score" vs "scored" | Return "scored" in auto_state_to_string for case scored | Consistent logging |
| LOW | CatchingBlimpStateMachine.cpp | 286–288 | Redundant ZERO_MODE check | Remove inner if (!ZERO_MODE); keep outer | Cleaner control flow |
| LOW | CatchingBlimpStateMachine.cpp | 197, 249, 263, 269, 300, 358 | RCLCPP_* in FSM | Add rate limit (e.g. log only once per transition or throttle to 1 Hz) or leave as-is and document | Reduces log spam and blocking if console is slow |
| LOW | math_helpers.cpp | 24–26 | map() division by zero | If (in_max == in_min) return out_min (or midpoint); else current formula | Avoids NaN on degenerate input |
| LOW | CatchingBlimp.hpp | 429 | avoidance vector | Change to std::array<double, 9> if API allows | Removes small heap allocation |
| LOW | CatchingBlimp.cpp | 402–444 | auto_state_to_string allocations | Implement via static const char* table[no_state+1] and return table[state] (or string_view) | Fewer allocations when logging state |

---

## DELIVERABLES SUMMARY

1. **Architectural summary (Phase 1):** Above — directory map, target platform (Orange Pi 5, ROS2 Humble, no RTOS), FSM states (11 enum values, 10 active + no_state), polling at 30 Hz, shared-memory sensor feed, flat FSM, unused approach_state_.
2. **Annotated issue list (Phase 2):** Above — grouped by FSM, Memory, CPU/Performance, I/O, Concurrency, Code Quality; each with file, line(s), type, severity, one-line description.
3. **Prioritized optimization plan (Phase 3):** Above — table with Priority, File, Line(s), Issue, Proposed Fix, Expected Impact; HIGH first, then MEDIUM, then LOW.

**Next step:** Review Phase 3 and approve which optimizations to implement. After approval, Phase 4 will apply only the approved items with minimal changes, OPT(category) comments, and updates to CODE_GRAVEYARD for any removed code.
