# DM4340 Radian Gimbal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the two-axis gimbal from LK4005 to J4340 MIT torque control on FDCAN3 while keeping IMU/vision semantics and converting the internal controller to radians.

**Architecture:** Keep the existing cascaded MCU controller and feed J4340 torque commands with all MIT position/velocity/gains set to zero. Make the DM driver bus-selectable, route both motor feedback frames through FDCAN3 FIFO0, and put degree-to-radian conversion at the IMU/vision control boundary. Keep the chassis yaw-offset protocol in degrees and read it from the Yaw motor encoder.

**Tech Stack:** STM32H723 HAL/FDCAN, FreeRTOS, C++17-compatible Keil ARM toolchain, existing `J4340`, PID, FSM, vision and IMU modules.

---

### Task 1: Add pure unit/conversion tests and isolate DM command encoding

**Files:**
- Create: `tests/dm4340_math_test.cpp`
- Modify: `user/core/BSP/Motor/DM/DmMotor.hpp`

- [ ] **Step 1: Write failing tests** for finite-value clamping, `0..2*pi` position encoding, and Yaw nearest-angle unwrapping using a host-testable helper API.
- [ ] **Step 2: Run the test executable and confirm the new helper symbols are missing.**
- [ ] **Step 3: Implement the minimal inline helpers and use them in MIT encoding.**
- [ ] **Step 4: Run the focused tests and confirm all pass.**
- [ ] **Step 5: Commit the driver math changes.**

### Task 2: Make the Damiao driver FDCAN-selectable

**Files:**
- Modify: `user/core/BSP/Motor/DM/DmMotor.hpp`
- Modify: `user/core/HAL/FDCAN/interface/fdcan_bus.hpp` only if the existing device accessor is insufficient

- [ ] **Step 1: Add a `FdcanDeviceId` constructor parameter with FDCAN2 as the backward-compatible default.**
- [ ] **Step 2: Store the selected device and route `ctrl_Mit`, `ctrl_AngleVelocity`, `ctrl_Velocity`, `On`, `Off`, and `ClearErr` through `get_device(selected_device)`.**
- [ ] **Step 3: Set J4340 parameters to the configured `0..2*pi` position range while retaining the existing J4340 velocity/torque/gain limits.**
- [ ] **Step 4: Build the project translation units that include `DmMotor.hpp` and verify no constructor call is broken.**
- [ ] **Step 5: Commit the bus-selectable driver.**

### Task 3: Replace the gimbal motor object and FDCAN routing

**Files:**
- Modify: `RtosTask/can_send_task.cpp`
- Modify: `RtosTask/can_send_task.hpp` if a typed encoder accessor is exported
- Modify: `Core/Src/fdcan.c` only if the existing FDCAN3 FIFO0 settings are insufficient

- [ ] **Step 1: Instantiate `BSP::Motor::DM::J4340<2>` with receive/send IDs `{1,2}` and `HAL_Fdcan3`.**
- [ ] **Step 2: Register FDCAN3 FIFO0 callback parsing feedback IDs 1 and 2; keep FDCAN1 for friction and FDCAN2 for referee/board frames.**
- [ ] **Step 3: Replace startup, zero-output, and normal output calls with DM MIT operations and `N*m` torque values.**
- [ ] **Step 4: Update online checks and state indexing so motor ID 1 is Yaw and motor ID 2 is Pitch.**
- [ ] **Step 5: Build and inspect the generated map/list for FDCAN3 send and receive references.**
- [ ] **Step 6: Commit the motor replacement and routing.**

### Task 4: Convert the gimbal control layer to radians and remove encoder fallback

**Files:**
- Modify: `RtosTask/can_send_task.cpp`
- Modify: `feeder_fsm/gimbal_fsm.cpp`
- Modify: `RtosTask/remote_control_task.cpp` only if vision conversion is best kept at reception

- [ ] **Step 1: Add named conversion constants and convert IMU angles/gyro rates at the control boundary.**
- [ ] **Step 2: Convert vision absolute degree targets to radians before `Struct_Gimbal_Input` is passed to the FSM.**
- [ ] **Step 3: Convert Yaw unwrap constants, Pitch limits, mouse/joystick scales, and FSM angle rules to radians.**
- [ ] **Step 4: Remove encoder-based IMU-fault control and make either-axis IMU fault command zero torque to both motors.**
- [ ] **Step 5: On IMU recovery, re-anchor both FSM targets to current IMU radians and reset controller state.**
- [ ] **Step 6: Build and review all changed expressions for degree/radian suffixes.**
- [ ] **Step 7: Commit the radian control and IMU safety behavior.**

### Task 5: Preserve degree-based chassis Yaw offset

**Files:**
- Modify: `communication_between_boards/boards_communication.cpp`
- Modify: `RtosTask/can_send_task.hpp` if an encoder-degree accessor is exported

- [ ] **Step 1: Read Yaw J4340 encoder angle for motor ID 1 using the driver's degree accessor.**
- [ ] **Step 2: Keep `YAW_FRONT_OFFSET_DEG`, `[-180,180]` normalization, and the existing FDCAN2 payload format.**
- [ ] **Step 3: Build and verify the board frame still contains a degree float.**
- [ ] **Step 4: Commit the chassis offset adaptation.**

### Task 6: Full verification and handoff

**Files:**
- Verify: `MDK-ARM/tank_gimbal.uvprojx`
- Verify: all files changed in Tasks 1–5

- [ ] **Step 1: Run focused host tests and record output.**
- [ ] **Step 2: Run the Keil/ARM project build using the repository's available build command.**
- [ ] **Step 3: Check that no old LK gimbal calls, hard-coded DM FDCAN2 sends, or encoder-feedback fallback remain.**
- [ ] **Step 4: Review the final diff for unrelated changes and preserve the pre-existing chassis modification.**
- [ ] **Step 5: Commit the complete migration if all checks pass.**
