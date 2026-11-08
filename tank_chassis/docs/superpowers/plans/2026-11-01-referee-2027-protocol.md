# Referee 2027 Protocol Repair Implementation Plan

**Goal:** Align the existing referee receive/UI/map interfaces with the supplied RoboMaster 2027 V1.0.0 protocol.

**Architecture:** Keep the byte-stream parser, existing public data objects and power fallback. Validate CRC and exact command payload sizes before publication. Treat referee control data as available only after both 0x0201 and 0x0202 are received and younger than the existing 1000 ms timeout. UI waits only for TX, yields while waiting and times out without overwriting an active DMA buffer; failed initialization remains retryable.

**Tech Stack:** STM32H723 HAL/CMSIS-RTOS C++, Python/g++ host tests, Keil ARMClang.

- [x] Add `tests/referee_protocol_test.py`, compiling copies of the actual referee/UI/CRC sources with hardware stubs. Reproduce 0303 layout, wrong-length publication, startup/noise/stale online state, RX-busy TX blocking, DMA failure/retry and 0305 length failures.
- [x] Update `user/core/APP/Referee/RM_RefereeSystem.h/.cpp`: preserve 0201=17, 0202=14, graphic=15; correct 0303=12 and 0305=48; enforce sizes before copy; update independent freshness only on validated 0201/0202; keep stale values with offline flag for existing fallback. Use 118-byte interaction payload capacity. Correct map TX length and radar sender guard.
- [x] Update existing UI send functions and `RMRefereeSystemUI.cpp` to check TX state only, bound waits to 20 ms with `osDelay(1)`, propagate failure, and mark UI initialized only after successful transmissions. Reject invalid figure counts and overlong string metadata before writing buffers; clamp the existing legacy string builder to its 30-byte payload.
- [x] Verify fragmented/concatenated frames, CRC failure recovery, exact size rejection, time rollover and recovery, UI wire lengths/CRCs/numeric packing, RX-busy and TX-busy handling, initialization retry and radar-only 57-byte map frames.
- [x] Run relevant power/link regressions and Keil build; inspect link/error log and fresh AXF/HEX. Review diff and preserve unrelated user edits. Do not commit or flash.

Protocol source: supplied `RoboMaster 2027 机甲大师高校系列赛通信协议 V1.0.0（20261029）.pdf`, tables 1-12, 1-13, 1-25 through 1-32, 1-35, 1-36 and appendix II. No additional telemetry, control policy or unused command support is introduced.

Validation: `python tests/referee_protocol_test.py` passes all 11 case groups, including the actual power strategy retaining its last valid limit after data expiry. `python tests/rs485_board_link_test.py` and `python tests/rs485_error_recovery_test.py` pass. Keil `MDK-ARM/referee_2027_build.log` records linking and HEX generation with 0 errors and 5 existing `RemoteData_t` warnings (UV4 exit 1 denotes the warning result). Fresh AXF/HEX generated. The chassis control source and three Keil configuration SHA256 hashes match the pre-build snapshot. No commit, flash or hardware validation performed.

Independent read-only review found no concrete defects in the scoped changes and independently reran all 11 host case groups successfully.
