"""Host-check actual gantry task/driver routing, feedback snapshots and outputs."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
task = (root / "RtosTask/gantry_task.cpp").read_text(encoding="utf-8")
task = task.replace('#include "remote_control_task.hpp"', "")
# Only header include paths and fixture tuning values are substituted.
task = task.replace('../feeder_fsm/gantry_fsm.hpp', 'gantry_fsm.hpp')
header = (root / "feeder_fsm/gantry_fsm.hpp").read_text(encoding="utf-8")
task_header = (root / "RtosTask/gantry_task.hpp").read_text(encoding="utf-8")
task_header = task_header.replace('../feeder_fsm/gantry_fsm.hpp', 'gantry_fsm.hpp')
fsm_source = (root / "feeder_fsm/gantry_fsm.cpp").read_text(encoding="utf-8")
gimbal_task = (root / "RtosTask/gimbal_task.cpp").read_text(encoding="utf-8")
friction_definition = re.search(
    r"BSP::Motor::Dji::GM3508<4> friction_motor\([^;]+;", gimbal_task
).group()
valid = {
    "CALIBRATED": "true",
    "LIFT_ID5_FIRST_DEG": "100", "LIFT_ID6_FIRST_DEG": "-100",
    "LIFT_ID5_SECOND_DEG": "200", "LIFT_ID6_SECOND_DEG": "-200",
    "DM_EXTEND_DEG": "100",
    "LIFT_ID5_SPEED_RAD_S": "10", "LIFT_ID6_SPEED_RAD_S": "10", "DM_SPEED_RAD_S": "2",
    "LIFT_FIRST_TIMEOUT_MS": "1000", "EXTEND_TIMEOUT_MS": "1000",
    "LIFT_SECOND_TIMEOUT_MS": "1000", "RETRACT_TIMEOUT_MS": "1000", "LOWER_TIMEOUT_MS": "1000",
    "DM_MOVE_KD": "2", "DM_HOLD_KD": "4", "DM_TORQUE_LIMIT_NM": ".2f",
}
# Supply distinct move/hold and ID5/ID6 PID values at the production constructors.
pid_fixtures = {
    "lift_move_pid": "    {20, 1, 0, 500, 100, 0},\n    {30, 2, 0, 500, 100, 0}",
    "lift_hold_pid": "    {100, 1, 0, 500, 100, 0},\n    {200, 2, 0, 500, 100, 0}",
    "hold_angle_pid": (
        "    {1, .5f, 0, GantryTuning::LIFT_ID5_SPEED_RAD_S, 0, 0},\n"
        "    {2, .25f, 0, GantryTuning::LIFT_ID6_SPEED_RAD_S, 0, 0},\n"
        "    {.5f, .25f, 0, GantryTuning::DM_SPEED_RAD_S, 0, 0}"
    ),
}
for name, values in pid_fixtures.items():
    task, count = re.subn(
        r"(ALG::PID::PID " + name + r"\[\d+\]\s*=\s*\{).*?(\n\};)",
        lambda match: match[1] + "\n" + values + match[2], task, flags=re.S)
    assert count == 1, name
dispatcher = (root / "communication_between_boards/input_dispatcher.cpp").read_text(encoding="utf-8")
dispatcher = dispatcher.replace('#include "../user/core/BSP/RemoteControl/DT7.hpp"', "")
# Compile the actual FDCAN3 lambda together with the task and real drivers.
can_task = (root / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
start = can_task.index('[]', can_task.index('fdcan3.register_rx_callback('))
end = can_task.index('\n    });', start)
callback = can_task[start:end] + '\n    }'
code = r'''
#define BUZZER_HPP
#include "user/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "user/core/BSP/Motor/DM/DmMotor.hpp"
#include "stm32h7xx_hal.h"
#include <cassert>
#include <vector>
#include <functional>
#include "gantry_fsm.hpp"
struct TaskStopped {};
std::function<void()> after_cycle;
void TestTaskDelay(uint32_t period) {
    assert(period == 5);
    after_cycle();
}
uint32_t tick = 0;
GPIO_TypeDef gpio_a;
uint16_t pins = 0;
extern "C" uint32_t HAL_GetTick() { return tick; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*, uint16_t pin) {
    return (pins & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
namespace BSP::REMOTE_CONTROL {
struct RemoteController {
    enum { MIDDLE=3, KEY_R=1<<8, KEY_G=1<<10, KEY_V=1<<14 };
    bool online=true, right=false;
    uint8_t s1=3, s2=3;
    bool isConnected() { return online; }
    uint8_t get_s1() { return s1; }
    uint8_t get_s2() { return s2; }
    bool get_mouseRight() { return right; }
};
}
BSP::REMOTE_CONTROL::RemoteController remoteController;
DISPATCHER_SOURCE
TASK_SOURCE
const uint8_t chassis_motor_idxs[4] = {1, 2, 3, 4};
FRICTION_DEFINITION
BSP::Motor::DM::J4340<1> gimbal_motor(
    0, {8}, {8}, HAL::FDCAN::FdcanDeviceId::HAL_Fdcan3);
auto fdcan3_callback = CALLBACK_SOURCE;

namespace HAL::FDCAN {
struct Device : IFdcanDevice {
    Frame last{};
    int sends = 0;
    std::vector<Frame> frames;
    void init() override {}
    void start() override {}
    bool send(const Frame &frame) override { last = frame; ++sends; frames.push_back(frame); return true; }
    bool receive(Frame &) override { return false; }
    FDCAN_HandleTypeDef *get_handle() const override { return nullptr; }
    void register_rx_callback(RxCallback) override {}
    void trigger_rx_callbacks(const Frame &) override {}
};
struct Bus : IFdcanBus {
    Device devices[3];
    IFdcanDevice &get_device(FdcanDeviceId id) override {
        return devices[static_cast<unsigned>(id)];
    }
    bool has_device(FdcanDeviceId) const override { return true; }
} bus;
int bus_accesses = 0;
IFdcanBus &get_fdcan_bus_instance() { ++bus_accesses; return bus; }
}

int main() {
    using namespace HAL::FDCAN;
    // All four speed PID controllers must be independently configured before startup.
    for (unsigned i=0; i<2; ++i) {
        assert(lift_move_pid[i].UpDate(1, 0) == (i==0 ? 21 : 32));
        assert(lift_hold_pid[i].UpDate(1, 0) == (i==0 ? 101 : 202));
        assert(lift_move_pid[i].UpDate(1000, 0) == 500);
        assert(lift_hold_pid[i].UpDate(1000, 0) == 500);
        lift_move_pid[i].reset();
        lift_hold_pid[i].reset();
    }
    const float angle_speed_limits[3] = {GantryTuning::LIFT_ID5_SPEED_RAD_S,
        GantryTuning::LIFT_ID6_SPEED_RAD_S, GantryTuning::DM_SPEED_RAD_S};
    for (unsigned i=0; i<3; ++i) {
        // Outer limits must follow the configured speed limits without winding up.
        for (unsigned cycle=0; cycle<3; ++cycle)
            assert(hold_angle_pid[i].UpDate(100, 0)==angle_speed_limits[i]);
        assert(hold_angle_pid[i].UpDate(0, 0)==0);
        assert(hold_angle_pid[i].UpDate(-100, 0)==-angle_speed_limits[i]);
        assert(hold_angle_pid[i].UpDate(0, 0)==0);
        hold_angle_pid[i].reset();
    }
    assert(bus_accesses == 0);  // Object definitions must not start hardware or send.
    gantry_3508_motor.setCAN(0x1234, 1);  // Physical ID 5, first slot of 0x1FF.
    gantry_3508_motor.setCAN(-2, 2);      // Physical ID 6, second slot.
    gantry_3508_motor.sendCAN();
    assert(bus.devices[0].sends == 0 && bus.devices[1].sends == 0);
    const auto &frame = bus.devices[2].last;
    assert(frame.id == 0x1FF && frame.dlc == 8);
    assert(!frame.is_extended_id && !frame.is_remote_frame);
    assert(frame.data[0] == 0x12 && frame.data[1] == 0x34);
    assert(frame.data[2] == 0xFF && frame.data[3] == 0xFE);
    for (int i = 4; i < 8; ++i) assert(frame.data[i] == 0);

    Frame feedback{};
    feedback.dlc = 8;
    feedback.data[0] = 0x08;  // Encoder 2048 -> 90 degrees.
    feedback.id = 0x205;
    gantry_3508_motor.Parse(feedback);
    assert(gantry_3508_motor.getAngleDeg(1) == 90.0f);
    feedback.data[0] = 0x10;  // Encoder 4096 -> 180 degrees.
    feedback.id = 0x206;
    gantry_3508_motor.Parse(feedback);
    assert(gantry_3508_motor.getAngleDeg(2) == 180.0f);
    assert(gantry_3508_motor.getAngleDeg(1) == 90.0f);

    gantry_dm4310_motor.ctrl_Mit(1, 0, 0, 0, 0, 0);
    assert(bus.devices[2].sends == 2 && bus.devices[2].last.id == 9);
    const Frame zero_dm_command=bus.devices[2].last;
    feedback.id = 9;
    feedback.data[0] = 9;
    feedback.data[1] = 0xFF;
    feedback.data[2] = 0xFF;
    gantry_dm4310_motor.Parse(feedback);
    assert(gantry_dm4310_motor.getAngleRad(1) > 3.14f);

    const uint8_t existing_ids[4] = {1, 2, 3, 4};
    BSP::Motor::Dji::GM3508<4> existing(0x200, existing_ids, 0x200);
    existing.sendCAN();
    assert(bus.devices[0].sends == 1 && bus.devices[0].last.id == 0x200);
    assert(bus.devices[1].sends == 0 && bus.devices[2].sends == 2);

    // Use identical raw rotor feedback for gantry ID5/6 and friction ID1/2.
    // Only gantry accumulated angles and speeds must be divided by 19.
    assert(gantry_3508_motor.params_.reduction_ratio == 19.0);
    assert(friction_motor.params_.reduction_ratio == 1.0);
    for (int id = 1; id <= 2; ++id) {
        const int sign = id == 1 ? 1 : -1;
        for (int step = 0; step <= 80; ++step) {
            const int encoder = ((sign * step * 2048) % 8192 + 8192) % 8192;
            const int16_t rpm = sign * 1140; // Output: +/-60 rpm, +/-2*pi rad/s.
            Frame sample{};
            sample.dlc = 8;
            sample.data[0] = encoder >> 8;
            sample.data[1] = encoder & 0xFF;
            sample.data[2] = static_cast<uint16_t>(rpm) >> 8;
            sample.data[3] = static_cast<uint16_t>(rpm) & 0xFF;
            sample.id = 0x204 + id;
            gantry_3508_motor.Parse(sample);
            sample.id = 0x200 + id;
            friction_motor.Parse(sample);
            const double rotor_angle = sign * step * 90.0;
            assert(std::abs(gantry_3508_motor.getAddAngleDeg(id) - rotor_angle / 19) < 0.0001);
            assert(std::abs(gantry_3508_motor.getAddAngleRad(id) - rotor_angle / 19 * PI / 180) < 0.00001);
            assert(std::abs(friction_motor.getAddAngleDeg(id) - rotor_angle) < 0.0001);
            assert(gantry_3508_motor.getVelocityRpm(id) == sign * 60.0f);
            assert(std::abs(gantry_3508_motor.getVelocityRads(id) - sign * 2 * PI) < 0.00001);
            assert(friction_motor.getVelocityRpm(id) == rpm);
            assert(std::abs(friction_motor.getVelocityRads(id) - rpm * 2 * PI / 60) < 0.00001);
        }
        // Restore the starting encoder position and zero speed for later fixtures.
        Frame stopped{};
        stopped.dlc = 8;
        stopped.data[0] = id == 1 ? 0x08 : 0x10;
        stopped.id = 0x204 + id;
        gantry_3508_motor.Parse(stopped);
    }

    // The CAN task delays 1000 ms before registering these RX callbacks.
    tick=1000;
    gimbal_motor.On(1, BSP::Motor::DM::MIT);
    assert(bus.devices[2].last.id==8);
    gimbal_motor.ctrl_Mit(1, 0, 0, 0, 0, 1.0f);
    assert(bus.devices[2].last.id==8);
    for (const auto &sent : bus.devices[2].frames) assert(sent.id!=7);
    // Removed Yaw ID7 is ignored; physical Pitch ID8 maps to the only slot.
    Frame gimbal_feedback{};
    gimbal_feedback.dlc=8;
    gimbal_feedback.id=7; gimbal_feedback.data[0]=0x17;
    gimbal_feedback.data[1]=gimbal_feedback.data[2]=0xFF;
    fdcan3_callback(gimbal_feedback);
    assert(gimbal_motor.getAngleRad(1)==0);
    gimbal_feedback.id=8; gimbal_feedback.data[0]=0x18;
    gimbal_feedback.data[1]=gimbal_feedback.data[2]=0;
    fdcan3_callback(gimbal_feedback);
    assert(gimbal_motor.getAngleRad(1)==-3.14f);
    assert(gimbal_motor.getVelocityRads(1)==-10.0f);
    assert(gimbal_motor.getTorque(1)==-27.0f);
    // J4340 uses the user's rad/rad-s/Nm ranges for both TX and RX.
    gimbal_motor.ctrl_Mit(1,-3.14f,-10.0f,0,0,-27.0f);
    for (uint8_t byte : bus.devices[2].last.data) assert(byte==0);
    gimbal_motor.ctrl_Mit(1,3.14f,10.0f,500.0f,5.0f,27.0f);
    for (uint8_t byte : bus.devices[2].last.data) assert(byte==0xFF);
    for (int i=1; i<=5; ++i) gimbal_feedback.data[i]=0xFF;
    fdcan3_callback(gimbal_feedback);
    assert(std::abs(gimbal_motor.getAngleRad(1)-3.14f)<0.00001f);
    assert(gimbal_motor.getVelocityRads(1)==10.0f);
    assert(gimbal_motor.getTorque(1)==27.0f);

    auto &sent=bus.devices[2].frames;
    auto is_dm_service = [](const Frame &frame, uint8_t command) {
        if (frame.id!=9 || frame.data[7]!=command) return false;
        for (unsigned i=0; i<7; ++i) if (frame.data[i]!=0xFF) return false;
        return true;
    };
    auto count_enables = [&] {
        unsigned count=0;
        for (const auto &frame : sent) if (is_dm_service(frame, 0xFC)) ++count;
        return count;
    };
    auto last_control = [&](uint32_t id) -> const Frame& {
        for (auto it=sent.rbegin(); it!=sent.rend(); ++it)
            if (it->id==id && !is_dm_service(*it, 0xFC) && !is_dm_service(*it, 0xFD)) return *it;
        throw "Missing motor control frame";
    };
    auto run_once = [&] {
        after_cycle = [] { throw TaskStopped{}; };
        try { gantry_task(nullptr); } catch (const TaskStopped&) {}
        for (const auto &frame : sent) assert(!is_dm_service(frame, 0xFD));
        if (!gantry_fsm.IsActive()) {
            for (uint8_t byte : last_control(0x1FF).data) assert(byte==0);
            const auto &dm=last_control(9);
            for (unsigned i=0; i<8; ++i) assert(dm.data[i]==zero_dm_command.data[i]);
        }
    };
    run_once();
    assert(gantry_watch.configured == bool(TEST_VALID_CONFIG));
    for (bool online : gantry_watch.online) assert(!online);
    feedback.id=0x205; feedback.dlc=7; fdcan3_callback(feedback);
    run_once(); assert(!gantry_watch.online[0]);
    feedback.dlc=8; feedback.is_extended_id=true; fdcan3_callback(feedback);
    run_once(); assert(!gantry_watch.online[0]);
    feedback.is_extended_id=false; feedback.is_remote_frame=true; fdcan3_callback(feedback);
    run_once(); assert(!gantry_watch.online[0]);
    feedback.is_remote_frame=false;
    feedback.id=8; fdcan3_callback(feedback);
    run_once(); assert(!gantry_watch.online[2]);
    feedback.id=0x205; fdcan3_callback(feedback);
    feedback.id=0x206; fdcan3_callback(feedback);
    feedback.id=9; feedback.data[0]=0x18; fdcan3_callback(feedback);
    run_once(); assert(!gantry_watch.online[2]);
    feedback.data[0]=0x19; fdcan3_callback(feedback);
    run_once();
    for (bool online : gantry_watch.online) assert(online);
    assert(gantry_watch.dm_status == 1);
    tick+=99; run_once();
    for (bool online : gantry_watch.online) assert(online);
    tick+=1; run_once();
    for (bool online : gantry_watch.online) assert(!online);

    feedback.id=0x205; fdcan3_callback(feedback);
    run_once();
    assert(gantry_watch.online[0] && !gantry_watch.online[1] && !gantry_watch.online[2]);
    feedback.id=0x206; fdcan3_callback(feedback);
    feedback.id=9; fdcan3_callback(feedback);
    input_dispatcher.Update(3,3,0,false,false);
    pins=GPIO_PIN_3 | GPIO_PIN_4;
    run_once(); assert(!gantry_watch.limit[3]);
    tick+=20; run_once();
    assert(gantry_watch.limit[3] && gantry_watch.limit[4] && !gantry_watch.limit[0]);

    if (!TEST_VALID_CONFIG) {
        assert(gantry_watch.state == GantryState::Disabled);
        assert(count_enables()==0);
        return 0;
    }
    run_once(); assert(gantry_watch.configured && gantry_watch.state == GantryState::Idle);
    const unsigned enables_before_stop=count_enables();
    remoteController.online=false; run_once(); assert(gantry_watch.state == GantryState::Disabled);
    assert(count_enables()==enables_before_stop);
    remoteController.online=true;
    remoteController.s1=2; run_once(); assert(gantry_watch.state == GantryState::Disabled);
    assert(count_enables()==enables_before_stop);
    remoteController.s1=3;
    input_dispatcher.Update(2,2,0,false,false);
    run_once(); assert(gantry_watch.state == GantryState::Disabled);
    assert(count_enables()==enables_before_stop);
    input_dispatcher.Update(3,3,0,false,false);
    auto refresh_feedback = [&](bool dm_feedback, uint8_t status=1) {
        Frame sample{};
        sample.dlc=8; sample.data[0]=0x10;
        sample.id=0x205; fdcan3_callback(sample);
        sample.id=0x206; fdcan3_callback(sample);
        if (dm_feedback) {
            sample.id=9; sample.data[0]=(status<<4)|9;
            sample.data[1]=0x80; sample.data[3]=0x80;
            fdcan3_callback(sample);
        }
    };

    // Run one continuous task invocation so recovery timers persist between cycles.
    // Cover healthy startup, online-but-disabled retry, missing-feedback retry,
    // safe-mode rearming and a running dropout that must remain fault-latched.
    const unsigned enables_before=count_enables();
    const unsigned expected_enables[] = {1,1,2,2,3,4,4,4,5,6,6,7,8,8,8,8,9};
    unsigned recovery_cycles=0;
    refresh_feedback(true);
    after_cycle = [&] {
        const unsigned cycle=recovery_cycles++;
        assert(cycle<sizeof(expected_enables)/sizeof(expected_enables[0]));
        assert(count_enables()-enables_before==expected_enables[cycle]);
        assert(!gantry_fsm.IsActive());
        for (const auto &frame : sent) assert(!is_dm_service(frame, 0xFD));
        if (cycle==13 || cycle==14) {
            assert(gantry_watch.state==GantryState::Fault);
            assert(gantry_watch.fault==GantryFault::Feedback);
            for (auto &pid : hold_angle_pid) assert(pid.getOutput()==0);
            for (unsigned i=0; i<2; ++i) {
                assert(lift_move_pid[i].getOutput()==0);
                assert(lift_hold_pid[i].getOutput()==0);
            }
        }
        for (uint8_t byte : last_control(0x1FF).data) assert(byte==0);
        const auto &dm=last_control(9);
        for (unsigned i=0; i<8; ++i) assert(dm.data[i]==zero_dm_command.data[i]);
        switch (cycle) {
        case 0: tick+=5; refresh_feedback(true); break;
        case 1: tick+=5; refresh_feedback(true, 0); break;
        case 2: tick+=99; refresh_feedback(true, 0); break;
        case 3: tick+=1; refresh_feedback(true, 0); break;
        case 4: tick+=1; refresh_feedback(true); break;
        case 5: tick+=5; refresh_feedback(true); break;
        case 6: remoteController.online=false; tick+=1; refresh_feedback(true); break;
        case 7: remoteController.online=true; tick+=1; refresh_feedback(true); break;
        case 8: tick+=100; refresh_feedback(false); break;
        case 9: tick+=99; refresh_feedback(false); break;
        case 10: tick+=1; refresh_feedback(false); break;
        case 11: tick+=1; refresh_feedback(true); break;
        case 12:
            gantry_fsm.Update(true, true, true, true,
                              false, false, false, true, true,
                              gantry_3508_motor.getAddAngleDeg(1),
                              gantry_3508_motor.getAddAngleDeg(2),
                              gantry_dm4310_motor.getAddAngleDeg(1), tick);
            assert(gantry_fsm.GetState()==GantryState::LiftFirst);
            tick+=100; refresh_feedback(false);
            break;
        case 13: tick+=1; refresh_feedback(true); break;
        case 14: remoteController.online=false; tick+=1; refresh_feedback(true); break;
        case 15: remoteController.online=true; tick+=1; refresh_feedback(true); break;
        case 16:
            assert(gantry_watch.state==GantryState::Idle);
            throw TaskStopped{};
        }
    };
    try { gantry_task(nullptr); } catch (const TaskStopped&) {}
    assert(recovery_cycles==17);

    // Prime Extend with stationary lift targets 2 degrees either side of feedback.
    // Then run the complete task to check hold currents, MIT torque clamp and stop.
    feedback = {};
    feedback.dlc=8;
    feedback.data[0]=0x10;
    feedback.id=0x205; fdcan3_callback(feedback);
    feedback.id=0x206; fdcan3_callback(feedback);
    feedback.id=9; feedback.data[0]=0x19;
    feedback.data[1]=0x80; feedback.data[2]=0;
    feedback.data[3]=0x80; feedback.data[4]=0; // MIT zero speed, within quantization.
    fdcan3_callback(feedback);
    const double lift_id5_angle=gantry_3508_motor.getAddAngleDeg(1)+2;
    const double lift_id6_angle=gantry_3508_motor.getAddAngleDeg(2)-2;
    const double dm_angle=gantry_dm4310_motor.getAddAngleDeg(1);
    // DM holds during LiftFirst: small errors check radian control; large errors
    // check that the independent hold Kd is also used in the torque clamp.
    for (double error_deg : {2.0, -2.0, 20.0, -20.0}) {
        hold_angle_pid[2].reset();
        gantry_fsm=Class_Gantry_FSM{};
        for (bool right_pressed : {false, true})
            gantry_fsm.Update(true, true, right_pressed, right_pressed,
                              false, false, false, true, true,
                              lift_id5_angle, lift_id6_angle, dm_angle+error_deg, tick);
        for (unsigned cycle=0; cycle<(std::abs(error_deg)<10 ? 2U : 1U); ++cycle) {
            run_once();
            assert(gantry_watch.state==GantryState::LiftFirst);
            assert(gantry_watch.angle_deg[2]==dm_angle);
            assert(gantry_fsm.GetHoldAngle(2)==dm_angle+error_deg);
            const double outer_target=(cycle==0 ? .75 : 1.0)*error_deg*PI/180;
            assert(std::abs(hold_angle_pid[2].getError()-error_deg*PI/180)<0.000001);
            assert(std::abs(hold_angle_pid[2].getOutput()-outer_target)<0.000001);
            const auto &dm=last_control(9);
            assert(dm.id==9);
            const float sent_kd=(dm.data[5]<<4 | dm.data[6]>>4)*5.0f/4095;
            assert(std::abs(sent_kd-GantryTuning::DM_HOLD_KD)<5.0f/4095);
            const int encoded_vel=(dm.data[2]<<4)|(dm.data[3]>>4);
            const float sent_vel=encoded_vel*60.0f/4095-30;
            const double expected_vel=std::abs(error_deg)<10 ? outer_target :
                gantry_dm4310_motor.getVelocityRads(1) +
                std::copysign(GantryTuning::DM_TORQUE_LIMIT_NM/GantryTuning::DM_HOLD_KD, error_deg);
            assert(std::abs(sent_vel-expected_vel)<60.0f/4095);
        }
    }
    gantry_fsm=Class_Gantry_FSM{};
    auto update_fsm = [&](bool right_pressed, bool lift_first_limit) {
        gantry_fsm.Update(true, true, right_pressed, right_pressed,
                          lift_first_limit, false, false, true, true,
                          lift_id5_angle, lift_id6_angle, dm_angle, tick);
    };
    update_fsm(false, false);
    update_fsm(true, false);
    update_fsm(true, true);
    assert(gantry_fsm.GetState()==GantryState::Extend);
    unsigned cycles=0;
    float first_hold_target=0;
    unsigned enables_before_running_stop=0;
    after_cycle = [&] {
        const unsigned cycle=cycles++;
        for (const auto &frame : sent) assert(!is_dm_service(frame, 0xFD));
        if (cycle==0 || cycle==1 || cycle==3) {
            assert(gantry_watch.state==GantryState::Extend);
            const auto &lift=last_control(0x1FF);
            // Both angle and speed loops integrate independently during hold.
            assert(lift.id==0x1FF && lift.data[0]==0 && lift.data[1]==(cycle==1 ? 7 : 5));
            assert(lift.data[2]==0xFF && lift.data[3]==(cycle==1 ? 0xEF : 0xF1));
            assert(std::abs(hold_angle_pid[0].getError()-2*PI/180)<0.000001);
            assert(std::abs(hold_angle_pid[1].getError()+2*PI/180)<0.000001);
            assert(std::abs(lift_hold_pid[0].getError()-(cycle==1 ? 2 : 1.5)*2*PI/180)<0.000001);
            assert(std::abs(lift_hold_pid[1].getError()+(cycle==1 ? 2.5 : 2.25)*2*PI/180)<0.000001);
            if (cycle==0) first_hold_target=lift_hold_pid[0].getError();
            if (cycle==1) assert(lift_hold_pid[0].getError()>first_hold_target+0.001f);
            // Keep integral history within hold mode, clear it after leaving hold.
            assert(std::abs(lift_hold_pid[0].getOutput()-(cycle==1 ? 203.5 : 151.5)*2*PI/180)<0.00001);
            assert(std::abs(lift_hold_pid[1].getOutput()+(cycle==1 ? 509.5 : 454.5)*2*PI/180)<0.00001);
            for (auto &pid : lift_move_pid) assert(pid.getOutput()==0 && pid.getError()==0);
            assert(hold_angle_pid[2].getOutput()==0 && hold_angle_pid[2].getError()==0);
            assert(gantry_watch.angle_deg[0]==lift_id5_angle-2);
            assert(gantry_watch.angle_deg[1]==lift_id6_angle+2);
            const auto &dm=last_control(9);
            assert(dm.id==9);
            const float sent_kd=(dm.data[5]<<4 | dm.data[6]>>4)*5.0f/4095;
            assert(std::abs(sent_kd-GantryTuning::DM_MOVE_KD)<5.0f/4095);
            int encoded_vel=(dm.data[2]<<4)|(dm.data[3]>>4);
            float sent_vel=encoded_vel*60.0f/4095-30;
            const float actual=gantry_dm4310_motor.getVelocityRads(1);
            const float expected_delta=GantryTuning::DM_TORQUE_LIMIT_NM/GantryTuning::DM_MOVE_KD;
            assert(std::abs(sent_vel-actual-expected_delta)<60.0f/4095);
            if (cycle==1) {
                gantry_fsm=Class_Gantry_FSM{};
                update_fsm(false, false);
                update_fsm(true, false);
            }
            else if (cycle==3) {
                enables_before_running_stop=count_enables();
                remoteController.online=false;
                tick+=5;
            }
            return;
        }
        if (cycle==2) {
            assert(gantry_watch.state==GantryState::LiftFirst);
            const auto &lift=last_control(0x1FF);
            // Moving PIDs restart with zero integral: +10/-10 rad/s -> +210/-320.
            assert(lift.id==0x1FF && lift.data[0]==0 && lift.data[1]==210);
            assert(lift.data[2]==0xFE && lift.data[3]==0xC0);
            for (auto &pid : lift_hold_pid) assert(pid.getOutput()==0 && pid.getError()==0);
            for (unsigned i=0; i<2; ++i)
                assert(hold_angle_pid[i].getOutput()==0 && hold_angle_pid[i].getError()==0);
            update_fsm(true, true);
            return;
        }
        assert(cycle==4);
        assert(gantry_watch.state==GantryState::Disabled);
        for (unsigned i=0; i<2; ++i) {
            assert(lift_move_pid[i].getOutput()==0 && lift_move_pid[i].getError()==0);
            assert(lift_hold_pid[i].getOutput()==0 && lift_hold_pid[i].getError()==0);
        }
        for (auto &pid : hold_angle_pid) assert(pid.getOutput()==0 && pid.getError()==0);
        const auto &zero=last_control(0x1FF);
        assert(zero.id==0x1FF);
        for (int i=0;i<8;++i) assert(zero.data[i]==0);
        const auto &dm=last_control(9);
        for (unsigned i=0; i<8; ++i) assert(dm.data[i]==zero_dm_command.data[i]);
        assert(count_enables()==enables_before_running_stop);
        throw TaskStopped{};
    };
    try { gantry_task(nullptr); } catch (const TaskStopped&) {}
    assert(cycles==5);
}
'''.replace("DISPATCHER_SOURCE", dispatcher).replace("TASK_SOURCE", task).replace("CALLBACK_SOURCE", callback).replace("FRICTION_DEFINITION", friction_definition)

with tempfile.TemporaryDirectory(prefix="gantry_motors_") as directory:
    work = Path(directory)
    (work / "fdcan.h").write_text(
        "#pragma once\n#include <cstdint>\n"
        "struct FDCAN_HandleTypeDef {};\nstruct FDCAN_RxHeaderTypeDef {};\n",
        encoding="utf-8",
    )
    (work / "FreeRTOS.h").write_text(
        "#pragma once\n#include <cstdint>\nusing TickType_t=uint32_t;\n"
        "#define pdMS_TO_TICKS(ms) (ms)\n"
        "#define taskENTER_CRITICAL() ((void)0)\n#define taskEXIT_CRITICAL() ((void)0)\n",
        encoding="utf-8"
    )
    (work / "task.h").write_text(
        "#pragma once\n", encoding="utf-8"
    )
    (work / "cmsis_os.h").write_text(
        "#pragma once\n#include <cstdint>\nenum osStatus_t { osOK=0 };\n"
        "void TestTaskDelay(uint32_t);\n"
        "inline osStatus_t osDelay(uint32_t ticks) { TestTaskDelay(ticks); return osOK; }\n",
        encoding="utf-8"
    )
    (work / "stm32h7xx_hal.h").write_text(
        '#pragma once\n#include <cstdint>\nstruct GPIO_TypeDef {};\n'
        'extern GPIO_TypeDef gpio_a;\n#define GPIOA (&gpio_a)\n'
        + ''.join(f'#define GPIO_PIN_{i} (1U<<{i})\n' for i in range(5))
        + 'enum GPIO_PinState { GPIO_PIN_RESET, GPIO_PIN_SET };\n'
        'extern "C" uint32_t HAL_GetTick();\n'
        'GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*, uint16_t);\n', encoding="utf-8")
    cpp, exe = work / "test.cpp", work / "test.exe"
    cpp.write_text(code, encoding="utf-8")
    (work / "gantry_task.hpp").write_text(task_header, encoding="utf-8")
    (work / "gantry_fsm.cpp").write_text(fsm_source, encoding="utf-8")
    cases = [
        ("defaults", {}, False),
        ("valid", valid, True),
        ("swapped_kd", {**valid, "DM_MOVE_KD": "4", "DM_HOLD_KD": "2"}, True),
        ("dm_speed_limit", {**valid, "DM_SPEED_RAD_S": "31"}, False),
        ("dm_move_kd_limit", {**valid, "DM_MOVE_KD": "6"}, False),
        ("dm_hold_kd_limit", {**valid, "DM_HOLD_KD": "6"}, False),
        ("dm_move_kd_zero", {**valid, "DM_MOVE_KD": "0"}, False),
        ("dm_hold_kd_zero", {**valid, "DM_HOLD_KD": "0"}, False),
    ]
    for name, values, configured in cases:
        fixture = header
        for key, value in values.items():
            fixture, count = re.subn(
                r"(static constexpr \w+ " + key + r"\s*=\s*)[^;]+;",
                lambda match: match[1] + value + ";", fixture)
            assert count == 1, key
        (work / "gantry_fsm.hpp").write_text(fixture, encoding="utf-8")
        subprocess.run(
            ["g++", "-std=c++17", "-DTEST_VALID_CONFIG=" + str(int(configured)),
             "-I", str(work), "-I", str(root),
             "-I", str(root / "RtosTask"),
             "-I", str(root / "communication_between_boards"),
             "-I", str(root / "user/core/Alg/PID"),
             "-I", str(root / "user/core/BSP/Common/StateWatch"),
             "-I", str(root / "tests/stubs"), str(cpp),
             str(work / "gantry_fsm.cpp"),
             str(root / "user/core/Alg/PID/pid.cpp"),
             str(root / "user/core/BSP/Common/StateWatch/state_watch.cpp"), "-o", str(exe)], check=True
        )
        subprocess.run([str(exe)], check=True)
        print("PASS: gantry task " + name)
print("PASS: gantry IDs, FDCAN3 TX/RX, PID limits/resets, DM Kd, shared recovery retries/gates, fault latch, zero output, no Off")
