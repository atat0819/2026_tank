"""Real motor feedback must arrive before online status can latch gantry home."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
task = (root / "RtosTask/gantry_task.cpp").read_text(encoding="utf-8")
definition = re.search(r"BSP::Motor::Dji::GM3508<2> gantry_3508_motor\([^;]+;", task, re.S)[0]
definition = definition.replace("gantry_3508_motor_idxs", "ids").replace("gantry_3508_motor", "lift")
code = r'''
#define BUZZER_HPP
#include "user/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "user/core/BSP/Motor/DM/DmMotor.hpp"
#include "gantry_fsm.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
uint32_t tick=0;
extern "C" uint32_t HAL_GetTick() { return tick; }
const uint8_t ids[2]={5,6};
BSP::Motor::Dji::GM3508<2> lift(0x200,ids,0x1ff);
BSP::Motor::DM::J4310<1> dm(0,{9},{9},HAL::FDCAN::FdcanDeviceId::HAL_Fdcan3);
int main() {
    Class_Gantry_FSM f;
    for (unsigned t : {0U,1U,50U,99U,100U,1000U}) {
        tick=t;
        const bool a=lift.isConnected(1,5), b=lift.isConnected(2,6);
        assert(!a && !b && !dm.isConnected(1,9));
        f.Update(false,false,a&&b,false,false,false,false,false,false,
                 0,0,0,0,0,0,0,tick);
    }
    HAL::FDCAN::Frame sample{};
    sample.dlc=8; sample.id=0x205; sample.data[0]=0x10;
    lift.Parse(sample);
    assert(lift.isConnected(1,5) && !lift.isConnected(2,6));
    sample.id=0x206; lift.Parse(sample);
    sample.id=9; sample.data[0]=0x19; dm.Parse(sample);
    assert(lift.isConnected(2,6) && dm.isConnected(1,9));
    const double a=lift.getAddAngleDeg(1), b=lift.getAddAngleDeg(2);
    assert(std::abs(a)>GantryTuning::LIFT_HOME_TOLERANCE_DEG);
    assert(std::abs(b)>GantryTuning::LIFT_HOME_TOLERANCE_DEG);
    assert(lift.params_.reduction_ratio==19);
    assert(std::abs(a-180.0/19)<1e-5 && std::abs(b-180.0/19)<1e-5);
    f.Update(true,true,true,false,false,false,false,false,true,
             a,b,0,0,0,0,0,tick);
    f.Update(true,true,true,true,true,false,false,false,true,
             a,b,0,0,0,0,0,tick+1);
    assert(f.GetState()==GantryState::LiftFirst && f.GetFault()==GantryFault::None);
    tick+=99;
    assert(lift.isConnected(1,5) && dm.isConnected(1,9));
    ++tick;
    assert(!lift.isConnected(1,5) && !dm.isConnected(1,9));
    sample.id=0x205; sample.data[2]=0x04; sample.data[3]=0x74; // Rotor 1140 rpm.
    sample.data[4]=0x20; sample.data[5]=0;
    lift.Parse(sample);
    assert(std::abs(lift.getVelocityRads(1)-2*PI)<1e-5);
    assert(lift.getCurrent(1)==10); // Current remains in A, independent of reduction ratio.
    // A genuine first feedback at tick zero must also count as received.
    static BSP::Motor::Dji::GM3508<2> at_zero(0x200,ids,0x1ff);
    tick=0; sample.id=0x205; at_zero.Parse(sample);
    assert(at_zero.params_.reduction_ratio==1); // Existing default instances stay at rotor units.
    assert(at_zero.isConnected(1,5) && !at_zero.isConnected(2,6));
    std::cout<<"PASS: no online before first feedback; correct startup home; timeout unchanged; tick-zero feedback accepted\n";
}
'''
code = code.replace("BSP::Motor::Dji::GM3508<2> lift(0x200,ids,0x1ff);", definition)
with tempfile.TemporaryDirectory(prefix="motor_feedback_startup_") as directory:
    work = Path(directory)
    (work / "fdcan.h").write_text(
        "#pragma once\n#include <cstdint>\n"
        "struct FDCAN_HandleTypeDef {};\nstruct FDCAN_RxHeaderTypeDef {};\n", encoding="utf-8")
    (work / "test.cpp").write_text(code, encoding="utf-8")
    exe = work / "test.exe"
    subprocess.run([
        "g++", "-std=c++17", "-I", work.as_posix(), "-I", root.as_posix(),
        "-I", (root / "feeder_fsm").as_posix(),
        "-I", (root / "user/core/BSP/Common/StateWatch").as_posix(),
        "-I", (root / "tests/stubs").as_posix(),
        (work / "test.cpp").as_posix(),
        (root / "user/core/BSP/Common/StateWatch/state_watch.cpp").as_posix(),
        (root / "feeder_fsm/gantry_fsm.cpp").as_posix(), "-o", exe.as_posix()
    ], check=True)
    subprocess.run([str(exe)], check=True)
