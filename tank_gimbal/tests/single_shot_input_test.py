"""Exercise the real key dispatcher and feeder FSM together on the host."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
dispatcher = (root / "communication_between_boards/input_dispatcher.cpp").read_text(encoding="utf-8")
feeder = (root / "feeder_fsm/feeder_fsm.cpp").read_text(encoding="utf-8")
dt7_include = '#include "../user/core/BSP/RemoteControl/DT7.hpp"'
dispatcher = dispatcher.replace(dt7_include, "")
feeder = feeder.replace(dt7_include, "")
code = r'''
#include <cassert>
#include <cstdint>
namespace BSP::REMOTE_CONTROL {
struct RemoteController { enum { UP=1, DOWN=2, MIDDLE=3, KEY_R=1<<8, KEY_G=1<<10, KEY_V=1<<14 }; };
}
uint32_t tick = 0;
extern "C" uint32_t HAL_GetTick() { return tick; }
// INSERT_DISPATCHER
// INSERT_FEEDER
int main() {
    InputDispatcher d;
    Class_Feeder_FSM f;
    f.Init();
    auto update_key = [&](uint16_t key, bool left) {
        d.Update(3, 3, key, left, false);
        tick += 30;
        d.Update(3, 3, key, left, false);
    };
    update_key(1<<8, false);
    assert(d.IsFrictionOn());
    update_key(0, false);
    update_key(1<<10, false);
    assert(d.GetKeyboardMask() == (1<<10));
    update_key(0, false);
    update_key(0, true);
    Struct_Feeder_Input input{};
    input.s1=3; input.s2=3; input.is_keymouse=true;
    input.friction_on=d.IsFrictionOn(); input.fire_triggered=d.IsFireTriggered();
    input.single_shot_allowed=true;
    f.Update(input, 0, 0, 0);
    assert(f.Get_Now_Status_Serial() == FEEDER_SINGLE_SHOT);
    f.Update(input, 0, 0, 0);
    assert(f.Get_Control_Type() == FEEDER_CONTROL_ANGLE);
    input.friction_on=false;
    f.Update(input, 0, 0, 0);
    assert(f.Get_Now_Status_Serial() == FEEDER_STOP);

    Class_Feeder_FSM remote;
    remote.Init();
    Struct_Feeder_Input r{};
    r.s1=1; r.s2=2; r.single_shot_allowed=true; r.scroll_value=1.0f;
    remote.Update(r, 0, 0, 0);
    assert(remote.Get_Now_Status_Serial() == FEEDER_SINGLE_SHOT);
    Class_Feeder_FSM reverse;
    reverse.Init();
    r.scroll_value=-1.0f;
    reverse.Update(r, 0, 0, 0);
    assert(reverse.Get_Now_Status_Serial() == FEEDER_MANUAL_REVERSE);
}
'''.replace("// INSERT_DISPATCHER", dispatcher).replace("// INSERT_FEEDER", feeder)

with tempfile.TemporaryDirectory(prefix="single_shot_") as directory:
    work = Path(directory)
    cpp, exe = work / "test.cpp", work / "test.exe"
    cpp.write_text(code, encoding="utf-8")
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-I", str(root),
                    "-I", str(root / "tests/stubs"), "-I", str(root / "communication_between_boards"),
                    "-I", str(root / "feeder_fsm"), "-I", str(root / "user/core/Alg/FSM"), str(cpp),
                    str(root / "user/core/Alg/FSM/alg_fsm.cpp"), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print("PASS: G is free, keyboard fires single shot, remote forward/reverse preserved")
