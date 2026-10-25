"""Exercise real input dispatch and switch debounce with a fake HAL clock."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
dispatcher = (root / "communication_between_boards/input_dispatcher.cpp").read_text(encoding="utf-8")
# Only the DT7 enum dependency is substituted. The complete dispatcher is compiled.
dispatcher = dispatcher.replace('#include "../user/core/BSP/RemoteControl/DT7.hpp"', "")
code = r'''
#include <cassert>
#include <cstdint>
namespace BSP::REMOTE_CONTROL {
struct RemoteController { enum { MIDDLE=3, KEY_R=1<<8, KEY_G=1<<10, KEY_V=1<<14 }; };
}
#include "user/core/BSP/switch/switch.hpp"
uint32_t tick = 0;
GPIO_PinState raw = GPIO_PIN_RESET;
extern "C" uint32_t HAL_GetTick() { return tick; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*, uint16_t) { return raw; }
DISPATCHER
int main() {
    InputDispatcher d;
    auto update = [&](bool right, bool left=false, uint16_t key=0) {
        d.Update(3,3,key,left,right);
    };
    update(true); tick=29; update(true); assert(!d.IsRightButtonHeld());
    tick=30; update(true); assert(d.IsRightButtonHeld());
    tick=1000; update(true); assert(!d.IsVisionMode());
    update(false,true,1<<8); tick+=30; update(false,true,1<<8);
    assert(!d.IsRightButtonHeld() && d.IsLeftButtonPressed() && d.IsFrictionOn());
    d.Update(2,2,0,false,false);
    assert(!d.IsRightButtonHeld() && !d.IsFrictionOn());

    BSP::Switch::MicroSwitch s(nullptr, 1);
    raw=GPIO_PIN_SET; tick=0; s.update(); assert(!s.getPress());
    tick=19; s.update(); assert(!s.getPress());
    tick=20; s.update(); assert(s.getPress());
    raw=GPIO_PIN_RESET; tick=21; s.update();
    raw=GPIO_PIN_SET; tick=25; s.update(); assert(s.getPress());
    raw=GPIO_PIN_RESET; tick=30; s.update();
    tick=49; s.update(); assert(s.getPress());
    tick=50; s.update(); assert(!s.getPress());
    raw=GPIO_PIN_SET; tick=UINT32_MAX-10; s.update();
    tick=9; s.update(); assert(s.getPress());
}
'''.replace("DISPATCHER", dispatcher)

with tempfile.TemporaryDirectory(prefix="gantry_input_") as directory:
    work = Path(directory)
    (work / "stm32h7xx_hal.h").write_text(
        '#pragma once\n#include <cstdint>\nstruct GPIO_TypeDef {};\n'
        'enum GPIO_PinState { GPIO_PIN_RESET, GPIO_PIN_SET };\n'
        'extern "C" uint32_t HAL_GetTick();\n'
        'GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*, uint16_t);\n', encoding="utf-8")
    cpp, exe = work / "test.cpp", work / "test.exe"
    cpp.write_text(code, encoding="utf-8")
    subprocess.run(
        ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
         "-I", str(work), "-I", str(root), "-I", str(root / "tests/stubs"),
         "-I", str(root / "communication_between_boards"), str(cpp), "-o", str(exe)],
        check=True)
    subprocess.run([str(exe)], check=True)
print("PASS: right button exclusive to gantry, existing R/left button, switch debounce")
