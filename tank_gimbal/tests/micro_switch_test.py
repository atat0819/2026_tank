from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as tmp:
    work = Path(tmp)
    (work / 'stm32h7xx_hal.h').write_text('''#pragma once
#include <stdint.h>
struct GPIO_TypeDef {};
enum GPIO_PinState { GPIO_PIN_RESET, GPIO_PIN_SET };
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*, uint16_t);
uint32_t HAL_GetTick();
''')
    (work / 'test.cpp').write_text('''#include "switch.hpp"
#include <cassert>
static GPIO_PinState level = GPIO_PIN_SET;
static uint32_t now = 0;
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*, uint16_t) { return level; }
uint32_t HAL_GetTick() { return now; }
int main() {
    GPIO_TypeDef port;
    BSP::Switch::MicroSwitch sw(&port, 0);
    assert(!sw.getPress());
    sw.update(); now = 20; sw.update(); assert(!sw.getPress());
    level = GPIO_PIN_RESET; now = 21; sw.update();
    now = 40; sw.update(); assert(!sw.getPress());
    now = 41; sw.update(); assert(sw.getPress());
    level = GPIO_PIN_SET; now = 42; sw.update();
    now = 61; sw.update(); assert(sw.getPress());
    now = 62; sw.update(); assert(!sw.getPress());
    BSP::Switch::MicroSwitch startup(&port, 0);
    level = GPIO_PIN_RESET; now = 100; startup.update();
    assert(!startup.getPress()); now = 120; startup.update(); assert(startup.getPress());
}
''')
    exe = work / 'test.exe'
    subprocess.run(['g++', '-std=c++17', '-I', str(work), '-I', str(root / 'user/core/BSP/switch'), str(work / 'test.cpp'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
