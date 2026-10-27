"""Cross-check switch order against the actual GPIO initialization and CubeMX pins."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
task = (root / "RtosTask/gantry_task.cpp").read_text(encoding="utf-8")
switches = re.search(r"MicroSwitch limits\[4\]\s*=\s*\{(.*?)\n\};", task, re.S)[1]
assert re.findall(r"\{(GPIO\w), GPIO_PIN_(\d+)\}", switches) == [
    ("GPIOE", "13"), ("GPIOE", "9"), ("GPIOA", "2"), ("GPIOA", "3")]
ioc = (root / "tank_gimbal.ioc").read_text(encoding="utf-8")
for pin in ("PE13", "PE9", "PA2", "PA3"):
    assert re.search(r"Mcu.Pin\d+=" + pin + r"$", ioc, re.M)
    assert pin + ".Signal=GPIO_Input" in ioc
    assert pin + ".GPIO_PuPd=GPIO_PULLDOWN" in ioc
for pin in ("PA0", "PA1"):
    assert not re.search(r"^" + pin + r"\.Signal=GPIO_Input$", ioc, re.M)
gpio = (root / "Core/Src/gpio.c").read_text(encoding="utf-8")
stub = '''#include <cassert>
struct GPIO_TypeDef {};
GPIO_TypeDef a,c,e;
#define GPIOA (&a)
#define GPIOC (&c)
#define GPIOE (&e)
struct GPIO_InitTypeDef { unsigned Pin=0, Mode=0, Pull=0, Speed=0; };
enum { GPIO_PIN_RESET=0, GPIO_MODE_OUTPUT_PP=1, GPIO_MODE_INPUT=2,
       GPIO_NOPULL=0, GPIO_PULLDOWN=1, GPIO_SPEED_FREQ_LOW=0 };
unsigned actual_a=0, actual_e=0;
void HAL_GPIO_WritePin(GPIO_TypeDef*, unsigned, unsigned) {}
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *cfg) {
    if (port==GPIOC) return;
    assert(cfg->Mode==GPIO_MODE_INPUT && cfg->Pull==GPIO_PULLDOWN);
    if (port==GPIOA) actual_a |= cfg->Pin;
    else { assert(port==GPIOE); actual_e |= cfg->Pin; }
}
'''
stub += ''.join(f'#define GPIO_PIN_{i} (1U<<{i})\n' for i in range(16))
stub += ''.join(f'#define __HAL_RCC_GPIO{port}_CLK_ENABLE() ((void)0)\n'
                for port in 'ACEHBD')
with tempfile.TemporaryDirectory(prefix="gantry_gpio_") as directory:
    work = Path(directory)
    (work / "gpio.h").write_text(stub, encoding="utf-8")
    (work / "test.cpp").write_text(gpio + '''
int main() {
    MX_GPIO_Init();
    assert(actual_a==(GPIO_PIN_2|GPIO_PIN_3));
    assert(actual_e==(GPIO_PIN_9|GPIO_PIN_13));
}
''', encoding="utf-8")
    exe = work / "test.exe"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    "-I", work.as_posix(), (work / "test.cpp").as_posix(),
                    "-o", exe.as_posix()], check=True)
    subprocess.run([str(exe)], check=True)
print("PASS: PE13/PE9/PA2/PA3 switch order, GPIO input/pulldown and CubeMX mapping")
