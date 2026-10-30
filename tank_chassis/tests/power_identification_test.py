"""Execute the actual USB, UART10 and four-wheel sampling code on the host."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
drive = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8")
remote = (ROOT / "RtosTask/remote_task.cpp").read_text(encoding="utf-8")


def block(source, anchor):
    start = source.index(anchor)
    pos = source.index("{", start) + 1
    depth = 1
    end = pos
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def run(name, code):
    with tempfile.TemporaryDirectory(prefix=name) as directory:
        cpp = Path(directory) / "test.cpp"
        exe = Path(directory) / "test.exe"
        cpp.write_text(code, encoding="utf-8")
        subprocess.run(["g++", "-std=c++17", "-I", ROOT.as_posix(),
                        cpp.as_posix(), "-o", exe.as_posix()], check=True)
        subprocess.run([str(exe)], check=True)
    print("PASS:", name)


usb = block(drive, "void vofa_sendN(const float *data, uint8_t count)")
assert "static uint8_t" in usb, "USB VOFA needs its own persistent transmit buffer"
common = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include <cmath>
int critical=0;
#define taskENTER_CRITICAL() (++critical)
#define taskEXIT_CRITICAL() (--critical)
'''
run("usb-power-vofa", common + r'''
struct USBD_CDC_HandleTypeDef { unsigned TxState=0; } cdc;
enum { USBD_STATE_DEFAULT, USBD_STATE_CONFIGURED };
struct { void *pClassData=nullptr; int dev_state=USBD_STATE_DEFAULT; } hUsbDeviceHS;
uint8_t *active=nullptr;
unsigned sends=0, size=0;
void CDC_Transmit_HS(uint8_t *data, uint16_t length) {
    assert(critical==1);
    active=data; size=length; ++sends; cdc.TxState=1;
}
''' + usb + r'''
int main() {
    float values[15]; for (int i=0;i<15;++i) values[i]=i-7.25f;
    vofa_sendN(values,15); assert(sends==0 && critical==0);
    hUsbDeviceHS.pClassData=&cdc;
    vofa_sendN(values,15); assert(sends==0 && critical==0);
    hUsbDeviceHS.dev_state=USBD_STATE_CONFIGURED;
    vofa_sendN(nullptr,15); vofa_sendN(values,0); vofa_sendN(values,16);
    assert(sends==0 && critical==0);
    vofa_sendN(values,15); assert(sends==1 && size==64 && critical==0);
    assert(std::memcmp(active,values,sizeof(values))==0);
    const uint8_t tail[4]={0,0,0x80,0x7f};
    assert(std::memcmp(active+60,tail,4)==0);
    uint8_t saved[64]; std::memcpy(saved,active,64);
    values[0]=999;
    vofa_sendN(values,15);
    assert(sends==1 && std::memcmp(active,saved,64)==0 && critical==0);
    cdc.TxState=0; vofa_sendN(values,15);
    assert(sends==2 && std::memcmp(active,values,60)==0);
}
''')

assert "// ---- 7. USART10 功率计" in remote, "UART10 power meter receive path is missing"
startup = remote[remote.index("// ---- 7. USART10 功率计"):remote.index("// ---- 8. 主循环")]
rx = block(remote[remote.index('extern "C" void HAL_UARTEx_RxEventCallback'):],
           "if (huart == &huart10)")
error = block(remote[remote.index('extern "C" void HAL_UART_ErrorCallback'):],
              "if (huart == &huart10)")
uart = common + r'''
#include <functional>
struct PowerData_t { float voltage,current,power; } PowerData{};
uint32_t power_meter_tick=0, power_meter_sequence=0, tick=100;
bool power_meter_valid=false;
uint8_t power_rx_buffer[12];
struct DMA { bool busy=false; } dma;
struct UART_HandleTypeDef { DMA *hdmarx=&dma; } huart10;
enum { HAL_UART_RXEVENT_HT=1, HAL_OK=0, DMA_IT_HT=1 };
int event_type=0, aborts=0, starts=0, disabled=0;
bool abort_ok=true;
uint32_t HAL_GetTick() { return tick; }
int HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *) { return event_type; }
int HAL_UART_AbortReceive(UART_HandleTypeDef *) {
    ++aborts; if(!abort_ok) return 1; dma.busy=false; return HAL_OK;
}
#define __HAL_DMA_DISABLE_IT(h,f) (++disabled)
#define __HAL_UART_CLEAR_OREFLAG(h) ((void)0)
#define __HAL_UART_CLEAR_NEFLAG(h) ((void)0)
#define __HAL_UART_CLEAR_FEFLAG(h) ((void)0)
#define __HAL_UART_CLEAR_PEFLAG(h) ((void)0)
namespace HAL::UART {
struct Data { uint8_t *buffer; uint16_t size; };
struct Device {
    std::function<void(const Data&)> callback;
    UART_HandleTypeDef *get_handle() { return &huart10; }
    void register_rx_callback(std::function<void(const Data&)> fn) { callback=fn; }
    void trigger_rx_callbacks(const Data &data) { assert(!dma.busy); callback(data); }
    bool receive_dma_idle(const Data &data) {
        assert(data.buffer==power_rx_buffer && data.size==12);
        if(dma.busy) return false;
        ++starts; dma.busy=true;
        // Deliberately overwrite once restarted: publication must already be done.
        std::memset(power_rx_buffer,0,12); return true;
    }
};
struct Bus { Device device; Device &get_uart10() { return device; } } bus;
Bus &get_uart_bus_instance() { return bus; }
}
HAL::UART::Data uart10_rx_data{power_rx_buffer,sizeof(power_rx_buffer)};
'''
uart += "void startup() {\n" + startup + "\n}\n"
uart += "void rx(UART_HandleTypeDef *huart, uint16_t Size) {\n" + rx + "\n}\n"
uart += "void error(UART_HandleTypeDef *huart) {\n" + error + "\n}\n"
uart += r'''
void feed(float voltage,float current,float power,uint16_t count=12) {
    float values[3]={voltage,current,power};
    std::memcpy(power_rx_buffer,values,12); dma.busy=false; rx(&huart10,count);
}
int main() {
    startup(); assert(starts==1 && disabled==1 && !power_meter_valid);
    event_type=HAL_UART_RXEVENT_HT; rx(&huart10,6);
    assert(starts==1 && power_meter_sequence==0); event_type=0;
    feed(24,2,48);
    assert(power_meter_valid && power_meter_sequence==1 && power_meter_tick==100);
    assert(PowerData.voltage==24 && PowerData.current==2 && PowerData.power==48);
    tick=105; feed(24,2,99,8);
    assert(!power_meter_valid && power_meter_sequence==1 && PowerData.power==48);
    feed(24,NAN,48); assert(!power_meter_valid && power_meter_sequence==1);
    feed(24,2,INFINITY); assert(!power_meter_valid && power_meter_sequence==1);
    feed(24,-1,-24); assert(power_meter_valid && PowerData.power==-24);
    assert(power_meter_tick==105 && power_meter_sequence==2);
    error(&huart10); assert(!power_meter_valid && aborts==1 && dma.busy);
    feed(24,0,0); assert(power_meter_valid && power_meter_sequence==3);
    const int before=starts; abort_ok=false; error(&huart10);
    assert(!power_meter_valid && starts==before);
}
'''
run("uart10-power-meter", uart)

start = drive.index("// 功率辨识：每个功率计新帧")
sample = drive[start:drive.index("// 更新裁判回复快照", start)]
coefficients = re.search(r"float poly_coeffs\[6\] = \{.*?\};", drive, re.S).group(0)
sampling = common + r'''
#include "user/core/Alg/PowerControl/PowerControlBase.hpp"
struct { float voltage=24, current=2, power=48; } PowerData;
uint32_t power_meter_tick=1000, power_meter_sequence=0;
bool power_meter_valid=false;
struct Motor {
    float current[4]={-2,3,4,-5}, speed[4]={-100,200,300,-400};
    unsigned mask=15;
    float getCurrent(int id) { assert(critical==1); return current[id-1]; }
    float getVelocityRads(int id) { assert(critical==1); return speed[id-1]; }
    bool isConnected(int id,int) { assert(critical==1); return mask&(1U<<(id-1)); }
} chassis_motor;
unsigned sends=0;
float observed[15];
void vofa_sendN(const float *data,uint8_t count) {
    assert(count==15 && critical==0); ++sends; std::memcpy(observed,data,sizeof(observed));
}
uint32_t last_power_meter_sequence=0;
ALG::PowerControl::PowerControlBase<4> power_identification_model;
'''
sampling += coefficients + "\nvoid sample() {\n" + sample + "\n}\n"
sampling += r'''
int main() {
    sample(); assert(sends==0);
    power_meter_valid=true; power_meter_sequence=1;
    sample(); assert(sends==1 && critical==0);
    assert(observed[0]==48 && observed[10]==1 && observed[11]==24 && observed[12]==2);
    assert(observed[13]==15 && observed[14]==1);
    float expected=0;
    for(int i=0;i<4;++i) {
        const float a=chassis_motor.current[i],w=chassis_motor.speed[i];
        assert(observed[1+2*i]==a && observed[2+2*i]==w);
        expected+=poly_coeffs[0]+poly_coeffs[1]*a+poly_coeffs[2]*fabsf(w)
            +poly_coeffs[3]*a*w+poly_coeffs[4]*a*a+poly_coeffs[5]*w*w;
    }
    assert(std::fabs(observed[9]-expected)<1e-4f);
    sample(); assert(sends==1); // No duplicates at faster control-task rate.
    power_meter_sequence=2; power_meter_tick=1005; chassis_motor.mask=7;
    sample(); assert(sends==2 && observed[13]==7 && observed[14]==2);
    power_meter_valid=false; power_meter_sequence=3;
    sample(); assert(sends==2);
}
'''
run("four-wheel-power-sampling", sampling)

# Sampling remains outside drive-enable/STOP branches; raw rotor speed is used.
uncommented = re.sub(r"//[^\n]*|/\*.*?\*/", "", drive[:start], flags=re.S)
assert uncommented.count("{") - uncommented.count("}") == 2
stairs = (ROOT / "RtosTask/up_stair.cpp").read_text(encoding="utf-8")
assert not re.search(r"\bvofa_sendN\s*\(", re.sub(r"//[^\n]*|/\*.*?\*/", "", stairs, flags=re.S)), \
    "IMU UART10 telemetry must not mix with the USB power stream"
assert "huart10.Init.BaudRate = 115200;" in (ROOT / "Core/Src/usart.c").read_text()
assert "USART10.BaudRate=115200" in (ROOT / "tank_chassis.ioc").read_text()
print("PASS: STOP telemetry, exclusive USB power stream, UART10 configuration")
