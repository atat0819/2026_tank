from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
driver = (root / 'user/core/BSP/IMU/HI12_imu.hpp').read_text(encoding='utf-8-sig')
base = (root / 'user/core/BSP/IMU/HI12Base.hpp').read_text(encoding='utf-8-sig')
base = base.replace('../user/core/BSP/Common/StateWatch/state_watch.hpp', 'state_watch.hpp')
base = base.replace('../user/core/BSP/Common/StateWatch/buzzer_manager.hpp', 'state_watch.hpp')
stub = r'''
#pragma once
#include <cstdint>
extern uint32_t now_ms, valid_updates;
struct UART_HandleTypeDef {};
#define RESET 0
#define UART_FLAG_ORE 0
#define __HAL_UART_GET_FLAG(huart, flag) ((void)(huart), (void)(flag), 0)
#define __HAL_UART_CLEAR_OREFLAG(huart) ((void)(huart))
#define HAL_UARTEx_ReceiveToIdle_DMA(huart, data, size) ((void)(huart), (void)(data), (void)(size))
namespace BSP::WATCH_STATE {
enum class Status { ONLINE, OFFLINE };
struct StateWatch {
    uint32_t last=0, timeout;
    Status status=Status::OFFLINE;
    StateWatch(int t):timeout(t) {}
    void UpdateLastTime() { last=now_ms; ++valid_updates; }
    void UpdateTime() {}
    void CheckStatus() { status=now_ms-last<=timeout ? Status::ONLINE : Status::OFFLINE; }
    Status GetStatus() { return status; }
};
struct BuzzerManagerSimple {
    static BuzzerManagerSimple& getInstance() { static BuzzerManagerSimple b; return b; }
    void requestIMURing() {}
};
}
'''
test = r'''
#include "HI12_imu.hpp"
#include <cassert>
#include <array>
#include <vector>
#include <cmath>
uint32_t now_ms=1000, valid_updates=0;
using Frame=std::array<uint8_t,82>;
uint16_t crc(const uint8_t *p, unsigned n, uint16_t c=0) {
    for (unsigned i=0;i<n;++i) {
        c ^= static_cast<uint16_t>(p[i])<<8;
        for (int b=0;b<8;++b) c=(c&0x8000) ? (c<<1)^0x1021 : c<<1;
    }
    return c;
}
Frame frame(float yaw) {
    Frame f{}; f[0]=0x5a; f[1]=0xa5; f[2]=76;
    const float values[]={1.25f,-2.5f,3.75f,4.5f,-5.25f,6.5f,7.25f,-8.5f,yaw,0.1f,0.2f,0.3f,0.4f};
    const unsigned offsets[]={18,22,26,30,34,38,54,58,62,66,70,74,78};
    for (unsigned i=0;i<13;++i) memcpy(f.data()+offsets[i],values+i,4);
    const uint16_t c=crc(f.data()+6,76,crc(f.data(),4));
    f[4]=c&255; f[5]=c>>8; return f;
}
void check(BSP::IMU::HI12_float &imu,float yaw) {
    assert(imu.GetAngle(2)==yaw && imu.GetAngle(0)==7.25f && imu.GetAngle(1)==-8.5f);
    assert(imu.GetGyro(0)==4.5f && imu.GetGyro(1)==-5.25f && imu.GetGyro(2)==6.5f);
    assert(imu.GetAcc(0)==1.25f && imu.GetAcc(1)==-2.5f && imu.GetAcc(2)==3.75f);
    assert(imu.GetQuaternion(3)==0.4f);
}
int main() {
    assert(crc(reinterpret_cast<const uint8_t*>("123456789"),9)==0x31c3);
    const auto f=frame(12.5f), second=frame(-23.0f);
    BSP::IMU::HI12_float imu;
    imu.DataUpdate(f.data(),82); check(imu,12.5f);
    // Exact-length allocations expose accidental reads beyond a short fragment.
    for (unsigned split=1;split<82;++split) {
        BSP::IMU::HI12_float fragmented;
        std::vector<uint8_t> a(f.begin(),f.begin()+split), b(f.begin()+split,f.end());
        unsigned before=valid_updates;
        fragmented.DataUpdate(a.data(),static_cast<uint16_t>(a.size()));
        assert(valid_updates==before);
        fragmented.DataUpdate(b.data(),static_cast<uint16_t>(b.size()));
        assert(valid_updates==before+1); check(fragmented,12.5f);
    }
    BSP::IMU::HI12_float bytewise;
    for (auto byte:f) bytewise.DataUpdate(&byte,1);
    check(bytewise,12.5f);
    for (unsigned prefix=1;prefix<83;++prefix) {
        std::vector<uint8_t> shifted(prefix,0x5a);
        shifted.insert(shifted.end(),f.begin(),f.end());
        BSP::IMU::HI12_float offset;
        offset.DataUpdate(shifted.data(),static_cast<uint16_t>(shifted.size()));
        check(offset,12.5f);
    }
    auto bad=f; bad[4]^=1;
    unsigned before=valid_updates;
    now_ms+=101;
    imu.DataUpdate(bad.data(),82);
    assert(valid_updates==before && !imu.isConnected()); check(imu,12.5f);
    const uint8_t bad_length[]={0x5a,0xa5,0xff,0xff,0,0,0x5a};
    imu.DataUpdate(bad_length,sizeof(bad_length));
    imu.DataUpdate(nullptr,0); imu.DataUpdate(f.data(),0);
    assert(valid_updates==before);
    imu.DataUpdate(second.data(),82); check(imu,-23.0f); assert(imu.isConnected());
    // A dropped byte or inserted byte must not prevent the next full frame decoding.
    for (bool insert:{false,true}) {
        auto stream=std::vector<uint8_t>(f.begin(),f.end());
        if (insert) stream.insert(stream.begin()+20,0xaa);
        else stream.erase(stream.begin()+20);
        stream.insert(stream.end(),second.begin(),second.end());
        imu.ResetReceiveBuffer(); before=valid_updates;
        imu.DataUpdate(stream.data(),static_cast<uint16_t>(stream.size()));
        assert(valid_updates==before+1); check(imu,-23.0f);
    }
    std::vector<uint8_t> joined(f.begin(),f.end());
    joined.insert(joined.end(),second.begin(),second.end());
    before=valid_updates; imu.DataUpdate(joined.data(),static_cast<uint16_t>(joined.size()));
    assert(valid_updates==before+2); check(imu,-23.0f);
    imu.DataUpdate(f.data(),40); imu.ResetReceiveBuffer(); before=valid_updates;
    imu.DataUpdate(f.data()+40,42); assert(valid_updates==before);
    imu.DataUpdate(second.data(),82); check(imu,-23.0f);
}
'''
with tempfile.TemporaryDirectory(prefix='hi12_receive_') as tmp:
    path = Path(tmp)
    (path / 'HI12_imu.hpp').write_text(driver, encoding='utf-8')
    (path / 'HI12Base.hpp').write_text(base, encoding='utf-8')
    (path / 'state_watch.hpp').write_text(stub, encoding='utf-8')
    (path / 'test.cpp').write_text(test, encoding='utf-8')
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-I', str(path), str(path / 'test.cpp'), '-o', str(path / 'test.exe')], check=True)
    subprocess.run([str(path / 'test.exe')], check=True)
print('PASS: HI12 fields, all split positions, byte stream, offset, CRC/length rejection, stale feedback and resynchronization')
