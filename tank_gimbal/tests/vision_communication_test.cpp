#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#define __REV(value) __builtin_bswap32(value)

static uint32_t fake_tick;
static uint8_t sent[64];
static uint16_t sent_size;
static uint8_t *active_tx;

extern "C" uint32_t HAL_GetTick(void) { return fake_tick; }
extern "C" uint8_t CDC_Transmit_HS(uint8_t *buf, uint16_t len)
{
    sent_size = len;
    active_tx = buf;
    memcpy(sent, buf, len);
    return 0;
}

#include "../user/core/BSP/version/vision_communication.cpp"

USBD_CDC_HandleTypeDef fake_cdc = {};
extern "C" { USBD_HandleTypeDef hUsbDeviceHS = {&fake_cdc}; }

static uint16_t crc(const uint8_t *data, uint16_t size)
{
    uint16_t result = 0xffff;
    for (uint16_t i = 0; i < size; ++i)
    {
        result ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
            result = (result & 1) ? (result >> 1) ^ 0x8408 : result >> 1;
    }
    return result;
}

static void put_float(uint8_t *dst, float value) { memcpy(dst, &value, 4); }
static float get_float(const uint8_t *src)
{
    float value;
    memcpy(&value, src, 4);
    return value;
}

int main()
{
    assert(crc(reinterpret_cast<const uint8_t *>("123456789"), 9) == 0x6f91);
    BSP::Vision::VisionCommunicator comm;
    const float q[4] = {1.0f, 0.0f, 0.25f, -0.5f};
    comm.SendToVision(q, 1.25f, -0.5f, 0.3f, -0.2f, 18.0f, 0x1234, 1);
    assert(sent_size == 43 && sent[0] == 'S' && sent[1] == 'P' && sent[2] == 1);
    assert(get_float(sent + 3) == q[0] && get_float(sent + 15) == q[3]);
    assert(get_float(sent + 19) == 1.25f && get_float(sent + 23) == -0.5f);
    assert(get_float(sent + 27) == 0.3f && get_float(sent + 31) == -0.2f);
    assert(get_float(sent + 35) == 18.0f && sent[39] == 0x34 && sent[40] == 0x12);
    assert((sent[41] | (sent[42] << 8)) == crc(sent, 41));
    fake_cdc.TxState = 1;
    comm.SendToVision(q, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
    assert(active_tx[2] == 1 && sent[2] == 1);
    fake_cdc.TxState = 0;

    uint8_t frame[29] = {'S', 'P', 2};
    put_float(frame + 3, 1.4f);
    put_float(frame + 7, 0.1f);
    put_float(frame + 11, 0.2f);
    put_float(frame + 15, -0.4f);
    put_float(frame + 19, -0.1f);
    put_float(frame + 23, -0.2f);
    const uint16_t check = crc(frame, 27);
    frame[27] = check & 0xff;
    frame[28] = check >> 8;
    assert(!comm.IsDataFresh());
    comm.ParseRxData(frame, 8);
    assert(!comm.IsVisionReady());
    comm.ParseRxData(frame + 8, 21);
    assert(comm.IsVisionReady() && comm.IsDataFresh());
    assert(comm.GetYawAngle() == 1.4f && comm.GetPitchAngle() == -0.4f);
    assert(comm.GetMode() == 2);
    assert(comm.GetYawVelocity() == 0.1f && comm.GetYawAcceleration() == 0.2f);
    assert(comm.GetPitchVelocity() == -0.1f && comm.GetPitchAcceleration() == -0.2f);

    frame[2] = 0;
    const uint16_t idle_crc = crc(frame, 27);
    frame[27] = idle_crc & 0xff;
    frame[28] = idle_crc >> 8;
    uint8_t combined[59] = {0xff};
    memcpy(combined + 1, frame, 29);
    frame[2] = 1;
    const uint16_t aim_crc = crc(frame, 27);
    frame[27] = aim_crc & 0xff;
    frame[28] = aim_crc >> 8;
    memcpy(combined + 30, frame, 29);
    comm.ParseRxData(combined, sizeof(combined));
    assert(comm.IsVisionReady());

    // yaw_vel 暂不用于控制，但它仍属于完整 29 字节帧和 CRC 覆盖范围。
    frame[7] ^= 1;
    fake_tick = 2000;
    comm.ParseRxData(frame, sizeof(frame));
    assert(!comm.IsDataFresh());
    assert(comm.GetYawAngle() == 1.4f);
    assert(comm.GetYawVelocity() == 0.1f && comm.GetMode() == 1);
    frame[7] ^= 1;
    put_float(frame + 7, std::numeric_limits<float>::infinity());
    const uint16_t nonfinite_crc = crc(frame, 27);
    frame[27] = nonfinite_crc & 0xff;
    frame[28] = nonfinite_crc >> 8;
    comm.ParseRxData(frame, sizeof(frame));
    assert(comm.GetYawVelocity() == 0.1f && !comm.IsDataFresh());
    put_float(frame + 7, 0.1f);
    frame[2] = 0;
    const uint16_t stop_crc = crc(frame, 27);
    frame[27] = stop_crc & 0xff;
    frame[28] = stop_crc >> 8;
    comm.ParseRxData(frame, sizeof(frame));
    // mode=0 仅保存供读取，不再决定电控是否采用视觉角度。
    assert(comm.IsVisionReady() && comm.IsDataFresh());
    assert(comm.GetMode() == 0);

    // mode 字节只保存，不影响其余字段的接收。
    frame[2] = 3;
    put_float(frame + 3, 0.9f);
    const uint16_t other_mode_crc = crc(frame, 27);
    frame[27] = other_mode_crc & 0xff;
    frame[28] = other_mode_crc >> 8;
    comm.ParseRxData(frame, sizeof(frame));
    assert(comm.GetMode() == 3 && comm.GetYawAngle() == 0.9f);
}
