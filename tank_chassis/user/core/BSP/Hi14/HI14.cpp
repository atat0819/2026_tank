#include "HI14.hpp"
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t IMUDataQueue;

IMU_Handle_t hImu;
IMU_Data_t ImuData;
IMU_Float_t ImuFloat;
volatile uint8_t ImuDataReady = 0;

HI14 hi14(hImu, ImuData, ImuFloat, ImuDataReady);

HI14::HI14(IMU_Handle_t &handle, IMU_Data_t &data, IMU_Float_t &float_data,
           volatile uint8_t &data_ready)
    : handle_(handle), data_(data), float_data_(float_data), data_ready_(data_ready)
{
}

void HI14::Init()
{
    handle_.Step = 0;
    handle_.Index = 0;
    handle_.ErrCount = 0;
    data_ready_ = 0;
}

// HI92: 5A A5 + payload length (48) + CRC16 + 48-byte payload, no tail.
// CRC16/XMODEM: polynomial 0x1021, initial value 0; omit the two CRC bytes.
static uint16_t Hi14Crc16(const uint8_t *buffer)
{
    uint16_t crc = 0;
    for (uint16_t i = 0; i < IMU_DATA_LEN; ++i)
    {
        if (i == 4U || i == 5U)
        {
            continue;
        }
        crc ^= static_cast<uint16_t>(buffer[i]) << 8;
        for (uint8_t bit = 0; bit < 8U; ++bit)
        {
            crc = (crc & 0x8000U) ? (crc << 1) ^ 0x1021U : crc << 1;
        }
    }
    return crc;
}

bool HI14::UartRxCallBack(uint8_t rxdata)
{
    if (handle_.Step == 0U)
    {
        if (rxdata == 0x5AU)
        {
            handle_.RxBuffer[0] = rxdata;
            handle_.Step = 1U;
        }
        return false;
    }
    if (handle_.Step == 1U)
    {
        if (rxdata == 0xA5U)
        {
            handle_.RxBuffer[1] = rxdata;
            handle_.Index = 2U;
            handle_.Step = 2U;
        }
        else if (rxdata != 0x5AU)
        {
            handle_.Step = 0U;
        }
        return false;
    }

    handle_.RxBuffer[handle_.Index++] = rxdata;
    // Only the configured HI92 payload is accepted; reject length before filling the buffer.
    if (handle_.Index == 4U &&
        (handle_.RxBuffer[2] != 48U || handle_.RxBuffer[3] != 0U))
    {
        ++handle_.ErrCount;
        handle_.Step = 0U;
        return false;
    }
    if (handle_.Index < IMU_DATA_LEN)
    {
        return false;
    }
    handle_.Step = 0U;
    const uint8_t *const buffer = handle_.RxBuffer;
    const uint16_t received_crc = static_cast<uint16_t>(buffer[4]) |
                                  (static_cast<uint16_t>(buffer[5]) << 8);
    if (buffer[IMU_CMD_POS] != IMU_EXPECTED_CMD || Hi14Crc16(buffer) != received_crc)
    {
        ++handle_.ErrCount;
        return false;
    }

    // Payload starts at byte 6; offsets below are relative to the HI92 tag.
    const uint8_t *const payload = buffer + 6;
    memcpy(&data_.Roll, payload + 28, 4);
    memcpy(&data_.Pitch, payload + 32, 4);
    memcpy(&data_.Yaw, payload + 36, 4);
    // Preserve the confirmed RFU mapping: Pitch=X, Roll=Y, Yaw=Z.
    memcpy(&data_.GyrPitch, payload + 10, 2);
    memcpy(&data_.GyrRoll, payload + 12, 2);
    memcpy(&data_.GyrYaw, payload + 14, 2);
    memcpy(&data_.Qw, payload + 40, 2);
    memcpy(&data_.Qx, payload + 42, 2);
    memcpy(&data_.Qy, payload + 44, 2);
    memcpy(&data_.Qz, payload + 46, 2);

    // Official HI92 units: Euler 0.001 deg/LSB; gyro 0.001 rad/s/LSB.
    float_data_.roll = static_cast<float>(data_.Roll) * 0.001f;
    float_data_.pitch = static_cast<float>(data_.Pitch) * 0.001f;
    float_data_.yaw = static_cast<float>(data_.Yaw) * 0.001f;
    constexpr float GYRO_DPS_PER_LSB = 0.001f * 180.0f / 3.14159265358979323846f;
    float_data_.gyr_pitch = static_cast<float>(data_.GyrPitch) * GYRO_DPS_PER_LSB;
    float_data_.gyr_roll = static_cast<float>(data_.GyrRoll) * GYRO_DPS_PER_LSB;
    float_data_.gyr_yaw = static_cast<float>(data_.GyrYaw) * GYRO_DPS_PER_LSB;
    float_data_.qw = static_cast<float>(data_.Qw) * 0.0001f;
    float_data_.qx = static_cast<float>(data_.Qx) * 0.0001f;
    float_data_.qy = static_cast<float>(data_.Qy) * 0.0001f;
    float_data_.qz = static_cast<float>(data_.Qz) * 0.0001f;
    data_ready_ = 1;
    return true;
}

void IMU_Init(void)
{
    hi14.Init();
}

void IMU_UART_RxCallBack(uint8_t rxdata)
{
    hi14.UartRxCallBack(rxdata);
}
