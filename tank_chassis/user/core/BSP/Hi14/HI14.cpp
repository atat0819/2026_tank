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
}

void HI14::UartRxCallBack(uint8_t rxdata)
{
    switch (handle_.Step)
    {
        case 0:
            if (rxdata == 0xA5)
            {
                handle_.Step = 1;
                handle_.Index = 0;
            }
            break;

        case 1:
            if (handle_.Index < IMU_DATA_LEN)
            {
                handle_.RxBuffer[handle_.Index++] = rxdata;
            }

            if (handle_.Index >= IMU_DATA_LEN)
            {
                if (handle_.RxBuffer[IMU_CMD_POS] == IMU_EXPECTED_CMD &&
                    handle_.RxBuffer[IMU_TAIL1_POS] == 0x5A &&
                    handle_.RxBuffer[IMU_TAIL2_POS] == 0xA5)
                {
                    memcpy(&data_.Roll, &handle_.RxBuffer[32], 4);
                    memcpy(&data_.Pitch, &handle_.RxBuffer[36], 4);
                    memcpy(&data_.Yaw, &handle_.RxBuffer[40], 4);
                    memcpy(&data_.GyrPitch, &handle_.RxBuffer[14], 2);
                    memcpy(&data_.GyrRoll, &handle_.RxBuffer[16], 2);
                    memcpy(&data_.GyrYaw, &handle_.RxBuffer[18], 2);
                    memcpy(&data_.Qw, &handle_.RxBuffer[44], 2);
                    memcpy(&data_.Qx, &handle_.RxBuffer[46], 2);
                    memcpy(&data_.Qy, &handle_.RxBuffer[48], 2);
                    memcpy(&data_.Qz, &handle_.RxBuffer[50], 2);

                    float_data_.roll = static_cast<float>(data_.Roll) / 87.890625f;
                    float_data_.pitch = static_cast<float>(data_.Pitch) / 87.890625f;
                    float_data_.yaw = static_cast<float>(data_.Yaw) / 87.890625f;
                    float_data_.gyr_pitch = static_cast<float>(data_.GyrPitch) * 0.0275f;
                    float_data_.gyr_roll = static_cast<float>(data_.GyrRoll) * 0.0275f;
                    float_data_.gyr_yaw = static_cast<float>(data_.GyrYaw) * 0.0275f;
                    float_data_.qw = static_cast<float>(data_.Qw) * 0.0001f;
                    float_data_.qx = static_cast<float>(data_.Qx) * 0.0001f;
                    float_data_.qy = static_cast<float>(data_.Qy) * 0.0001f;
                    float_data_.qz = static_cast<float>(data_.Qz) * 0.0001f;

                    data_ready_ = 1;
                }
                else
                {
                    handle_.ErrCount++;
                    if (handle_.ErrCount >= 20)
                    {
                        handle_.Step = 0;
                        handle_.ErrCount = 0;
                    }
                }

                handle_.Step = 0;
            }
            break;

        default:
            handle_.Step = 0;
            break;
    }
}

void IMU_Init(void)
{
    hi14.Init();
}

void IMU_UART_RxCallBack(uint8_t rxdata)
{
    hi14.UartRxCallBack(rxdata);
}
