#ifndef HI14_HPP
#define HI14_HPP

#include <stdint.h>

#define IMU_DATA_LEN         54
#define IMU_CMD_POS          6
#define IMU_EXPECTED_CMD     0x92

typedef struct {
    int32_t Roll;
    int32_t Pitch;
    int32_t Yaw;
    int16_t GyrPitch;
    int16_t GyrRoll;
    int16_t GyrYaw;
    int16_t Qw, Qx, Qy, Qz;
} IMU_Data_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float gyr_roll;
    float gyr_pitch;
    float gyr_yaw;
    float qw, qx, qy, qz;
} IMU_Float_t;

typedef struct {
    uint8_t  Step;
    uint8_t  RxBuffer[IMU_DATA_LEN];
    uint8_t  Index;
    uint32_t ErrCount;
} IMU_Handle_t;

class HI14
{
public:
    HI14(IMU_Handle_t &handle, IMU_Data_t &data, IMU_Float_t &float_data,
         volatile uint8_t &data_ready);

    void Init();
    bool UartRxCallBack(uint8_t rxdata); // 仅本字节完成有效帧时返回 true。

    // 最新有效帧的单个量；控制使用完整的 ImuControlSnapshot 检查有效性和时戳。
    float GetRollAngleDeg() const { return float_data_.roll; }
    float GetPitchAngleDeg() const { return float_data_.pitch; }
    float GetYawAngleDeg() const { return float_data_.yaw; }
    float GetRollRateDps() const { return float_data_.gyr_roll; }
    float GetPitchRateDps() const { return float_data_.gyr_pitch; }
    float GetYawRateDps() const { return float_data_.gyr_yaw; }

private:
    IMU_Handle_t &handle_;
    IMU_Data_t &data_;
    IMU_Float_t &float_data_;
    volatile uint8_t &data_ready_;
};

// 全局变量（在 HI14.cpp 中定义）
extern IMU_Handle_t hImu;
extern HI14 hi14;
extern IMU_Data_t   ImuData;
extern IMU_Float_t  ImuFloat;
extern volatile uint8_t ImuDataReady;  // IMU 是否已收到过有效数据帧

// 函数声明
void IMU_UART_RxCallBack(uint8_t rxdata);
void IMU_Init(void);   // 初始化状态机

#endif
