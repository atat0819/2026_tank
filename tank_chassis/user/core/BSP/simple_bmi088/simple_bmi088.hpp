#ifndef USER_CORE_BSP_SIMPLE_BMI088_HPP
#define USER_CORE_BSP_SIMPLE_BMI088_HPP

extern "C" {
#include "simple_bmi088.h"
}

namespace BSP
{
namespace simplebmi088
{
struct Vector3f
{
    float x;
    float y;
    float z;
};

class BMI088
{
public:
    BMI088_Data data = {};

    HAL_StatusTypeDef Read()
    {
        const HAL_StatusTypeDef status = BMI088_Read();
        SynchronizeData();
        return status;
    }

    void RefreshData()
    {
        SynchronizeData();
    }

    bool IsReady() const
    {
        return data.initialized != 0U && data.euler_valid != 0U;
    }

    bool IsGyroCalibrated() const
    {
        return data.gyro_calibrated != 0U;
    }

    /* ---------------- 欧拉角 (deg): roll / pitch / yaw ---------------- */
    float GetRollAngleDeg() const
    {
        return data.euler_deg[0];
    }

    float GetPitchAngleDeg() const
    {
        return data.euler_deg[1];
    }

    float GetYawAngleDeg() const
    {
        return data.euler_deg[2];
    }

    /* ---------------- 陀螺角速度 (deg/s, 已减零偏) ---------------- */
    float GetGyroRateXDps() const
    {
        return data.gyro_dps[0] - data.gyro_bias_dps[0];
    }

    float GetGyroRateYDps() const
    {
        return data.gyro_dps[1] - data.gyro_bias_dps[1];
    }

    float GetGyroRateZDps() const
    {
        return data.gyro_dps[2] - data.gyro_bias_dps[2];
    }

    /* ---------------- 角加速度 (deg/s²) ---------------- */
    float GetAngularAccelXDps2() const
    {
        return data.angular_accel_dps2[0];
    }

    float GetAngularAccelYDps2() const
    {
        return data.angular_accel_dps2[1];
    }

    float GetAngularAccelZDps2() const
    {
        return data.angular_accel_dps2[2];
    }

    /* ---------------- 加速度 (m/s²) ---------------- */
    float GetAccelXMps2() const
    {
        return data.accel_mps2[0];
    }

    float GetAccelYMps2() const
    {
        return data.accel_mps2[1];
    }

    float GetAccelZMps2() const
    {
        return data.accel_mps2[2];
    }

    /* 合加速度模长 (m/s²), 静止时约等于 9.8 */
    float GetAccelNormMps2() const
    {
        return data.accel_norm_mps2;
    }

    /* ---------------- 四元数 (w, x, y, z; 单位模长) ---------------- */
    float GetQuaternionW() const
    {
        return data.quaternion[0];
    }

    float GetQuaternionX() const
    {
        return data.quaternion[1];
    }

    float GetQuaternionY() const
    {
        return data.quaternion[2];
    }

    float GetQuaternionZ() const
    {
        return data.quaternion[3];
    }

    /* ---------------- 批量取整向量的旧接口 (当前无人调用) ---------------- */
    Vector3f GetEulerAngleDeg() const
    {
        return {data.euler_deg[0], data.euler_deg[1], data.euler_deg[2]};
    }

    Vector3f GetAngularVelocityDps() const
    {
        return {data.gyro_dps[0] - data.gyro_bias_dps[0],
                data.gyro_dps[1] - data.gyro_bias_dps[1],
                data.gyro_dps[2] - data.gyro_bias_dps[2]};
    }

    Vector3f GetAngularAccelerationDps2() const
    {
        return {data.angular_accel_dps2[0],
                data.angular_accel_dps2[1],
                data.angular_accel_dps2[2]};
    }

    Vector3f GetAccelerationMps2() const
    {
        return {data.accel_mps2[0], data.accel_mps2[1], data.accel_mps2[2]};
    }

private:
    void SynchronizeData()
    {
        for (uint32_t axis = 0U; axis < 3U; ++axis)
        {
            data.accel_mps2[axis] = g_bmi088.accel_mps2[axis];
            data.gyro_dps[axis] = g_bmi088.gyro_dps[axis];
            data.angular_accel_dps2[axis] = g_bmi088.angular_accel_dps2[axis];
            data.gyro_bias_dps[axis] = g_bmi088.gyro_bias_dps[axis];
            data.euler_deg[axis] = g_bmi088.euler_deg[axis];
        }

        for (uint32_t element = 0U; element < 4U; ++element)
        {
            data.quaternion[element] = g_bmi088.quaternion[element];
        }

        data.accel_norm_mps2 = g_bmi088.accel_norm_mps2;
        data.accel_chip_id = g_bmi088.accel_chip_id;
        data.gyro_chip_id = g_bmi088.gyro_chip_id;
        data.initialized = g_bmi088.initialized;
        data.euler_valid = g_bmi088.euler_valid;
        data.gyro_calibrated = g_bmi088.gyro_calibrated;
        data.init_stage = g_bmi088.init_stage;
        data.init_status = g_bmi088.init_status;
        data.accel_spi_status = g_bmi088.accel_spi_status;
        data.gyro_spi_status = g_bmi088.gyro_spi_status;
        data.update_count = g_bmi088.update_count;
        data.error_count = g_bmi088.error_count;
    }
};

inline BMI088 &GetOnboardBMI088()
{
    static BMI088 imu;
    return imu;
}
} // namespace simplebmi088
} // namespace BSP

#endif
