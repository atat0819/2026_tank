#ifndef REAR_TORQUE_SAFETY_HPP
#define REAR_TORQUE_SAFETY_HPP

namespace StairTorqueSafety
{
// J6248 的应用层最大力矩。此处先限制混控后的结果，避免进入
// 达妙协议编码器之前发生超范围编码；驱动层还有最终通用钳位。
static constexpr float J6248_TORQUE_LIMIT_NM = 40.0f;

inline float ClampJ6248Torque(float torque_nm)
{
    if (torque_nm > J6248_TORQUE_LIMIT_NM)
    {
        return J6248_TORQUE_LIMIT_NM;
    }
    if (torque_nm < -J6248_TORQUE_LIMIT_NM)
    {
        return -J6248_TORQUE_LIMIT_NM;
    }
    return torque_nm;
}
}  // namespace StairTorqueSafety

#endif  // REAR_TORQUE_SAFETY_HPP
