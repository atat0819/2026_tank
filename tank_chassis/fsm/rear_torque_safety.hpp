#ifndef REAR_TORQUE_SAFETY_HPP
#define REAR_TORQUE_SAFETY_HPP

namespace StairTorqueSafety
{
// J6248 的应用层最大力矩，限制状态机增益修正后的最终输出。
// 达妙协议量程保持 ±97 N·m；驱动层还有通用钳位。
static constexpr float J6248_TORQUE_LIMIT_NM = 30.0f;

inline float ClampJ6248Torque(float torque_nm)
{
    // 对调平/收腿的最终力矩执行应用层饱和。
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
