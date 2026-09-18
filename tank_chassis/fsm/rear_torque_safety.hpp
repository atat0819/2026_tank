#ifndef REAR_TORQUE_SAFETY_HPP
#define REAR_TORQUE_SAFETY_HPP

namespace StairTorqueSafety
{
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
