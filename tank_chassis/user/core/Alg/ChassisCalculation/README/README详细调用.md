# 全向轮与舵轮调用示例

两个示例均使用轮端 rad/s、舵角 rad、车体速度 m/s/rad/s、轮端力矩 N*m。以下几何只是演示值，不能替代实车尺寸和标定。原接口名保留，轮速与力矩到电机命令的换算在调用层完成。

## 全向轮

`azimuth` 是驱动方向，`position_angle` 是轮心极角。三轮时给 FK/ID 的有效轮数传 3，使用数组前三项，并相应配置为满秩的三轮布局。

```cpp
#include "user/core/Alg/ChassisCalculation/OmniCalculation.hpp"

int main()
{
    using namespace Alg::CalculationBase;
    const float pi = 3.14159265358979323846f;
    const float center_radius = .4f, wheel_radius = .05f;
    float position_angle[4] = {0,pi/2,pi,3*pi/2};
    float azimuth[4], coordinates[4][2];
    for (int i = 0; i < 4; ++i)
    {
        azimuth[i] = position_angle[i] + pi/2;
        coordinates[i][0] = center_radius*cosf(position_angle[i]);
        coordinates[i][1] = center_radius*sinf(position_angle[i]);
    }
    Omni_IK ik(center_radius,wheel_radius,azimuth,coordinates);
    Omni_FK fk(center_radius,wheel_radius,4,azimuth,position_angle);
    Omni_ID id(center_radius,wheel_radius,4,azimuth,coordinates);

    // 输入已是 m/s、rad/s，故两个倍率均为 1；坐标变换角为 0。
    ik.OmniInvKinematics(.5f,.2f,.3f,0,1,1);
    float wheel_target[4];
    for (int i = 0; i < 4; ++i) wheel_target[i] = ik.GetMotor(i);

    // 实车必须使用实际反馈；此数组仅表示电机已停止时的一次采样。
    float measured_wheel_rad_s[4] = {};
    fk.OmniForKinematics(measured_wheel_rad_s[0],measured_wheel_rad_s[1],
                        measured_wheel_rad_s[2],measured_wheel_rad_s[3]);
    id.OmniInvDynamics(4,-3,2); // Fx=4 N、Fy=-3 N、Mz=2 N*m
    float wheel_torque[4];
    for (int i = 0; i < 4; ++i) wheel_torque[i] = id.GetMotorTorque(i);
    // wheel_target 供轮速控制；wheel_torque 供力控，经电流/功率限制后输出。
    (void)wheel_target;
    (void)wheel_torque;
}
```

## 舵轮

IK、FK、ID 必须传同一组轮心位置和编码器零位。实际轮速需要预先转换到轮端 rad/s；原始舵角必须是 rad，FK/ID 内部会减零位。

```cpp
#include "user/core/Alg/ChassisCalculation/StringWheel.hpp"

int main()
{
    using namespace Alg::CalculationBase;
    const float pi = 3.14159265358979323846f;
    float position_angle[4] = {pi/4,3*pi/4,5*pi/4,7*pi/4};
    float phase_offset[4] = {-1.3f,3.44f,2.85f,1.33f};
    String_IK ik(.17f,.055f,position_angle,phase_offset);
    String_FK fk(.17f,.055f,position_angle,phase_offset);
    String_ID id(.17f,.055f,position_angle,phase_offset);

    // 实车替换为本周期实际反馈；示例是零轮速、物理朝向均为 0。
    float measured_wheel_rad_s[4] = {};
    float measured_encoder_rad[4] = {-1.3f,3.44f,2.85f,1.33f};
    for (int i = 0; i < 4; ++i)
    {
        ik.Set_current_steer_angles(measured_encoder_rad[i],i);
        fk.Set_current_steer_angles(measured_encoder_rad[i],i);
        id.Set_current_steer_angles(measured_encoder_rad[i],i);
    }
    ik.StringInvKinematics(.5f,.2f,.3f,0,1,1);
    fk.OmniForKinematics(measured_wheel_rad_s[0],measured_wheel_rad_s[1],
                        measured_wheel_rad_s[2],measured_wheel_rad_s[3]);
    // 当前朝向均为 X，Fx 可实现；此时不能要求任意 Fy。
    id.OmniInvDynamics(4,0,0);
    for (int i = 0; i < 4; ++i)
    {
        float steer_target_encoder_rad = ik.GetMotor_direction(i);
        float wheel_target_rad_s = ik.GetMotor_wheel(i);
        float wheel_target_torque_nm = id.GetMotorTorque(i);
        // 舵向目标交给舵向控制；速控用轮速目标，力控用轮端力矩目标。
        // 不在此叠加未经设计的轮速 PID 和力控输出。
        (void)steer_target_encoder_rad;
        (void)wheel_target_rad_s;
        (void)wheel_target_torque_nm;
    }
}
```

舵向环若使用角度制，统一在交给舵向控制器前将 rad 转为 deg；轮速环若比较转子 RPM，则将轮端目标乘总减速比并换算 RPM。不要再次给 IK 输出乘旧的固定 `40`。

这两套解算不会自行判断电机在线、限制电流或功率。上车前应独立核对方向、编码器零位、减速比和力矩常数；几何模型及主机测试不能证明这些实车量正确。
