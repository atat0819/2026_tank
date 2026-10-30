#ifndef MECANUM_FORCE_CONTROL_HPP
#define MECANUM_FORCE_CONTROL_HPP

#include <algorithm>
#include <cmath>

namespace Alg::CalculationBase
{
    // 车体速度 PI -> 合力/转矩 -> 四轮电流；三轴顺序为 X、Y、旋转。
    // 四轮顺序为 ID1 左前、ID2 右前、ID3 右后、ID4 左后，方向沿用原操作约定。
    class MecanumForceControl
    {
    public:
        // 车轮半径，m，用于轮速/车速和轮缘力/轮端力矩换算。
        static constexpr float WHEEL_RADIUS = 0.076f;
        // 前后半轮距 + 左右半轮距，m，用于旋转速度与转矩分配。
        static constexpr float LEVER_ARM = 0.215f + 0.243f;
        // 转子到车轮的总减速比；转子角速度除以此值才是轮端角速度。
        static constexpr float REDUCTION_RATIO = 15.8f;
        // PI 积分使用的控制周期，s。
        static constexpr float CONTROL_PERIOD = 0.001f;
        // 名义轮端力矩系数，N*m/A；0.3 是原 19:1 减速箱输出侧系数，未计实际传动效率。
        static constexpr float WHEEL_TORQUE_PER_AMP = 0.3f / 19.0f * REDUCTION_RATIO;
        // 电流命令换算系数，raw/A：20 A 对应 16384。
        static constexpr float RAW_PER_AMP = 16384.0f / 20.0f;
        // 单轮电流上限，A；超限时四轮统一缩放，保留力分配比例。
        static constexpr float MAX_CURRENT = 20.0f;

        // 比例增益：速度误差越大，请求作用力越大；单位依次为 N/(m/s)、N/(m/s)、N*m/(rad/s)。
        float kp[3] = {420.0f, 420.0f, 65.0f};
        // 积分增益：累积速度误差以消除稳态偏差；单位依次为 N/m、N/m、N*m/rad。
        float ki[3] = {3.0f, 3.0f, 0.0f};
        // 积分输出绝对值上限，N、N、N*m；须非负，0 表示该轴不产生积分输出。
        float integral_limit[3] = {10.0f, 10.0f, 1.0f};
        // 已乘入 ki 的积分输出状态，N、N、N*m。
        float integral[3] = {};
        // 本周期限幅前的 Fx、Fy、Mz 请求，N、N、N*m，不是实测作用力。
        float force[3] = {};
        // 电流限幅与后续功率限幅的合成电流缩放比例。
        float output_scale = 1.0f;

        // 逆运动学：[vx,vy,wz]（m/s、m/s、rad/s）-> 四轮轮端角速度（rad/s）。
        // 当前任务只用结果观察目标轮速，不再经过四个轮速 PID。
        static void InverseKinematics(const float velocity[3], float wheel_speed[4])
        {
            for (unsigned i = 0; i < 4; ++i)
            {
                wheel_speed[i] = (DIRECTION_X[i] * velocity[0] +
                    DIRECTION_Y[i] * velocity[1] + LEVER_ARM * velocity[2]) / WHEEL_RADIUS;
            }
        }

        // 正运动学：四轮轮端角速度（已除减速比，rad/s）-> 车体速度（m/s、m/s、rad/s）。
        // 轮速估计依赖滚动条件，打滑或悬空时不等于真实车速。
        static void ForwardKinematics(const float wheel_speed[4], float velocity[3])
        {
            velocity[0] = velocity[1] = velocity[2] = 0.0f;
            for (unsigned i = 0; i < 4; ++i)
            {
                velocity[0] += DIRECTION_X[i] * wheel_speed[i] * WHEEL_RADIUS / 4.0f;
                velocity[1] += DIRECTION_Y[i] * wheel_speed[i] * WHEEL_RADIUS / 4.0f;
                velocity[2] += wheel_speed[i] * WHEEL_RADIUS / (4.0f * LEVER_ARM);
            }
        }

        // 逆动力学分配：[Fx,Fy,Mz]（N、N、N*m）-> 四轮轮端力矩（N*m）。
        // 输出已包含电机安装方向；本函数不乘质量或惯量，也不执行功率限制。
        static void InverseDynamics(const float body_force[3], float wheel_torque[4])
        {
            for (unsigned i = 0; i < 4; ++i)
            {
                wheel_torque[i] = WHEEL_RADIUS / 4.0f * (DIRECTION_X[i] * body_force[0] +
                    DIRECTION_Y[i] * body_force[1] + body_force[2] / LEVER_ARM);
            }
        }

        // 执行三轴速度 PI 和力分配；target/feedback 为同坐标车速，raw_output 为四轮电流命令。
        // 本函数完成统一电流限幅；在线判断、总功率限制和 CAN 发送由任务层负责。
        void Update(const float target[3], const float feedback[3], float raw_output[4])
        {
            for (unsigned axis = 0; axis < 3; ++axis)
            {
                error_[axis] = target[axis] - feedback[axis];
                previous_integral_[axis] = integral[axis];
                integral[axis] = std::clamp(integral[axis] + ki[axis] * error_[axis] * CONTROL_PERIOD,
                                            -integral_limit[axis], integral_limit[axis]);
                force[axis] = kp[axis] * error_[axis] + integral[axis];
            }
            float torque[4];
            InverseDynamics(force, torque);
            // 暂存电流 A，并找到限流所需的峰值；未超限时不会放大电流。
            float peak_current = MAX_CURRENT;
            for (unsigned i = 0; i < 4; ++i)
            {
                raw_output[i] = torque[i] / WHEEL_TORQUE_PER_AMP;
                peak_current = std::fmax(peak_current, std::fabs(raw_output[i]));
            }
            const float current_scale = MAX_CURRENT / peak_current;
            for (unsigned i = 0; i < 4; ++i)
            {
                // 四轮统一限流，再将 A 换算成 raw。
                raw_output[i] *= current_scale * RAW_PER_AMP;
            }
            output_scale = 1.0f;
            ApplyOutputScale(current_scale);
        }

        // 反馈本级电流缩放比例 scale（0~1），累计比例并回退继续推向饱和的积分。
        // 任务层限功率后传入 eta；此函数不重新缩放输出，反向误差仍可释放积分。
        void ApplyOutputScale(float scale)
        {
            output_scale *= scale;
            if (scale < 1.0f)
            {
                for (unsigned axis = 0; axis < 3; ++axis)
                {
                    if (error_[axis] * force[axis] > 0.0f)
                    {
                        integral[axis] = previous_integral_[axis];
                    }
                }
            }
        }

        // 清除积分、误差和力请求历史，保留增益与限幅；任务层另行清零输出并发送 CAN。
        void Reset()
        {
            for (unsigned axis = 0; axis < 3; ++axis)
            {
                integral[axis] = force[axis] = error_[axis] = previous_integral_[axis] = 0.0f;
            }
            output_scale = 1.0f;
        }

    private:
        // 正 X、正 Y 运动对应的四轮电机方向；Y 正向沿用原侧移指令，与旧几何 Y 方向相反。
        static constexpr float DIRECTION_X[4] = {-1.0f, 1.0f, 1.0f, -1.0f};
        static constexpr float DIRECTION_Y[4] = {-1.0f, -1.0f, 1.0f, 1.0f};
        // 最近一次速度误差，m/s、m/s、rad/s，用于 PI 和抗积分饱和判断。
        float error_[3] = {};
        // 本次积分更新前的输出快照，N、N、N*m，限幅时用于回退。
        float previous_integral_[3] = {};
    };
}

#endif
