#ifndef MECANUM_FORCE_CONTROL_HPP
#define MECANUM_FORCE_CONTROL_HPP

#include <algorithm>
#include <cmath>

namespace Alg::CalculationBase
{
    // 13 kg 平动惯性前馈 + 四轮速度 PID -> 合成电流 -> 统一限流。
    // 四轮顺序为 ID1 左前、ID2 右前、ID3 右后、ID4 左后，方向沿用原操作约定。
    class MecanumForceControl
    {
    public:
        // 车轮半径，m，用于轮速/车速和轮缘力/轮端力矩换算。
        static constexpr float WHEEL_RADIUS = 0.076f;
        // 前后半轮距 + 左右半轮距，m，用于旋转速度与转矩分配。
        static constexpr float LEVER_ARM = 0.215f + 0.243f;
        // 转子到车轮的总减速比；转子角速度除以此值才是轮端角速度。
        static constexpr float REDUCTION_RATIO = 268.0f / 17.0f;
        // 四轮 PID 使用的控制周期，s。
        static constexpr float CONTROL_PERIOD = 0.001f;
        static constexpr float MASS_KG = 14.0f;
        // 名义轮端力矩系数，N*m/A；0.3 是原 19:1 减速箱输出侧系数，未计实际传动效率。
        static constexpr float WHEEL_TORQUE_PER_AMP = 0.3f / 19.0f * REDUCTION_RATIO;
        // 电流命令换算系数，raw/A：20 A 对应 16384。
        static constexpr float RAW_PER_AMP = 16384.0f / 20.0f;
        // 单轮电流上限，A；超限时四轮统一缩放，保留力分配比例。
        static constexpr float MAX_CURRENT = 20.0f;

        // 四轮速度反馈单位为轮端 rad/s，PID 输出 A；以下为待实车整定的初值。
        float kp[4] = {1.0f, 1.0f, 1.0f, 1.0f};       // A/(rad/s)
        float ki[4] = {0.5f, 0.5f, 0.5f, 0.5f};       // A/rad
        float kd[4] = {};                              // A/(rad/s^2)
        float integral_limit[4] = {2.0f, 2.0f, 2.0f, 2.0f}; // 积分输出上限 A
        float integral[4] = {};
        // 本周期平动惯性前馈 Fx、Fy、Mz，N、N、N*m；旋转惯性前馈固定为 0。
        float force[3] = {};
        // 本周期限幅前的两路电流请求，A；摩擦前馈未启用。
        float feedforward_current[4] = {};
        float feedback_current[4] = {};
        // 电流限幅与后续功率限幅的合成电流缩放比例。
        float output_scale = 1.0f;

        // 逆运动学：[vx,vy,wz]（m/s、m/s、rad/s）-> 四轮轮端角速度（rad/s）。
        // 用于四轮速度 PID 的目标轮速。
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

        // target/feedback 均为四轮轮端 rad/s；acceleration 为规划 X/Y 加速度 m/s^2。
        // 惯性前馈与轮速 PID 电流相加后统一限流，不存在独立绕过限幅的前馈通道。
        // 本函数完成统一电流限幅；在线判断、总功率限制和 CAN 发送由任务层负责。
        void Update(const float target[4], const float acceleration[2],
                    const float feedback[4], float raw_output[4])
        {
            force[0] = MASS_KG * acceleration[0];
            force[1] = MASS_KG * acceleration[1];
            force[2] = 0.0f;
            float torque[4];
            InverseDynamics(force, torque);
            // 暂存电流 A，并找到限流所需的峰值；未超限时不会放大电流。
            float peak_current = MAX_CURRENT;
            for (unsigned i = 0; i < 4; ++i)
            {
                error_[i] = target[i] - feedback[i];
                previous_integral_[i] = integral[i];
                integral[i] = std::clamp(integral[i] + ki[i] * error_[i] * CONTROL_PERIOD,
                                         -integral_limit[i], integral_limit[i]);
                feedforward_current[i] = torque[i] / WHEEL_TORQUE_PER_AMP;
                feedback_current[i] = kp[i] * error_[i] + integral[i] +
                    kd[i] * (error_[i] - previous_error_[i]) / CONTROL_PERIOD;
                previous_error_[i] = error_[i];
                requested_current_[i] = feedforward_current[i] + feedback_current[i];
                raw_output[i] = requested_current_[i];
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
                for (unsigned i = 0; i < 4; ++i)
                {
                    if (error_[i] * requested_current_[i] > 0.0f)
                    {
                        integral[i] = previous_integral_[i];
                    }
                }
            }
        }

        // 清除积分、误差和力请求历史，保留增益与限幅；任务层另行清零输出并发送 CAN。
        void Reset()
        {
            for (float &value : force) value = 0.0f;
            for (unsigned i = 0; i < 4; ++i)
            {
                integral[i] = error_[i] = previous_error_[i] = previous_integral_[i] = 0.0f;
                feedforward_current[i] = feedback_current[i] = requested_current_[i] = 0.0f;
            }
            output_scale = 1.0f;
        }

    private:
        // 正 X、正 Y 运动对应的四轮电机方向；Y 正向沿用原侧移指令，与旧几何 Y 方向相反。
        static constexpr float DIRECTION_X[4] = {-1.0f, 1.0f, 1.0f, -1.0f};
        static constexpr float DIRECTION_Y[4] = {-1.0f, -1.0f, 1.0f, 1.0f};
        float error_[4] = {};
        float previous_error_[4] = {};
        // 本次积分更新前的 A 快照，限流/限功率时用于回退。
        float previous_integral_[4] = {};
        float requested_current_[4] = {};
    };
}

#endif
