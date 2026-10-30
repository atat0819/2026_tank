#ifndef STRINGWHEEL_HPP
#define STRINGWHEEL_HPP

#include "CalculationBase.hpp"
#include "PlanarWheelMath.hpp"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Alg::CalculationBase
{
    // 单位：几何 m、车速 m/s、轮端转速 rad/s、轮端力矩 N*m；不含减速比/RPM。
    // wheel_azimuth 为轮心位置极角。舵角输入/输出均为编码器 rad，物理轮朝向=编码器角-phase[i]。
    /**
     * @class Omni_FK
     * @brief 舵轮底盘正向运动学计算类
     * 
     * 根据四个舵轮的转速计算底盘的运动状态（速度和角速度）
     * 继承自ForwardKinematicsBase类
     */
    class String_FK : public ForwardKinematicsBase
    {
        public:
            /**
             * @brief 构造函数
             * @param r 轮心到车体中心的相同距离，m
             * @param s 轮子半径
             * @param wheel_azimuth 轮安装方位角 Wheel_Azimuth = {0, M_PI/2, M_PI, 3*M_PI/2}
             */
            // phase 为每轮编码器零位 rad；与 IK 使用同一数组，省略表示全 0。
            String_FK(float r, float s, const float wheel_azimuth[4], const float *phase = nullptr)
                : R(r), S(s), ChassisVx(0.0f), ChassisVy(0.0f), ChassisVw(0.0f) 
            {
                for(int i = 0; i < 4; i++)
                {
                    Wheel_Azimuth[i] = wheel_azimuth[i];
                    current_steer_angles[i] = 0.0f;
                    Phase[i] = phase == nullptr ? 0.0f : phase[i];
                }
            }

            /**
             * @brief 执行正向运动学计算(小心符号问题)
             * 
             * 根据已设置的四个轮子转速，计算底盘在三个自由度上的运动状态
             * 使用舵轮正向运动学公式
             */
            void ForKinematics()
            {
                double normal[3][3] = {}, rhs[3] = {}, velocity[3];
                for (int i = 0; i < 4; i++)
                {
                    // 每轮速度满足 vx_i=vx-y_i*wz、vy_i=vy+x_i*wz；逐次重算，不沿用旧输出。
                    const float angle = current_steer_angles[i] - Phase[i];
                    const double row_x[3] = {1, 0, -sinf(Wheel_Azimuth[i])};
                    const double row_y[3] = {0, 1, cosf(Wheel_Azimuth[i])};
                    WheelMath::Accumulate(row_x, Get_w(i)*S*cosf(angle), normal, rhs);
                    WheelMath::Accumulate(row_y, Get_w(i)*S*sinf(angle), normal, rhs);
                }
                WheelMath::Solve(normal, rhs, velocity);
                ChassisVx = static_cast<float>(velocity[0]);
                ChassisVy = static_cast<float>(velocity[1]);
                ChassisVw = static_cast<float>(velocity[2] / R);
            }

            /**
             * @brief 完整的舵轮正向运动学计算
             * @param w0 轮子0的轮端转速 rad/s
             * @param w1 轮子1的轮端转速 rad/s
             * @param w2 轮子2的轮端转速 rad/s
             * @param w3 轮子3的轮端转速 rad/s
             * 
             * 先设置轮子转速，然后执行正向运动学计算
             * 必须先使用 void Set_current_steer_angles(float angle, int index) 获取舵向电机当前角度
             */            
            void OmniForKinematics(float w0, float w1, float w2, float w3)
            {
                Set_w0w1w2w3(w0, w1, w2, w3);
                ForKinematics();
            }

            /**
             * @brief 设置指定索引的舵向电机当前角度
             * @param angle 舵向电机角度值(弧度)
             * @param index 舵向电机索引(0-3)
             * 
             * 更新舵向电机当前角度数组中指定索引位置的角度值，
             * 用于后续舵向电机就近转位计算
             */
            void Set_current_steer_angles(float angle, int index)
            {
                current_steer_angles[index] = angle;
            }

            /**
             * @brief 获取中心投影点到轮子投影点的距离
             * @return 中心投影点到轮子投影点的距离R
             */
            float GetRadius() const { return R; }

            /**
             * @brief 获取轮子半径 
             * @return 轮子半径S
             */
            float GetScaling() const { return S; }

            /**
             * @brief 获取底盘X方向速度
             * @return X方向速度(前进/后退方向)
             */                  
            float GetChassisVx() const { return ChassisVx; }
            
            /**
             * @brief 获取底盘Y方向速度
             * @return Y方向速度(左移/右移方向)
             */   
            float GetChassisVy() const { return ChassisVy; }

            /**
             * @brief 获取底盘角速度
             * @return 绕Z轴角速度(旋转速度)
             */     
            float GetChassisVw() const { return ChassisVw; }

            /**
             * @brief 获取指定轮的安装方位角
             * @param index 轮索引(0-3)
             * @return 对应轮的安装方位角
             */
            float GetWheel_Azimuth(int index) { return Wheel_Azimuth[index]; }

        private:
            float R;                 // 中心投影点到轮子投影点的距离
            float S;                 // 轮子半径
            float ChassisVx;         // 底盘X方向速度
            float ChassisVy;         // 底盘Y方向速度
            float ChassisVw;         // 底盘绕Z轴角速度
            float current_steer_angles[4] = {0};  // 当前舵向电机角度
            float Phase[4];           // 编码器零位，rad
            float Wheel_Azimuth[4];   // 轮安装方位角（弧度）
    };





    /**
     * @class String_ID
     * @brief 舵轮底盘逆向动力学计算类
     * 
     * 根据底盘受到的外力和力矩计算每个轮子需要产生的扭矩
     * 继承自InverseDynamicsBase类
     */
    class String_ID : public InverseDynamicsBase
    {
        public:
            /**
             * @brief 构造函数
             * @param r 中心投影点到轮子投影点距离
             * @param s 轮子半径
             * @param wheel_azimuth 轮安装方位角(弧度) Wheel_Azimuth = {0, M_PI/2, M_PI, 3*M_PI/2}
             */
            // phase 为每轮编码器零位 rad；与 IK 使用同一数组，省略表示全 0。
            String_ID(float r, float s, const float wheel_azimuth[4], const float *phase = nullptr)
                : R(r), S(s)
            {
                for(int i = 0; i < 4; i++)
                {
                    MotorTorque[i] = 0.0f;
                    Wheel_Azimuth[i] = wheel_azimuth[i];
                    current_steer_angles[i] = 0.0f;
                    Phase[i] = phase == nullptr ? 0.0f : phase[i];
                }
            }

            /**
             * @brief 执行逆向动力学计算(小心符号问题)
             * 
             * 根据已设置的底盘受力情况，计算每个轮子需要产生的扭矩
             * 使用舵轮逆向动力学公式
             */
            void InverseDynamics()
            {
                double rows[4][3], normal[3][3] = {}, allocation[3];
                const double demand[3] = {GetFx(), GetFy(), GetTorque()/R};
                for(int i = 0; i < 4; i++)
                {
                    WheelMath::Row(current_steer_angles[i] - Phase[i], R*cosf(Wheel_Azimuth[i]),
                                   R*sinf(Wheel_Azimuth[i]), R, rows[i]);
                    WheelMath::AddOuter(rows[i], normal);
                }
                // 求最小范数轮缘力；舵角降秩时只保留可产生的合力/转矩分量。
                WheelMath::Solve(normal, demand, allocation);
                for(int i = 0; i < 4; i++)
                    MotorTorque[i] = static_cast<float>(S * (rows[i][0]*allocation[0] +
                                                           rows[i][1]*allocation[1] + rows[i][2]*allocation[2]));
            }

            /**
             * @brief 完整的全向轮逆向动力学计算
             * @param fx X方向力
             * @param fy Y方向力
             * @param torque 绕Z轴力矩
             * 
             * 先设置底盘受力情况，然后执行逆向动力学计算
             * 使用前必须先调用 void Set_current_steer_angles(float angle, int index)
             */
            void OmniInvDynamics(float fx, float fy, float torque)
            {
                Set_FxFyTor(fx, fy, torque);
                InverseDynamics();
            }

            /**
             * @brief 设置指定索引的舵向电机当前角度
             * @param angle 舵向电机角度值(弧度)
             * @param index 舵向电机索引(0-3)
             * 
             * 更新舵向电机当前角度数组中指定索引位置的角度值，
             * 用于后续舵向电机就近转位计算
             */
            void Set_current_steer_angles(float angle, int index)
            {
                current_steer_angles[index] = angle;
            }

            /**
             * @brief 获取指定索引的电机所需扭矩
             * @param index 电机索引(0-3)
             * @return 对应电机的扭矩
             */
            float GetMotorTorque(int index) const 
            { 
                if(index >= 0 && index < 4) 
                {
                    return MotorTorque[index];
                }
                return 0.0f; // 错误情况返回0
            }

            /**
             * @brief 获取指定舵向电机当前角度
             * @param index 舵向电机索引(0-3)
             * @return 对应舵向电机当前角度
             */
            float GetCurrent_steer_angles(int index) { return current_steer_angles[index]; }

            /**
             * @brief 获取指定轮的安装方位角
             * @param index 轮索引(0-3)
             * @return 对应轮的安装方位角
             */
            float GetWheel_Azimuth(int index) { return Wheel_Azimuth[index]; }
        
        private:
            float R;              // 轮子投影点到中心距离
            float S;              // 轮子半径
            float MotorTorque[4]; // 四个电机的扭矩
            float Wheel_Azimuth[4];   // 轮安装方位角（弧度）
            float current_steer_angles[4] = {0};  // 当前舵向电机角度
            float Phase[4];        // 编码器零位，rad
    };





    /**
     * @class String_IK
     * @brief 舵轮底盘逆向运动学计算类
     * 
     * 根据期望的底盘运动状态计算每个轮子的目标速度
     * 继承自InverseKinematicsBase类
     */
    class String_IK : public InverseKinematicsBase
    {
        public:
            /**
             * @brief 构造函数
             * @param r 轮子投影点到中心距离
             * @param s 轮子半径
             * @param wheel_azimuth 轮子安装方位角 Wheel_Azimuth = {0, M_PI/2, M_PI, 3*M_PI/2}
             */
            String_IK(float r, float s, const float wheel_azimuth[4], const float phase[4])
                : R(r), S(s) 
            {
                for(int i = 0; i < 4; i++)
                {
                    Motor_wheel[i] = 0.0f;
                    Motor_direction[i] = 0.0f;
                    Wheel_Azimuth[i] = wheel_azimuth[i];
                    Phase[i] = phase[i];
                }
            }

            
            /**
             * @brief 角度归一化到指定范围内
             * @param angle 待归一化的角度
             * @param tar_angle 角度周期，当前调用为 2*PI
             * @return 归一化后的角度
             */
            float NormalizeAngle(float angle, float tar_angle)
            {
                return std::remainder(angle, tar_angle);
            }


            /**
             * @brief 转向电机就近转位处理
             * 
             * 确保转向电机以最短路径转动到目标角度，避免不必要的整圈转动
             */
            void _Steer_Motor_Kinematics_Nearest_Transposition()
            {
                for (int i = 0; i < 4; i++)
                {
                    float tmp_delta_angle = NormalizeAngle(Motor_direction[i] - current_steer_angles[i], 2.0f * M_PI);

                    // 根据转动角度范围决定是否需要就近转位
                    if (-M_PI / 2.0f <= tmp_delta_angle && tmp_delta_angle <= M_PI / 2.0f)
                    {
                        // ±PI / 2之间无需反向就近转位
                        Motor_direction[i] = tmp_delta_angle + current_steer_angles[i];
                    }
                    else
                    {
                        // 需要反转扣圈情况
                        Motor_direction[i] = NormalizeAngle(tmp_delta_angle + M_PI, 2.0f * M_PI) + current_steer_angles[i];
                        Motor_wheel[i] *= -1.0f;
                    }
                }
            }

            /**
             * @brief 计算考虑增益和相位调整后的速度
             * 
             * 根据设定的目标速度和相位角，计算实际需要的速度分量
             */
            void CalculateVelocities()
            {
                Vx = GetSpeedGain() * (GetSignal_x() *  cosf(GetPhase()) + GetSignal_y() * sinf(GetPhase()));
                Vy = GetSpeedGain() * (GetSignal_x() * -sinf(GetPhase()) + GetSignal_y() * cosf(GetPhase()));
                Vw = GetRotationalGain() * GetSignal_w();
            }

            /**
             * @brief 执行逆向运动学计算(小心符号问题)
             * 
             * 根据已计算的速度分量，计算四个轮子的目标转速，以及舵向电机的方向
             * 使用舵轮逆向运动学公式
             */
            void InvKinematics()
            {   
                for (int i = 0; i < 4; i++)
                {
                    float tmp_velocity_x, tmp_velocity_y, tmp_velocity_modulus;

                    tmp_velocity_x = Vx  - Vw * R * sinf(Wheel_Azimuth[i]);
                    tmp_velocity_y = Vy  + Vw * R * cosf(Wheel_Azimuth[i]);
                    
                    tmp_velocity_modulus = sqrtf(tmp_velocity_x * tmp_velocity_x + tmp_velocity_y * tmp_velocity_y) / S;

                    Motor_wheel[i] = tmp_velocity_modulus; // 轮端 rad/s；减速比和电机 RPM 由调用层处理。

                    // 根据速度的xy分量分别决定舵向电机角度
                    if (tmp_velocity_modulus == 0.0f)
                    {
                        // 零速时保持当前编码器角度，不自动转到切向或锁轮角度。
                        Motor_direction[i] = current_steer_angles[i];
                    }
                    else
                    {
                        // 没有除零问题
                        Motor_direction[i] = atan2f(tmp_velocity_y, tmp_velocity_x) + Phase[i];
                    }
                }
                // 执行就近转位
                _Steer_Motor_Kinematics_Nearest_Transposition();
            }

            /**
             * @brief 完整的舵轮逆向运动学计算，使用前必须先调用 void Set_current_steer_angles(float angle, int index)
             * @param vx X方向速度
             * @param vy Y方向速度
             * @param vw 绕Z轴角速度
             * @param phase 旋转矩阵的角度，需要包含补偿相位
             * @param speed_gain 速度增益
             * 
             * 设置目标运动状态，计算速度分量，然后执行逆向运动学计算
             */
            void StringInvKinematics(float vx, float vy, float vw, float phase, float speed_gain, float rotate_gain)
            {
                SetPhase(phase);
                SetSpeedGain(speed_gain);
                SetRotationalGain(rotate_gain);
                SetSignal_xyw(vx, vy, vw);
                CalculateVelocities(); 
                InvKinematics();
            }

            /**
             * @brief 设置指定索引的舵向电机当前角度
             * @param angle 舵向电机角度值(弧度)
             * @param index 舵向电机索引(0-3)
             * 
             * 更新舵向电机当前角度数组中指定索引位置的角度值，
             * 用于后续舵向电机就近转位计算
             */
            void Set_current_steer_angles(float angle, int index)
            {
                current_steer_angles[index] = angle;
            }

            /**
             * @brief 获取指定索引的轮向电机目标速度
             * @param index 电机索引(0-3)
             * @return 对应轮端的目标速度，rad/s
             */
            float GetMotor_wheel(int index) const 
            { 
                if(index >= 0 && index < 4) 
                {
                    return Motor_wheel[index];
                }
                return 0.0f; // 错误情况返回0
            }
            
            /**
             * @brief 获取指定索引的舵向电机目标角度
             * @param index 舵向电机索引(0-3)
             * @return 对应舵向电机的目标角度
             */
            float GetMotor_direction(int index) const
            { 
                if(index >= 0 && index < 4) 
                {
                    return Motor_direction[index];
                }
                return 0.0f; // 错误情况返回0
            }
            
            /**
             * @brief 获取X方向速度分量
             * @return X方向速度分量
             */
            float GetVx() const { return Vx; }

            /**
             * @brief 获取Y方向速度分量
             * @return Y方向速度分量
             */
            float GetVy() const { return Vy; }
            
            /**
             * @brief 获取角速度分量
             * @return 角速度分量
             */
            float GetVw() const { return Vw; }

            /**
             * @brief 获取指定轮的安装方位角
             * @param index 轮索引(0-3)
             * @return 对应轮的安装方位角
             */
            float GetWheel_Azimuth(int index) { return Wheel_Azimuth[index]; }

            /**
             * @brief 获取指定舵向电机当前角度
             * @param index 舵向电机索引(0-3)
             * @return 对应舵向电机当前角度
             */
            float GetCurrent_steer_angles(int index) { return current_steer_angles[index]; }

        private:
            float Vx{0.0f};       // X方向速度分量
            float Vy{0.0f};       // Y方向速度分量
            float Vw{0.0f};       // 角速度分量
            float R;              // 轮子投影点到中心距离
            float S;              // 轮子半径
            float Motor_wheel[4]; // 轮向电机的目标速度
            float Motor_direction[4];       // 舵向电机编码器目标角度，rad
            float Wheel_Azimuth[4];   // 轮安装方位角（弧度）
            float current_steer_angles[4] = {0};  // 当前舵向电机角度
            float Phase[4]; //初始相位
    };
}

#endif
