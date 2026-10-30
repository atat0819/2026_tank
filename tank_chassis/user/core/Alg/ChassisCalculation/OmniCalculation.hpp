#ifndef OmniCalculation_HPP
#define OmniCalculation_HPP

#include "CalculationBase.hpp"
#include "PlanarWheelMath.hpp"
#include <math.h>

namespace Alg::CalculationBase
{
    // 单位：几何 m、角度 rad、车速 m/s、角速度和轮端转速 rad/s、轮端力矩 N*m。
    // 车体 X/Y 为右手坐标，正角度/正角速度为逆时针；不含减速比和 RPM 换算。
    /**
     * @class Omni_FK
     * @brief 全向轮底盘正向运动学计算类
     */
    class Omni_FK : public ForwardKinematicsBase
    {
        public:
            // r: 轮心到中心的相同距离；s: 轮半径；n: 有效轮数 3 或 4。
            // wheel_azimuth: 各轮正转驱动方向；wheel_direction: 各轮心位置极角，均为 rad。
            Omni_FK(float r, float s, float n, const float wheel_azimuth[4], const float wheel_direction[4]) 
                : R(r), S(s), N(n), ChassisVx(0.0f), ChassisVy(0.0f), ChassisVw(0.0f) 
            {
                for(int i = 0; i < 4; i++)
                {
                    Wheel_Azimuth[i] = wheel_azimuth[i];
                    Wheel_Direction[i] = wheel_direction[i];
                }
            }

            void ForKinematics()
            {
                double normal[3][3] = {}, rhs[3] = {}, velocity[3];
                for(int i = 0; i < static_cast<int>(N); i++)
                {
                    double row[3];
                    WheelMath::Row(Wheel_Azimuth[i], R*cosf(Wheel_Direction[i]),
                                   R*sinf(Wheel_Direction[i]), R, row);
                    WheelMath::Accumulate(row, S*Get_w(i), normal, rhs);
                }
                WheelMath::Solve(normal, rhs, velocity);
                ChassisVx = static_cast<float>(velocity[0]);
                ChassisVy = static_cast<float>(velocity[1]);
                ChassisVw = static_cast<float>(velocity[2] / R);
            }

            void OmniForKinematics(float w0, float w1, float w2, float w3)  //传入4个轮子的角速度
            {
                Set_w0w1w2w3(w0, w1, w2, w3);
                ForKinematics();
            }

            float GetChassisVx() const { return ChassisVx; }
            float GetChassisVy() const { return ChassisVy; }
            float GetChassisVw() const { return ChassisVw; }

        private:
            float R, S, N;
            float ChassisVx, ChassisVy, ChassisVw;
            float Wheel_Azimuth[4];   // 修复：去掉 const，以便在构造函数中赋值
            float Wheel_Direction[4]; 
    };

    /**
     * @class Omni_ID
     * @brief 全向轮底盘逆向动力学计算类
     */
    class Omni_ID : public InverseDynamicsBase
    {
        public:
            Omni_ID(float r, float s, float n, const float wheel_azimuth[4], const float wheel_coordinate[4][2]) 
                : R(r), S(s), N(n)
            {
                for(int i = 0; i < 4; i++)
                {
                    MotorTorque[i] = 0.0f;
                    Wheel_Azimuth[i] = wheel_azimuth[i]; // 修复：统一拼写为 Wheel_Azimuth
                    Wheel_Coordinates[i][0] = wheel_coordinate[i][0];
                    Wheel_Coordinates[i][1] = wheel_coordinate[i][1];
                }
            }

            void InverseDynamics()
            {
                double rows[4][3] = {}, normal[3][3] = {}, allocation[3];
                const double demand[3] = {GetFx(), GetFy(), GetTorque()/R};
                for(int i = 0; i < static_cast<int>(N); i++)
                {
                    WheelMath::Row(Wheel_Azimuth[i], Wheel_Coordinates[i][0], Wheel_Coordinates[i][1], R, rows[i]);
                    WheelMath::AddOuter(rows[i], normal);
                }
                WheelMath::Solve(normal, demand, allocation);
                for(int i = 0; i < 4; i++)
                    MotorTorque[i] = static_cast<float>(S * (rows[i][0]*allocation[0] +
                                                           rows[i][1]*allocation[1] + rows[i][2]*allocation[2]));
            }

            void OmniInvDynamics(float fx, float fy, float torque)
            {
                Set_FxFyTor(fx, fy, torque);
                InverseDynamics();
            }

            float GetMotorTorque(int index) const { return (index >= 0 && index < 4) ? MotorTorque[index] : 0.0f; }

        private:
            float R, S, N;
            float MotorTorque[4];
            float Wheel_Azimuth[4];      // 修复：去掉 const
            float Wheel_Coordinates[4][2]; 
    };

    /**
     * @class Omni_IK
     * @brief 全向轮底盘逆向运动学计算类
     */
    class Omni_IK : public InverseKinematicsBase
    {
        public:
            Omni_IK(float r, float s, const float wheel_azimuth[4], const float wheel_coordinate[4][2]) 
                : R(r), S(s) 
            {
                for(int i = 0; i < 4; i++)
                {
                    Motor[i] = 0.0f;
                    Wheel_Azimuth[i] = wheel_azimuth[i]; // 修复：统一拼写
                    Wheel_Coordinates[i][0] = wheel_coordinate[i][0];
                    Wheel_Coordinates[i][1] = wheel_coordinate[i][1];
                }
            }

            void CalculateVelocities()
            {
                Vx = GetSpeedGain() * (GetSignal_x() * cosf(GetPhase()) + GetSignal_y() * sinf(GetPhase()));
                Vy = GetSpeedGain() * (GetSignal_x() * -sinf(GetPhase()) + GetSignal_y() * cosf(GetPhase()));
                Vw = GetRotationalGain() * GetSignal_w();
            }

            void InvKinematics()
            {   
                for(int i = 0; i < 4; i++)
                {
                    double row[3];
                    WheelMath::Row(Wheel_Azimuth[i], Wheel_Coordinates[i][0], Wheel_Coordinates[i][1], R, row);
                    Motor[i] = static_cast<float>((row[0]*Vx + row[1]*Vy + row[2]*R*Vw) / S);
                }
            }

            void OmniInvKinematics(float vx, float vy, float vw, float phase, float speed_gain, float rotate_gain)
            {
                SetPhase(phase);
                SetSpeedGain(speed_gain);
                SetRotationalGain(rotate_gain);
                SetSignal_xyw(vx, vy, vw);
                CalculateVelocities(); 
                InvKinematics();
            }

            float GetMotor(int index) const { return (index >= 0 && index < 4) ? Motor[index] : 0.0f; }

        private:
            float Vx{0.0f}, Vy{0.0f}, Vw{0.0f};
            float R, S;
            float Motor[4];
            float Wheel_Azimuth[4];      // 修复：去掉 const
            float Wheel_Coordinates[4][2]; 
    };
}

#endif
