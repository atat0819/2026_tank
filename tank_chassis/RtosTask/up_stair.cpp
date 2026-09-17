#include "up_stair.hpp"
#include "imu_task.hpp"
#include "Alg/PID/pid.hpp"

BSP::Motor::DM::J4310<2> front_4340(
    0x00, {0x01, 0x02}, {0x01, 0x02}, HAL::FDCAN::FdcanDeviceId::HAL_Fdcan2);

BSP::Motor::DM::J6248<2> rear_6248(
    0x00, {0x03, 0x04}, {0x03, 0x04}, HAL::FDCAN::FdcanDeviceId::HAL_Fdcan3);

ALG::PID::PID front_4340_left_pid[2] = {
    {0.0f, 0.0f, 0.0f, 16384.0f, 5000.0f, 500.0f},   
	{0.0f, 0.0f, 0.0f, 16384.0f, 5000.0f, 500.0f},    
};

ALG::PID::PID front_4340_right_pid[2] = {
    {0.0f, 0.0f, 0.0f, 16384.0f, 5000.0f, 500.0f},   
	{0.0f, 0.0f, 0.0f, 16384.0f, 5000.0f, 500.0f},    
};


ALG::PID::PID rear_6248_left_pid[2] = {
    {0.0f, 0.0f, 0.0f, 16384.0f, 5000.0f, 500.0f},   
	{0.0f, 0.0f, 0.0f, 16384.0f, 5000.0f, 500.0f},    
};

ALG::PID::PID rear_6248_right_pid[2] = {    
    {0.0f, 0.0f, 0.0f, 16384.0f, 5000.0f, 500.0f},   
	{0.0f, 0.0f, 0.0f, 16384.0f, 5000.0f, 500.0f},    
};

volatile bool dm_motor_control_ready = false;


    float torque_front_left = 0.0f;
    float torque_front_right = 0.0f;
    float torque_rear_left = 0.0f;
    float torque_rear_right = 0.0f;


extern "C" void up_stair_task(void *argument)
{
    (void)argument;

    while (!dm_motor_control_ready)
    {
        osDelay(1U);
    }

    front_4340.On(1, BSP::Motor::DM::Model::MIT);
    front_4340.On(2, BSP::Motor::DM::Model::MIT);
    rear_6248.On(1, BSP::Motor::DM::Model::MIT);
    rear_6248.On(2, BSP::Motor::DM::Model::MIT);

    for (;;)
    {
        if (bmi088.IsReady())
        {
            const float pitch_deg = bmi088.GetPitchAngleDeg();
            const float pitch_rate_dps = bmi088.GetGyroRateYDps();

            // Calculate the four torques here, then send them directly below.
            (void)pitch_deg;
            (void)pitch_rate_dps;


            front_4340.ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, torque_front_left);
            front_4340.ctrl_Mit(2, 0.0f, 0.0f, 0.0f, 0.0f, torque_front_right);
            rear_6248.ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, torque_rear_left);
            rear_6248.ctrl_Mit(2, 0.0f, 0.0f, 0.0f, 0.0f, torque_rear_right);


        }

        osDelay(1U);
    }
}
