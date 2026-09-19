#include "chassis_keyboard_fsm.hpp"

/*
 * 底盘键盘控制状态机的实现。
 *
 * 本文件只负责：
 * 1. 对 16 位原始按键掩码进行逐键去抖；
 * 2. 在键盘模式或键盘链路失效时清零控制状态；
 * 3. 将 CTRL、Z、B 按键的稳定按下沿转换为开关或一次性楼梯动作；
 * 4. 将 W/S/A/D、SHIFT 等稳定按键转换为底盘运动和控制选项。
 *
 * 本文件不直接控制底盘电机。任务层通过 GetCommand() 读取本文件生成的
 * KeyboardMotionCommand，再根据 vx、vy 和各个开关状态执行具体控制。
 */

/*
 * 初始化键盘状态机。
 *
 * 所有按键状态、边沿检测状态和输出命令都由 Reset() 恢复到安全初始值。
 */
void ChassisKeyboardFSM::Init()
{
    Reset();
}

/*
 * 清空键盘状态和输出命令。
 *
 * Reset() 用于初始化，也用于退出键盘模式或键盘链路掉线时的安全复位：
 * - 清除原始掩码和稳定掩码；
 * - 清除每个按键的去抖计时；
 * - 清除 CTRL、Z、B 的边沿检测状态；
 * - 关闭陀螺和跟随选项；
 * - 将 command_ 恢复为全零无效命令。
 */
void ChassisKeyboardFSM::Reset()
{
    stable_key_mask_ = 0;
    last_raw_key_mask_ = 0;
    first_sample_tick_ = 0;
    first_sample_pending_ = true;
    stable_mask_initialized_ = false;
    last_ctrl_pressed_ = false;
    gyro_enabled_ = false;
    last_z_pressed_ = false;
    last_b_pressed_ = false;
    follow_enabled_ = false;
    command_ = {};

    for (uint8_t i = 0; i < 16; ++i)
    {
        key_change_tick_[i] = 0;
    }
}

/*
 * 更新键盘状态机。
 *
 * 参数说明：
 * - raw_key_mask：本周期读取到的 16 位原始按键掩码；
 * - keyboard_mode：当前是否允许使用键盘控制；
 * - keyboard_online：键盘通信链路是否在线；
 * - now_tick：当前系统 tick，用于 20 ms 去抖计时。
 *
 * 函数先检查键盘控制条件，然后完成首次进入同步、初始稳定掩码建立、
 * 各按键独立去抖、CTRL/Z/B 边沿处理，最后刷新运动命令。
 */
void ChassisKeyboardFSM::Update(uint16_t raw_key_mask,
                                bool keyboard_mode,
                                bool keyboard_online,
                                uint32_t now_tick)
{
    // 不在键盘模式或键盘链路掉线时，立即撤销上一次键盘命令。
    if (!keyboard_mode || !keyboard_online)
    {
        Reset();
        return;
    }

    // 本次更新已经具备有效的键盘输入；楼梯动作默认每周期不触发。
    command_.valid = true;
    command_.stair_toggle = false;

    /*
     * 第一次进入键盘模式只记录当前按键，不立即产生 CTRL/Z/B 的切换动作。
     * 这样可以避免进入键盘模式时已经按住的按键被误认为“新按下”。
     */
    if (first_sample_pending_)
    {
        last_raw_key_mask_ = raw_key_mask;
        first_sample_tick_ = now_tick;
        first_sample_pending_ = false;
        // 进入键盘模式默认打开跟随；当前已按住的 CTRL/Z 会在后续稳定化时
        // 同步为“已按下”，但不会产生一次新的翻转事件。
        follow_enabled_ = true;
        for (uint8_t i = 0; i < 16; ++i)
        {
            key_change_tick_[i] = now_tick;
        }
        UpdateCommand();
        return;
    }

    /*
     * 首次样本建立阶段：要求整帧原始掩码连续稳定 KEY_DEBOUNCE_MS 后，才把
     * 它作为稳定掩码。期间不执行 CTRL/Z/B 边沿动作，避免进入模式时误触发。
     */
    if (!stable_mask_initialized_)
    {
        if (raw_key_mask != last_raw_key_mask_)
        {
            // 原始整帧发生变化，重新开始首次稳定计时。
            last_raw_key_mask_ = raw_key_mask;
            first_sample_tick_ = now_tick;
        }
        else if (now_tick - first_sample_tick_ >= KEY_DEBOUNCE_MS)
        {
            // 整帧稳定达到去抖时间，建立初始稳定状态和边沿基准。
            stable_key_mask_ = last_raw_key_mask_;
            stable_mask_initialized_ = true;
            last_ctrl_pressed_ = (stable_key_mask_ & KEY_CTRL) != 0U;
            last_z_pressed_ = (stable_key_mask_ & KEY_Z) != 0U;
            last_b_pressed_ = (stable_key_mask_ & KEY_B) != 0U;
            UpdateCommand();
            return;
        }

        UpdateCommand();
        return;
    }

    // 每个按键独立去抖，因此 W+A 或 W+D 同时输入不会互相延迟或干扰。
    for (uint8_t i = 0; i < 16; ++i)
    {
        const uint16_t bit = static_cast<uint16_t>(1U << i);
        const bool raw_pressed = (raw_key_mask & bit) != 0U;
        const bool last_raw_pressed = (last_raw_key_mask_ & bit) != 0U;

        if (raw_pressed != last_raw_pressed)
        {
            // 记录该键新的原始电平，并从当前 tick 重新计时。
            if (raw_pressed)
            {
                last_raw_key_mask_ |= bit;
            }
            else
            {
                last_raw_key_mask_ &= static_cast<uint16_t>(~bit);
            }
            key_change_tick_[i] = now_tick;
        }
        else if (raw_pressed != ((stable_key_mask_ & bit) != 0U) &&
                 now_tick - key_change_tick_[i] >= KEY_DEBOUNCE_MS)
        {
            // 原始电平持续稳定达到去抖时间，才提交到稳定掩码。
            if (raw_pressed)
            {
                stable_key_mask_ |= bit;
            }
            else
            {
                stable_key_mask_ &= static_cast<uint16_t>(~bit);
            }
        }
    }

    // CTRL 采用稳定按下沿翻转陀螺开关，长按不会重复翻转。
    const bool ctrl_pressed = (stable_key_mask_ & KEY_CTRL) != 0U;
    if (ctrl_pressed && !last_ctrl_pressed_)
    {
        gyro_enabled_ = !gyro_enabled_;
    }
    last_ctrl_pressed_ = ctrl_pressed;

    // Z 采用稳定按下沿翻转跟随开关，长按不会重复翻转。
    const bool z_pressed = (stable_key_mask_ & KEY_Z) != 0U;
    if (z_pressed && !last_z_pressed_)
    {
        follow_enabled_ = !follow_enabled_;
    }
    last_z_pressed_ = z_pressed;

    // B 的稳定按下沿只生成一个周期的楼梯切换请求。
    const bool b_pressed = (stable_key_mask_ & KEY_B) != 0U;
    if (b_pressed && !last_b_pressed_)
    {
        command_.stair_toggle = true;
    }
    last_b_pressed_ = b_pressed;

    UpdateCommand();
}

/*
 * 根据稳定按键状态生成当前键盘控制命令。
 *
 * W/S 叠加生成前后方向 vx，A/D 叠加生成左右方向 vy；SHIFT、陀螺开关和
 * 跟随开关直接写入 command_。如果方向相反的按键同时按下，对应分量会
 * 相互抵消。
 */
void ChassisKeyboardFSM::UpdateCommand()
{
    float forward = 0.0f;
    float lateral = 0.0f;

    // W 前进，S 后退；两个同时按下时相互抵消。
    if ((stable_key_mask_ & KEY_W) != 0U)
    {
        forward += 1.0f;
    }
    if ((stable_key_mask_ & KEY_S) != 0U)
    {
        forward -= 1.0f;
    }
    // A 左移，D 右移；两个同时按下时相互抵消。
    if ((stable_key_mask_ & KEY_A) != 0U)
    {
        lateral -= 1.0f;
    }
    if ((stable_key_mask_ & KEY_D) != 0U)
    {
        lateral += 1.0f;
    }

    // 输出方向量和模式选项，供任务层读取。
    command_.vx = forward;
    command_.vy = lateral;
    command_.shift_pressed = (stable_key_mask_ & KEY_SHIFT) != 0U;
    command_.gyro_enabled = gyro_enabled_;
    command_.follow_enabled = follow_enabled_;
}
