#include <cassert>
#include <cmath>

#include "yaw_auto_lqr_eso_controller.h"

static bool Near(float actual, float expected)
{
    return std::fabs(actual - expected) < 1.0e-5f;
}

int main()
{
    YawAutoLqrEso_t ctrl;
    YawAutoLqrEso_Init(&ctrl);

    YawAutoLqrEsoConfig_t cfg = {};
    cfg.k_theta = 12.0f;
    cfg.k_omega = 3.4f;
    cfg.k_i = 0.5f;
    cfg.theta_integral_limit_rad_s = 1.0f;

    const YawAutoLqrEsoFeedback_t feedback = {0.0f, 0.0f, 0.0f, 1U};
    const YawAutoLqrEsoReference_t reference = {0.1f, 0.2f, 0.0f};
    YawAutoLqrEsoOutput_t output = {};

    YawAutoLqrEso_Calc(&ctrl, &cfg, &feedback, &reference, 0.01f, &output);
    assert(Near(output.tau_cmd_nm, 0.0f)); // 首次有效反馈只同步状态

    YawAutoLqrEso_Calc(&ctrl, &cfg, &feedback, &reference, 0.01f, &output);
    assert(Near(output.e_theta_rad, 0.1f));
    assert(Near(output.e_omega_rad_s, 0.2f));
    assert(Near(output.tau_i_nm, 0.0005f));
    assert(Near(output.tau_cmd_nm, 1.8805f));
}
