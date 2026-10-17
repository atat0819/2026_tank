"""Verify source selection occurs before control, limits and compensation."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FeedbackIntegrationTest(unittest.TestCase):
    def test_gravity_encoder_horizontal_zero(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        self.assertTrue("PITCH_ENCODER_HORIZONTAL_RAD = 46.7f * DEG_TO_RAD" in task,
                        "measured horizontal encoder zero is missing")
        start = task.index(" // 重力补偿按反馈来源换算水平角：")
        end = task.index(" // Pitch STOP", start)
        block = task[start:end]
        header = (ROOT / "user/core/Alg/Feedforward/Feedforward.hpp").read_text(encoding="utf-8-sig")
        start = header.index("    class Gravity\n")
        end = header.index("\n    };", start) + len("\n    };")
        harness = '#include <cassert>\n#include <cmath>\nnamespace Alg::Feedforward {\n'
        harness += header[start:end] + '\n}\n'
        harness += 'constexpr float DEG_TO_RAD=3.14159265358979323846f/180.0f;\n'
        harness += 'constexpr float PITCH_ENCODER_HORIZONTAL_RAD=46.7f*DEG_TO_RAD;\n'
        harness += 'Alg::Feedforward::Gravity pitch_gravity_ff(2.0f,0.0f);\n'
        harness += 'float compute(bool using_encoder,float pitch_current_angle){\n'
        harness += block + 'return pitch_gravity_output;\n}\n'
        harness += r'''
int main() {
    assert(compute(false,0.0f)==2.0f);
    assert(compute(true,PITCH_ENCODER_HORIZONTAL_RAD)==2.0f);
    for (float angle : {-10.0f, 20.0f}) {
        const float imu=compute(false,angle*DEG_TO_RAD);
        const float encoder=compute(true,PITCH_ENCODER_HORIZONTAL_RAD+angle*DEG_TO_RAD);
        assert(std::fabs(imu-encoder)<0.00001f);
    }
}
'''
        harness = '#include <initializer_list>\n' + harness
        with tempfile.TemporaryDirectory(prefix="pitch_zero_") as temp:
            cpp = Path(temp) / "test.cpp"
            exe = Path(temp) / "test.exe"
            cpp.write_text(harness, encoding="utf-8")
            subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                            cpp.as_posix(), "-o", exe.as_posix()], check=True)
            subprocess.run([str(exe)], check=True)

    def test_actual_feedback_selector(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        begin = task.index("static bool UpdateGimbalFeedback(", task.index("void ControlTask() {"))
        selector = task[begin:task.index("\n}", begin) + 2]
        harness = r'''
#include <cassert>
#include <cmath>
#include <limits>
#include "feeder_fsm/imu_feedback_fsm.hpp"
ImuFeedbackFSM imu_feedback_fsm;
unsigned tick=0;
unsigned HAL_GetTick() { return tick; }
float DegreesToRadians(float x) { return x * 3.14159265358979323846f / 180.0f; }
int unwrap_calls=0;
bool last_unwrap_reset=false;
float GetContinuousYawAngle(float angle, bool reset) {
    assert(std::isfinite(angle)); ++unwrap_calls; last_unwrap_reset=reset; return angle;
}
struct {
    bool online=true;
    float angles[3]={0,10,20};
    float rates[3]={30,0,40};
    bool isConnected() { return online; }
    float GetAngle(int i) { return angles[i]; }
    float GetGyro(int i) { return rates[i]; }
} imu;
struct {
    float getAngleRad(int id) { return id==1 ? 0.3f : 0.4f; }
    float getVelocityRads(int id) { return id==1 ? 0.5f : 0.6f; }
} gimbal_motor;
struct { float yaw=0,pitch=0,gyro_z=0,gyro_y=0; } ImuData_user;
SELECTOR
int main() {
    float ya=0,ys=0,pa=0,ps=0;
    assert(!UpdateGimbalFeedback(ya,ys,pa,ps));
    assert(ya==0.3f && ys==0.5f && pa==0.4f && ps==0.6f);
    assert(unwrap_calls==0);
    tick=100;
    assert(UpdateGimbalFeedback(ya,ys,pa,ps));
    assert(ya==DegreesToRadians(20) && ys==DegreesToRadians(40));
    assert(pa==DegreesToRadians(10) && ps==DegreesToRadians(30));
    assert(last_unwrap_reset);
    tick=101;
    assert(!UpdateGimbalFeedback(ya,ys,pa,ps));
    assert(!last_unwrap_reset);
    imu.angles[2]=std::numeric_limits<float>::infinity();
    assert(UpdateGimbalFeedback(ya,ys,pa,ps));
    assert(unwrap_calls==2 && ya==0.3f && pa==0.4f);
    imu.angles[2]=20;
    tick=102; UpdateGimbalFeedback(ya,ys,pa,ps);
    tick=202; assert(UpdateGimbalFeedback(ya,ys,pa,ps));
    assert(last_unwrap_reset);
    imu.online=false;
    tick=203; assert(UpdateGimbalFeedback(ya,ys,pa,ps));
    assert(ys==0.5f && ps==0.6f);
    // IMU checks keep running during remote safety: another loss restarts recovery.
    imu.online=true;
    tick=204; assert(!UpdateGimbalFeedback(ya,ys,pa,ps));
    imu.online=false;
    tick=250; assert(!UpdateGimbalFeedback(ya,ys,pa,ps));
    imu.online=true;
    tick=550; assert(!UpdateGimbalFeedback(ya,ys,pa,ps));
    tick=649; assert(!UpdateGimbalFeedback(ya,ys,pa,ps));
    tick=650; assert(UpdateGimbalFeedback(ya,ys,pa,ps));
}
'''.replace("SELECTOR", selector)
        with tempfile.TemporaryDirectory(prefix="imu_selector_") as temp:
            cpp = Path(temp) / "test.cpp"
            exe = Path(temp) / "test.exe"
            cpp.write_text(harness, encoding="utf-8")
            subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                            "-I", str(ROOT), str(cpp), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_selected_feedback_and_source_transition(self):
        task = (ROOT / "RtosTask/can_send_task.cpp").read_text(encoding="utf-8-sig")
        self.assertTrue("UpdateGimbalFeedback" in task, "feedback selector is missing")
        loop = task.split("    for(;;)", 1)[1]
        self.assertLess(loop.index("const bool feedback_source_changed = UpdateGimbalFeedback("),
                        loop.index("if(!remoteController.isConnected()"))
        selector = task[task.index("static bool UpdateGimbalFeedback(", task.index("void ControlTask() {")):]
        self.assertLess(selector.index("imu_feedback_fsm.Update"), selector.index("GetContinuousYawAngle"))
        for value in ("gimbal_motor.getAngleRad(1)", "gimbal_motor.getAngleRad(2)",
                      "gimbal_motor.getVelocityRads(1)", "gimbal_motor.getVelocityRads(2)"):
            self.assertIn(value, selector)
        transition = task.split("if (feedback_source_changed)", 1)[1].split("// 电机恢复按轴处理", 1)[0]
        self.assertIn("yaw_gimbal_fsm.ReAnchor(yaw_current_angle)", transition)
        self.assertIn("pitch_gimbal_fsm.ReAnchor(pitch_current_angle)", transition)
        self.assertIn("ResetGimbalControlState()", transition)
        self.assertNotIn("Set_Angle_Limit_Enabled", task)
        self.assertIn("pitch_gimbal_fsm.Update_Angle_Limit(DegreesToRadians(gimbal_motor.getAngleDeg(2)), pitch_current_angle)", task)
        for axis in ("yaw", "pitch"):
            self.assertIn(f"{axis}_input.vision_ready = !using_encoder &&", task)
            self.assertIn(f"{axis}_input.vision_fresh = !using_encoder &&", task)
        self.assertIn("Alg::Feedforward::Gravity pitch_gravity_ff", task)
        gravity = task.split(" // 重力补偿按反馈来源换算水平角：", 1)[1].split("// Pitch STOP", 1)[0]
        self.assertIn("pitch_current_angle - PITCH_ENCODER_HORIZONTAL_RAD", gravity)
        self.assertIn("pitch_gravity_ff.GravityFeedforward(pitch_gravity_angle)", gravity)
        self.assertIn("pitch_gravity_output = pitch_gravity_ff.getFeedforward()", gravity)
        self.assertNotIn("pitch_ude", task)
        self.assertNotIn("pitch_last_control_output", task)
        self.assertNotIn("&& !imu_fault", task)
        self.assertIn("yaw_gimbal_fsm_config.min_angle = DegreesToRadians(-60.0f)", task)
        self.assertIn("yaw_gimbal_fsm_config.max_angle = DegreesToRadians(60.0f)", task)
        self.assertIn("yaw_gimbal_fsm.Limit_Torque(yaw_control_output)", task)


if __name__ == "__main__":
    unittest.main()
