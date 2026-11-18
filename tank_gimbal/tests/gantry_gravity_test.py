"""Exercise the production lift output loop with nonzero signed compensation."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
task = (root / 'RtosTask/gantry_task.cpp').read_text(encoding='utf-8')
start = task.index('        for (unsigned i = 0; i < 2; ++i)', task.index('// 计算保持速度'))
end = task.index('        gantry_3508_motor.sendCAN();', start)
loop = task[start:end]
code = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
enum class GantryState { LiftFirst, Extend, LiftSecond, Retract, Lower, Idle };
namespace GantryTuning {
constexpr float LIFT_ID5_GRAVITY_EMPTY_CMD=819.2f, LIFT_ID6_GRAVITY_EMPTY_CMD=-1638.4f;
constexpr float LIFT_ID5_GRAVITY_LOADED_CMD=2457.6f, LIFT_ID6_GRAVITY_LOADED_CMD=-3276.8f;
}
using namespace GantryTuning;
struct FSM {
    GantryState state=GantryState::LiftFirst;
    bool active=true, moving=true;
    float factor=0;
    bool IsMoving(unsigned) const { return active && moving; }
    bool IsKeeping(unsigned) const { return active && !moving && state!=GantryState::Idle; }
    float GetTargetSpeed(unsigned) const { return 0; }
    float GetHoldAngle(unsigned) const { return 0; }
    GantryState GetState() const { return state; }
    float GetGravityLoadFactor() const { return factor; }
} gantry_fsm;
struct PID { float output=0; void reset() {} float UpDate(float, float) { return output; } };
PID lift_move_pid[2], lift_hold_pid[2], hold_angle_pid[2];
struct Watch { float target_velocity_rad_s[2]; int16_t lift_command[2]; } gantry_watch;
struct Motor { int16_t command[2]; void setCAN(int16_t c, unsigned i) { command[i-1]=c; } } gantry_3508_motor;
float velocity[2]={}, angle_deg[2]={};
void run() {
LOOP
}
int main() {
    run(); assert(gantry_3508_motor.command[0]==819 && gantry_3508_motor.command[1]==-1638);
    gantry_fsm.state=GantryState::Extend; gantry_fsm.moving=false; run();
    assert(gantry_3508_motor.command[0]==819 && gantry_3508_motor.command[1]==-1638);
    gantry_fsm.moving=true;
    gantry_fsm.state=GantryState::LiftSecond; gantry_fsm.factor=1; run();
    assert(gantry_3508_motor.command[0]==2457 && gantry_3508_motor.command[1]==-3276);
    gantry_fsm.state=GantryState::Retract; gantry_fsm.moving=false; run();
    assert(gantry_3508_motor.command[0]==2457 && gantry_3508_motor.command[1]==-3276);
    gantry_fsm.state=GantryState::Lower; gantry_fsm.moving=true; gantry_fsm.factor=.5f; run();
    assert(gantry_3508_motor.command[0]==1638 && gantry_3508_motor.command[1]==-2457);
    gantry_fsm.factor=0; run();
    assert(gantry_3508_motor.command[0]==819 && gantry_3508_motor.command[1]==-1638);
    gantry_fsm.factor=.5f;
    lift_move_pid[0].output=-5000; lift_move_pid[1].output=5000; run();
    assert(gantry_3508_motor.command[0]==-3361 && gantry_3508_motor.command[1]==2542);
    lift_move_pid[0].output=16000; lift_move_pid[1].output=-16000; run();
    assert(gantry_3508_motor.command[0]==16384 && gantry_3508_motor.command[1]==-16384);
    gantry_fsm.state=GantryState::LiftSecond; lift_move_pid[0].output=16000;
    lift_move_pid[1].output=-16000; run();
    assert(gantry_3508_motor.command[0]==16384 && gantry_3508_motor.command[1]==-16384);
    gantry_fsm.active=false; run();
    assert(gantry_3508_motor.command[0]==0 && gantry_3508_motor.command[1]==0);
    gantry_fsm.active=true; gantry_fsm.moving=false; gantry_fsm.state=GantryState::Idle; run();
    assert(gantry_3508_motor.command[0]==0 && gantry_3508_motor.command[1]==0);
}
'''.replace('LOOP', loop)
with tempfile.TemporaryDirectory(prefix='gantry_gravity_') as directory:
    cpp, exe = Path(directory) / 'test.cpp', Path(directory) / 'test.exe'
    cpp.write_text(code, encoding='utf-8')
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-Wdouble-promotion',
                    str(cpp), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print('PASS: signed gravity, motion/hold, blending, total limits and zero output')
