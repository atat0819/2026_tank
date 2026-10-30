#include <cmath>
#include <cstdio>
#include "user/core/Alg/ChassisCalculation/OmniCalculation.hpp"
#include "user/core/Alg/ChassisCalculation/StringWheel.hpp"

using namespace Alg::CalculationBase;
constexpr float PI_TEST = 3.14159265358979323846f;
static int failures = 0;
static const char *section;

static void near(float actual, float expected, float tolerance = .0003f)
{
    if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance)
    {
        if (failures < 25) std::printf("FAIL %s: actual=%g expected=%g\n", section, actual, expected);
        ++failures;
    }
}

// Independently reconstruct chassis wrench from wheel torques and physical headings.
template<class Allocator>
static void wrench(const Allocator &allocator, const float phi[4], const float heading[4],
                   float radius, float wheel_radius, int count, float out[3])
{
    out[0] = out[1] = out[2] = 0;
    for (int i = 0; i < count; ++i)
    {
        const float f = allocator.GetMotorTorque(i) / wheel_radius;
        const float fx = f * std::cos(heading[i]), fy = f * std::sin(heading[i]);
        out[0] += fx;
        out[1] += fy;
        out[2] += radius * (std::cos(phi[i]) * fy - std::sin(phi[i]) * fx);
    }
}

static void omni()
{
    section = "omni IK/FK and force allocation";
    const float commands[][3] = {{1,0,0},{0,1,0},{0,0,1},{-.6f,.4f,-.8f},{0,0,0}};
    // Four symmetric wheels, three wheels, and a nonuniform full-rank four-wheel layout.
    const float layouts[][4] = {{0,PI_TEST/2,PI_TEST,3*PI_TEST/2},
                                {0,2*PI_TEST/3,4*PI_TEST/3,0}, {.1f,1.3f,3.0f,4.7f}};
    for (int layout = 0; layout < 3; ++layout)
    {
        const int count = layout == 1 ? 3 : 4;
        const float radius = .4f, wheel_radius = .05f;
        float phi[4], heading[4], coords[4][2];
        for (int i = 0; i < 4; ++i)
        {
            phi[i] = layouts[layout][i];
            heading[i] = phi[i] + PI_TEST / 2;
            coords[i][0] = radius * std::cos(phi[i]);
            coords[i][1] = radius * std::sin(phi[i]);
        }
        Omni_IK ik(radius, wheel_radius, heading, coords);
        Omni_FK fk(radius, wheel_radius, static_cast<float>(count), heading, phi);
        Omni_ID id(radius, wheel_radius, static_cast<float>(count), heading, coords);
        for (const auto &v : commands)
        {
            const float phase = .37f, gain = 1.3f, rotation_gain = .7f;
            const float expected[3] = {gain * (v[0]*std::cos(phase)+v[1]*std::sin(phase)),
                                      gain * (-v[0]*std::sin(phase)+v[1]*std::cos(phase)),
                                      rotation_gain*v[2]};
            ik.OmniInvKinematics(v[0], v[1], v[2], phase, gain, rotation_gain);
            float wheel[4];
            for (int i = 0; i < 4; ++i)
            {
                wheel[i] = ik.GetMotor(i);
                // Check actual rigid-body wheel velocity, independent of the inverse implementation.
                const float local_x = expected[0] - expected[2] * coords[i][1];
                const float local_y = expected[1] + expected[2] * coords[i][0];
                near(wheel[i], (local_x*std::cos(heading[i])+local_y*std::sin(heading[i]))/wheel_radius);
            }
            for (int repeat = 0; repeat < 3; ++repeat)
            {
                fk.OmniForKinematics(wheel[0], wheel[1], wheel[2], wheel[3]);
                near(fk.GetChassisVx(), expected[0]); near(fk.GetChassisVy(), expected[1]);
                near(fk.GetChassisVw(), expected[2]);
            }
            const float demand[3] = {v[0]*7, v[1]*9, v[2]*3};
            id.OmniInvDynamics(demand[0], demand[1], demand[2]);
            float reconstructed[3];
            wrench(id, phi, heading, radius, wheel_radius, count, reconstructed);
            for (int axis = 0; axis < 3; ++axis) near(reconstructed[axis], demand[axis]);
            float wheel_power = 0;
            for (int i = 0; i < count; ++i) wheel_power += id.GetMotorTorque(i)*wheel[i];
            near(wheel_power, demand[0]*expected[0]+demand[1]*expected[1]+demand[2]*expected[2]);
            if (count == 3) near(id.GetMotorTorque(3), 0);
        }
    }
}

static void swerve()
{
    section = "swerve units, IK/FK, repeated feedback and nearest turn";
    float phi[4] = {.1f, 1.3f, 3.0f, 4.7f}, offset[4] = {};
    const float radius = .4f, wheel_radius = .05f;
    String_IK ik(radius, wheel_radius, phi, offset);
    String_FK fk(radius, wheel_radius, phi);
    const float commands[][3] = {{1,0,0},{0,1,0},{0,0,1},{-.6f,.4f,-.8f},{0,0,0}};
    for (const auto &v : commands)
    {
        float current[4], wheel[4];
        for (int i = 0; i < 4; ++i)
        {
            current[i] = (i-2)*2.9f;
            ik.Set_current_steer_angles(current[i], i);
        }
        ik.StringInvKinematics(v[0], v[1], v[2], 0, 1, 1);
        for (int i = 0; i < 4; ++i)
        {
            wheel[i] = ik.GetMotor_wheel(i);
            const float angle = ik.GetMotor_direction(i);
            near(std::fmax(0.f,std::fabs(angle-current[i])-PI_TEST/2), 0);
            near(wheel[i]*wheel_radius*std::cos(angle), v[0]-v[2]*radius*std::sin(phi[i]));
            near(wheel[i]*wheel_radius*std::sin(angle), v[1]+v[2]*radius*std::cos(phi[i]));
            if (v[0] == 0 && v[1] == 0 && v[2] == 0) near(angle, current[i]);
            fk.Set_current_steer_angles(angle, i);
        }
        for (int repeat = 0; repeat < 5; ++repeat)
        {
            fk.OmniForKinematics(wheel[0], wheel[1], wheel[2], wheel[3]);
            near(fk.GetChassisVx(), v[0]); near(fk.GetChassisVy(), v[1]); near(fk.GetChassisVw(), v[2]);
        }
    }
    // Fresh object, repeated identical input: no historical accumulator is permitted.
    String_FK straight(radius, wheel_radius, phi);
    for (int repeat = 0; repeat < 5; ++repeat)
    {
        straight.OmniForKinematics(20,20,20,20);
        near(straight.GetChassisVx(),1); near(straight.GetChassisVy(),0); near(straight.GetChassisVw(),0);
    }
    section = "swerve wrench, radius and parallel/rank-deficient steering";
    float symmetric[4] = {0,PI_TEST/2,PI_TEST,3*PI_TEST/2};
    String_ID id(radius, wheel_radius, symmetric);
    id.OmniInvDynamics(4,0,0);
    for (int i = 0; i < 4; ++i) near(id.GetMotorTorque(i), .05f);
    // A symmetric layout makes the achievable projection particularly clear.
    String_ID parallel(radius, wheel_radius, symmetric);
    parallel.OmniInvDynamics(4,7,2);
    float zero[4] = {}, actual[3];
    wrench(parallel, symmetric, zero, radius, wheel_radius, 4, actual);
    near(actual[0],4); near(actual[1],0); near(actual[2],2);
    // Both full-rank and nearly parallel directions must remain finite.
    for (int mode = 0; mode < 2; ++mode)
    {
        float heading[4];
        for (int i = 0; i < 4; ++i)
        {
            heading[i] = mode == 0 ? symmetric[i]+PI_TEST/2 : (i-2)*1e-7f;
            parallel.Set_current_steer_angles(heading[i],i);
        }
        parallel.OmniInvDynamics(4,mode == 0 ? 7 : 0,2);
        wrench(parallel, symmetric, heading, radius, wheel_radius, 4, actual);
        near(actual[0],4); near(actual[1],mode == 0 ? 7 : 0); near(actual[2],2);
    }
}

static void offsets_and_boundaries()
{
    section = "swerve offsets, scaled/rotated commands and encoder wrap";
    float phi[4] = {PI_TEST/4,3*PI_TEST/4,5*PI_TEST/4,7*PI_TEST/4};
    float offsets[4] = {-1.3f, 3.44f, 2.85f, 1.33f};
    for (int sample = 0; sample < 80; ++sample)
    {
        const float radius = sample % 2 == 0 ? .17f : .6f, wheel_radius = .055f;
        String_IK ik(radius, wheel_radius, phi, offsets);
        String_FK fk(radius, wheel_radius, phi, offsets);
        String_ID id(radius, wheel_radius, phi, offsets);
        const float cmd[3] = {.7f*std::sin(sample*.71f), .6f*std::cos(sample*.43f), .5f};
        const float phase = .63f, gain = .8f, rotation_gain = 1.2f;
        const float expected[3] = {gain*(cmd[0]*std::cos(phase)+cmd[1]*std::sin(phase)),
                                  gain*(-cmd[0]*std::sin(phase)+cmd[1]*std::cos(phase)), rotation_gain*cmd[2]};
        float current[4], wheel[4];
        for (int i = 0; i < 4; ++i)
        {
            current[i] = offsets[i] + (sample-40)*.31f + i*1.8f;
            ik.Set_current_steer_angles(current[i], i);
        }
        ik.StringInvKinematics(cmd[0],cmd[1],cmd[2],phase,gain,rotation_gain);
        for (int i = 0; i < 4; ++i)
        {
            const float raw_angle = ik.GetMotor_direction(i), angle = raw_angle-offsets[i];
            wheel[i] = ik.GetMotor_wheel(i);
            near(std::fmax(0.f,std::fabs(raw_angle-current[i])-PI_TEST/2),0);
            near(wheel[i]*wheel_radius*std::cos(angle),expected[0]-expected[2]*radius*std::sin(phi[i]));
            near(wheel[i]*wheel_radius*std::sin(angle),expected[1]+expected[2]*radius*std::cos(phi[i]));
            fk.Set_current_steer_angles(raw_angle,i);
            // Independent full-rank physical headings, with encoder offsets and full turns added.
            id.Set_current_steer_angles(phi[i]+PI_TEST/2+offsets[i]+2*PI_TEST*(i-2),i);
        }
        fk.OmniForKinematics(wheel[0],wheel[1],wheel[2],wheel[3]);
        near(fk.GetChassisVx(),expected[0]); near(fk.GetChassisVy(),expected[1]); near(fk.GetChassisVw(),expected[2]);
        const float demand[3] = {4,-3,2};
        id.OmniInvDynamics(demand[0],demand[1],demand[2]);
        float physical[4], reconstructed[3], wheel_power = 0;
        for (int i = 0; i < 4; ++i)
        {
            physical[i] = phi[i]+PI_TEST/2;
            const float local_x = expected[0]-expected[2]*radius*std::sin(phi[i]);
            const float local_y = expected[1]+expected[2]*radius*std::cos(phi[i]);
            const float speed = (local_x*std::cos(physical[i])+local_y*std::sin(physical[i]))/wheel_radius;
            wheel_power += id.GetMotorTorque(i)*speed;
        }
        wrench(id,phi,physical,radius,wheel_radius,4,reconstructed);
        for (int axis = 0; axis < 3; ++axis) near(reconstructed[axis],demand[axis]);
        near(wheel_power,demand[0]*expected[0]+demand[1]*expected[1]+demand[2]*expected[2]);
        ik.StringInvKinematics(0,0,0,0,1,1);
        for (int i = 0; i < 4; ++i) { near(ik.GetMotor_direction(i),current[i]); near(ik.GetMotor_wheel(i),0); }
    }
    // Explicit 179 -> -179 degree seam and 180-degree reversal.
    float zero[4] = {};
    String_IK seam(.4f,.05f,phi,zero);
    for (int i = 0; i < 4; ++i) seam.Set_current_steer_angles(179*PI_TEST/180,i);
    seam.StringInvKinematics(std::cos(-179*PI_TEST/180),std::sin(-179*PI_TEST/180),0,0,1,1);
    for (int i = 0; i < 4; ++i) { near(seam.GetMotor_direction(i),181*PI_TEST/180); near(seam.GetMotor_wheel(i),20); }
    for (int i = 0; i < 4; ++i) seam.Set_current_steer_angles(0,i);
    seam.StringInvKinematics(-1,0,0,0,1,1);
    for (int i = 0; i < 4; ++i) { near(seam.GetMotor_direction(i),0); near(seam.GetMotor_wheel(i),-20); }
}

int main()
{
    omni(); swerve(); offsets_and_boundaries();
    if (failures) { std::printf("%d numerical checks failed\n", failures); return 1; }
    std::puts("PASS: omni/swerve units, independent geometry, IK/FK, wrench, virtual power and degenerate steering");
}
