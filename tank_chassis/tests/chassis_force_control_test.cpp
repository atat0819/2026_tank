#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include "user/core/Alg/ChassisCalculation/MecanumForceControl.hpp"
#include "user/core/Alg/PowerControl/PowerControl.hpp"
#include "user/core/Alg/Filter/Filter.hpp"
#include "task_power_coefficients.hpp"
using Controller = Alg::CalculationBase::MecanumForceControl;

static void zero_gains() {
    Controller control;
    for(unsigned axis=0;axis<3;++axis) {
        control.kp[axis]=0;
        control.ki[axis]=0;
        control.integral_limit[axis]=0;
    }
    const float target[3]={2,-1,3},feedback[3]={-.5f,.2f,-.7f};
    float raw[4];
    for(unsigned step=0;step<1000;++step) control.Update(target,feedback,raw);
    for(float value:raw) assert(value==0);
}

// Synthetic test gains only; independent of the user's firmware tuning.
static void configure_test_gains(Controller &control) {
    for(unsigned axis=0;axis<3;++axis) {
        control.kp[axis]=axis==2?10:100;
        control.ki[axis]=axis==2?3:30;
        control.integral_limit[axis]=axis==2?5:20;
    }
}

static void near(float actual, float expected, float tolerance=0.0001f) {
    assert(std::isfinite(actual) && std::isfinite(expected));
    if (std::fabs(actual-expected)>tolerance) {
        std::printf("actual=%g expected=%g tolerance=%g\n",actual,expected,tolerance);
        assert(false);
    }
}
// Independent signed wheel-force sum in the existing operator axes.
static void reconstruct(const float current[4], float force[3]) {
    const float k=Controller::WHEEL_TORQUE_PER_AMP/0.076f;
    force[0]=k*(-current[0]+current[1]+current[2]-current[3]);
    force[1]=k*(-current[0]-current[1]+current[2]+current[3]);
    force[2]=k*0.458f*(current[0]+current[1]+current[2]+current[3]);
}
static void geometry() {
    const float commands[][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},
                              {0,0,1},{0,0,-1},{.4f,-.7f,.2f},{0,0,0}};
    const float sx[4]={-1,1,1,-1}, sy[4]={-1,-1,1,1};
    for (const auto &command:commands) {
        float wheels[4], feedback[3];
        Controller::InverseKinematics(command,wheels);
        for(unsigned i=0;i<4;++i)
            near(wheels[i],(sx[i]*command[0]+sy[i]*command[1]+.458f*command[2])/.076f);
        Controller::ForwardKinematics(wheels,feedback);
        for(unsigned i=0;i<3;++i) near(feedback[i],command[i]);
    }
    const float demands[][3]={{20,0,0},{0,-30,0},{0,0,4},{11,-7,3}};
    const float velocity[3]={.7f,-.2f,.8f};
    float wheels[4]; Controller::InverseKinematics(velocity,wheels);
    for(const auto &demand:demands) {
        float torque[4],current[4],actual[3],pw=0,pb=0;
        Controller::InverseDynamics(demand,torque);
        for(unsigned i=0;i<4;++i) {current[i]=torque[i]/Controller::WHEEL_TORQUE_PER_AMP;pw+=torque[i]*wheels[i];}
        reconstruct(current,actual);
        for(unsigned i=0;i<3;++i) {near(actual[i],demand[i]);pb+=demand[i]*velocity[i];}
        near(pw,pb);
    }
}
static void pi_and_saturation() {
    Controller control;
    configure_test_gains(control);
    const float ref[3]={.1f,-.08f,.15f},fb[3]={.03f,.02f,-.1f},zero[3]={};
    float raw[4],current[4],actual_force[3];
    for(float &ki:control.ki) ki=0;
    control.Update(ref,fb,raw);
    for(unsigned i=0;i<4;++i) current[i]=raw[i]/Controller::RAW_PER_AMP;
    reconstruct(current,actual_force);
    for(unsigned axis=0;axis<3;++axis) near(actual_force[axis],control.kp[axis]*(ref[axis]-fb[axis]));
    const float large[3]={8,-6,10};control.Update(large,zero,raw);
    float torque[4],peak=0;Controller::InverseDynamics(control.force,torque);
    for(float value:raw) {assert(std::isfinite(value));peak=std::fmax(peak,std::fabs(value));}
    near(peak,16384,.01f);
    for(unsigned i=0;i<4;++i)
        near(raw[i]/16384,torque[i]/(Controller::WHEEL_TORQUE_PER_AMP*20)*control.output_scale);
    control=Controller();
    configure_test_gains(control);
    for(unsigned step=0;step<10000;++step) control.Update(large,zero,raw);
    for(float value:control.integral) near(value,0); // Internal current limit also stops windup.
    for(unsigned step=0;step<10000;++step) {control.Update(ref,zero,raw);control.ApplyOutputScale(0);}
    for(float value:control.integral) near(value,0);
    control.Update(zero,zero,raw);for(float value:raw) near(value,0);
    control.kp[0]=0;control.ki[0]=1;control.integral[0]=1;
    const float unwind[3]={-.1f,0,0};control.Update(unwind,zero,raw);control.ApplyOutputScale(.5f);
    assert(control.integral[0]<1);
    control.Reset();control.Update(zero,zero,raw);for(float value:raw) near(value,0);
}
static void velocity_plant() {
    Controller control;
    configure_test_gains(control);
    SecondOrderLPFFilter xf(test_x_filter[0],test_x_filter[1],test_x_filter[2]);
    SecondOrderLPFFilter yf(test_y_filter[0],test_y_filter[1],test_y_filter[2]);
    SecondOrderLPFFilter zf(test_z_filter[0],test_z_filter[1],test_z_filter[2]);
    ALG::PowerControl::PowerControl<4> limiter;
    float velocity[3]={},acceleration[3]={};
    const float inertia[3]={23,23,1.5f},drag[3]={4,4,.8f},disturbance[3]={5,-2,.2f};
    const float refs[][3]={{.8f,-.4f,.6f},{0,0,0},{-.5f,.3f,-.4f}};
    for(const auto &ref:refs) {
        for(unsigned step=0;step<12000;++step) {
            float wheels[4],fb[3],raw[4],current[4],rotor[4],other[4]={};
            Controller::InverseKinematics(velocity,wheels);Controller::ForwardKinematics(wheels,fb);
            fb[0]=xf.Filter(fb[0]);fb[1]=yf.Filter(fb[1]);fb[2]=zf.Filter(fb[2]);control.Update(ref,fb,raw);
            for(unsigned i=0;i<4;++i) {current[i]=raw[i]*(20.f/16384);rotor[i]=wheels[i]*Controller::REDUCTION_RATIO;}
            limiter.DecayingCurrent(current,rotor,poly_coeffs,other,0,50);control.ApplyOutputScale(limiter.getEta());
            float power=0;
            for(unsigned i=0;i<4;++i) {
                current[i]=limiter.getCurrentCalculate(i);
                assert(std::isfinite(current[i])&&std::fabs(current[i])<=20.0001f);
                power+=poly_coeffs[0]+poly_coeffs[1]*current[i]+poly_coeffs[2]*std::fabs(rotor[i])+
                    poly_coeffs[3]*current[i]*rotor[i]+poly_coeffs[4]*current[i]*current[i]+poly_coeffs[5]*rotor[i]*rotor[i];
            }
            assert(power<=50.01f);
            float force[3];reconstruct(current,force);
            for(unsigned i=0;i<3;++i) {acceleration[i]=(force[i]-drag[i]*velocity[i]-disturbance[i])/inertia[i];velocity[i]+=.001f*acceleration[i];}
        }
        for(unsigned i=0;i<3;++i) {near(velocity[i],ref[i],.015f);near(acceleration[i],0,.015f);}
    }
}
int main() {
    zero_gains();geometry();pi_and_saturation();velocity_plant();
    std::puts("PASS: geometry, force/power consistency, PI, current/power limits and filtered speed plant");
}
