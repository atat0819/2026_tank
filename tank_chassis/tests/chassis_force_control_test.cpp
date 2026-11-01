#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include "user/core/Alg/ChassisCalculation/MecanumForceControl.hpp"
#include "user/core/Alg/UtilityFunction/SecondOrderSetpointPlanner.hpp"
#include "user/core/Alg/PowerControl/PowerControl.hpp"
#include "task_power_coefficients.hpp"
using Controller = Alg::CalculationBase::MecanumForceControl;

static void near(float actual,float expected,float tolerance=.0001f) {
    assert(std::isfinite(actual)&&std::isfinite(expected));
    if(std::fabs(actual-expected)>tolerance) {
        std::printf("actual=%g expected=%g tolerance=%g\n",actual,expected,tolerance);assert(false);
    }
}
static void reconstruct(const float current[4],float force[3]) {
    const float k=Controller::WHEEL_TORQUE_PER_AMP/.076f;
    force[0]=k*(-current[0]+current[1]+current[2]-current[3]);
    force[1]=k*(-current[0]-current[1]+current[2]+current[3]);
    force[2]=k*.458f*(current[0]+current[1]+current[2]+current[3]);
}
static void geometry() {
    near(Controller::REDUCTION_RATIO,268.0f/17.0f,.000001f);
    near(Controller::WHEEL_TORQUE_PER_AMP,.2489164087f,.0000001f);
    const float commands[][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1},{.4f,-.7f,.2f}};
    const float sx[4]={-1,1,1,-1},sy[4]={-1,-1,1,1};
    for(const auto &command:commands) {
        float wheels[4],feedback[3];Controller::InverseKinematics(command,wheels);
        for(unsigned i=0;i<4;++i)near(wheels[i],(sx[i]*command[0]+sy[i]*command[1]+.458f*command[2])/.076f);
        Controller::ForwardKinematics(wheels,feedback);
        for(unsigned i=0;i<3;++i)near(feedback[i],command[i]);
    }
    const float demands[][3]={{20,0,0},{0,-30,0},{0,0,4},{11,-7,3}};
    const float velocity[3]={.7f,-.2f,.8f};float wheels[4];Controller::InverseKinematics(velocity,wheels);
    for(const auto &demand:demands) {
        float torque[4],current[4],actual[3],wp=0,bp=0;Controller::InverseDynamics(demand,torque);
        for(unsigned i=0;i<4;++i){current[i]=torque[i]/Controller::WHEEL_TORQUE_PER_AMP;wp+=torque[i]*wheels[i];}
        reconstruct(current,actual);
        for(unsigned i=0;i<3;++i){near(actual[i],demand[i]);bp+=demand[i]*velocity[i];}
        near(wp,bp);
    }
}
static void feedforward_and_wheel_feedback() {
    Controller control;near(Controller::MASS_KG,13);
    const float zero[4]={},acceleration[2]={2,-1},no_acceleration[2]={};
    float raw[4],current[4],force[3];
    for(unsigned i=0;i<4;++i)control.kp[i]=control.ki[i]=control.kd[i]=0;
    control.Update(zero,acceleration,zero,raw);
    for(unsigned i=0;i<4;++i){current[i]=raw[i]/Controller::RAW_PER_AMP;near(current[i],control.feedforward_current[i]);}
    reconstruct(current,force);near(force[0],26);near(force[1],-13);near(force[2],0);
    control.Update(zero,no_acceleration,zero,raw);for(float value:raw)near(value,0);
    const float yaw[3]={0,0,.5f};float target[4];Controller::InverseKinematics(yaw,target);
    control.Update(target,no_acceleration,zero,raw);for(float value:raw)near(value,0);
    for(unsigned i=0;i<4;++i)control.kp[i]=1;
    control.Update(target,no_acceleration,zero,raw);
    for(unsigned i=0;i<4;++i){near(raw[i]/Controller::RAW_PER_AMP,target[i]);near(control.feedforward_current[i],0);}
    control.Reset();const float one_wheel[4]={1,0,0,0};control.Update(one_wheel,acceleration,zero,raw);
    for(unsigned i=0;i<4;++i){near(control.feedback_current[i],one_wheel[i]);near(raw[i]/Controller::RAW_PER_AMP,control.feedforward_current[i]+control.feedback_current[i]);}
}
static void saturation_and_reset() {
    Controller control;const float target[4]={50,-50,25,-25},zero[4]={},acceleration[2]={4,2};float raw[4];
    for(unsigned step=0;step<1000;++step)control.Update(target,acceleration,zero,raw);
    float peak=0;
    for(unsigned i=0;i<4;++i){near(control.integral[i],0);peak=std::fmax(peak,std::fabs(raw[i]));near(raw[i]/Controller::RAW_PER_AMP,(control.feedback_current[i]+control.feedforward_current[i])*control.output_scale,.001f);}
    near(peak,16384,.01f);
    control.Reset();const float small[4]={.1f,-.1f,.2f,-.2f},no_acceleration[2]={};
    for(unsigned step=0;step<1000;++step){control.Update(small,no_acceleration,zero,raw);control.ApplyOutputScale(.2f);}
    for(float value:control.integral)near(value,0);
    control.integral[0]=1;const float reverse_error[4]={-.1f,0,0,0};
    control.Update(reverse_error,no_acceleration,zero,raw);control.ApplyOutputScale(.5f);assert(control.integral[0]<1);
    control.Reset();control.Update(zero,no_acceleration,zero,raw);
    for(unsigned i=0;i<4;++i){near(raw[i],0);near(control.integral[i],0);near(control.feedforward_current[i],0);near(control.feedback_current[i],0);}
}
static void velocity_plant() {
    Controller control;Alg::Utility::SecondOrderSetpointPlanner x(4,40,.001f),y(4,40,.001f);
    ALG::PowerControl::PowerControl<4> limiter;float velocity[3]={};
    const float inertia[3]={13,13,1.5f},drag[3]={4,4,.8f},disturbance[3]={2,-1,.1f};
    const float references[][3]={{.8f,-.4f,.6f},{0,0,0},{-.5f,.3f,-.4f}};bool braking=false,limited=false;
    for(const auto &reference:references) {
        for(unsigned step=0;step<7000;++step) {
            x.Update(reference[0]);y.Update(reference[1]);
            const float planned[3]={x.GetOut(),y.GetOut(),reference[2]},acceleration[2]={x.GetAcceleration(),y.GetAcceleration()};
            float wheels[4],target[4],raw[4],current[4],rotor[4],other[4]={};
            Controller::InverseKinematics(velocity,wheels);Controller::InverseKinematics(planned,target);
            control.Update(target,acceleration,wheels,raw);braking|=velocity[0]>.1f&&control.force[0]<-1;
            for(unsigned i=0;i<4;++i){current[i]=raw[i]/Controller::RAW_PER_AMP;rotor[i]=wheels[i]*Controller::REDUCTION_RATIO;}
            limiter.DecayingCurrent(current,rotor,poly_coeffs,other,0,50);control.ApplyOutputScale(limiter.getEta());limited|=control.output_scale<.99f;
            float power=0;
            for(unsigned i=0;i<4;++i){current[i]=limiter.getCurrentCalculate(i);assert(std::isfinite(current[i])&&std::fabs(current[i])<=20.0001f);power+=poly_coeffs[0]+poly_coeffs[1]*current[i]+poly_coeffs[2]*std::fabs(rotor[i])+poly_coeffs[3]*current[i]*rotor[i]+poly_coeffs[4]*current[i]*current[i]+poly_coeffs[5]*rotor[i]*rotor[i];}
            assert(power<=50.01f);float force[3];reconstruct(current,force);
            for(unsigned axis=0;axis<3;++axis)velocity[axis]+=.001f*(force[axis]-drag[axis]*velocity[axis]-disturbance[axis])/inertia[axis];
        }
        for(unsigned axis=0;axis<3;++axis)near(velocity[axis],reference[axis],.02f);
    }
    assert(braking&&limited);
}
int main(){geometry();feedforward_and_wheel_feedback();saturation_and_reset();velocity_plant();std::puts("PASS: mecanum geometry, 13 kg feedforward, wheel PID, summed current, saturation/reset and power-limited plant");}
