#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include "user/core/Alg/PowerControl/ChassisPowerAllocator.hpp"
#include "user/core/Alg/ChassisCalculation/MecanumForceControl.hpp"
#include "task_power_coefficients.hpp"

using Allocator = ALG::PowerControl::ChassisPowerAllocator;
using Controller = Alg::CalculationBase::MecanumForceControl;

static void near(float actual, float expected, float tolerance = 0.001f)
{
    assert(std::isfinite(actual));
    if (std::fabs(actual - expected) > tolerance)
    {
        std::fprintf(stderr,"actual=%g expected=%g\n", actual, expected);
        assert(false);
    }
}

static float power(const float current[4], const float speed[4], const float k[6])
{
    float sum = 0;
    for (unsigned i = 0; i < 4; ++i)
        sum += k[0] + k[1]*current[i] + k[2]*std::fabs(speed[i]) +
               k[3]*current[i]*speed[i] + k[4]*current[i]*current[i] + k[5]*speed[i]*speed[i];
    return sum;
}

static void check(const Allocator &a, const float t[4], const float r[4], const float s[4],
                  const float speed[4], const float k[6], float budget)
{
    for (float scale : {a.alpha, a.beta, a.gamma}) assert(scale >= 0 && scale <= 1);
    assert(a.beta == a.gamma);
    for (unsigned i = 0; i < 4; ++i)
    {
        near(a.current[i], a.alpha*t[i] + a.beta*r[i] + a.gamma*s[i]);
        assert(std::fabs(a.current[i]) <= 20.0001f);
    }
    near(a.predicted_power, power(a.current, speed, k), 0.003f);
    if (a.feasible) assert(a.predicted_power <= budget + 0.005f);
}

int main()
{
    Allocator a;
    const float zero[4] = {};
    const float copper[6] = {0, 0, 0, 0, 1, 0};
    const float t[4] = {-10,10,10,-10}, r[4] = {3,3,3,3};
    const float sync[4] = {1,-1,1,-1};
    a.Allocate(t,r,zero,zero,copper,100,20);
    near(a.beta,1);
    near(a.alpha,.4f);
    near(a.predicted_power,100);
    // The resulting yaw sum stays at 12 A, even while translation is limited.
    near(a.current[0]+a.current[1]+a.current[2]+a.current[3],12);
    a.Allocate(t,r,sync,zero,copper,1000,20);
    near(a.alpha,1); near(a.beta,1); near(a.gamma,1);
    a.Allocate(t,r,zero,zero,copper,10000,5);
    near(a.beta,1); near(a.alpha,.2f);
    a.Allocate(zero,r,zero,zero,copper,16,20);
    near(a.beta,2.f/3);
    a.Allocate(t,zero,zero,zero,copper,100,20);
    near(a.alpha,.5f);
    // Translation yields budget to both yaw and synchronization.
    a.Allocate(t,r,sync,zero,copper,100,20);
    near(a.beta,1); near(a.gamma,1); near(a.alpha,std::sqrt(.15f));
    check(a,t,r,sync,zero,copper,100);
    // Insufficient correction budget reduces yaw and synchronization together.
    a.Allocate(t,r,sync,zero,copper,16,20);
    near(a.beta,std::sqrt(.4f)); near(a.gamma,a.beta); near(a.alpha,0);
    check(a,t,r,sync,zero,copper,16);
    // A single-wheel current limit must also preserve their common scale.
    a.Allocate(zero,r,sync,zero,copper,10000,2);
    near(a.beta,.5f); near(a.gamma,.5f);
    for(float value:a.current) assert(std::fabs(value)<=2.0001f);
    // Synchronization is preserved while translation consumes the remainder.
    a.Allocate(t,zero,sync,zero,copper,100,20);
    near(a.gamma,1); near(a.alpha,std::sqrt(.24f));
    check(a,t,zero,sync,zero,copper,100);

    // A double root that rounds upward to float must move back below the limit.
    const float unit[4] = {1,1,1,1};
    const float rounding_budget = .3f;
    const double boundary = std::sqrt(static_cast<double>(rounding_budget) / 4.0);
    assert(static_cast<float>(boundary) > boundary);
    a.Allocate(zero,unit,zero,zero,copper,rounding_budget,20);
    assert(a.feasible && a.beta <= boundary && a.beta == a.gamma);
    near(a.beta,static_cast<float>(boundary));

    const float linear[6] = {0,1,0,0,0,0};
    a.Allocate(zero,r,zero,zero,linear,6,20);
    near(a.beta,.5f);
    // A regenerative solution can have an allowed interval excluding zero.
    const float regen[6] = {10,0,0,1,1,0};
    const float braking[4] = {-10,-10,-10,-10}, speed[4] = {10,10,10,10};
    a.Allocate(zero,braking,zero,speed,regen,0,20);
    assert(a.feasible); near(a.beta,(1+std::sqrt(.6f))/2);
    check(a,zero,braking,zero,speed,regen,0);
    // Translation may provide the braking needed for feasibility.
    a.Allocate(braking,zero,zero,speed,regen,0,20);
    assert(a.feasible);
    check(a,braking,zero,zero,speed,regen,0);
    const float idle[6] = {10,0,0,0,1,0};
    a.Allocate(zero,zero,zero,zero,idle,20,20);
    assert(!a.feasible); near(a.predicted_power,40);
    for (float value : a.current) near(value,0);

    // Real force-controller requests reach the allocator before uniform clipping.
    Controller control;
    const float target[3] = {3.75f,0,1}, feedback[3] = {};
    float raw[4], translation[4], rotation[4];
    control.Update(target,feedback,raw,false);
    control.GetCurrentComponents(translation,rotation);
    assert(std::fabs(raw[0]) > 16384 || std::fabs(raw[1]) > 16384);
    a.Allocate(translation,rotation,zero,zero,poly_coeffs,80,20);
    near(a.beta,1); assert(a.alpha < 1);
    check(a,translation,rotation,zero,zero,poly_coeffs,80);
    // Only saturated axes roll back their integral increments.
    control.ki[2]=1;
    const float translation_integral=control.integral[0];
    control.Update(target,feedback,raw,false);
    const float yaw_integral=control.integral[2];
    control.ApplyAllocationScale(.1f,1);
    near(control.integral[0],translation_integral); near(control.integral[2],yaw_integral);

    std::mt19937 random(42);
    std::uniform_real_distribution<float> current(-40,40), rotor(-600,600), budget(10,200);
    for (unsigned sample=0;sample<20000;++sample)
    {
        float trans[4],rot[4],s[4],w[4];
        const float yaw=current(random),q=current(random)*.1f;
        const float x=current(random),y=current(random);
        const float sx[4]={-1,1,1,-1},sy[4]={-1,-1,1,1},sn[4]={1,-1,1,-1};
        for(unsigned i=0;i<4;++i)
        {
            trans[i]=sx[i]*x+sy[i]*y; rot[i]=yaw; s[i]=sn[i]*q; w[i]=rotor(random);
        }
        const float limit=budget(random);
        a.Allocate(trans,rot,s,w,poly_coeffs,limit,20);
        if(power(zero,w,poly_coeffs)<limit-.01f) assert(a.feasible);
        check(a,trans,rot,s,w,poly_coeffs,limit);
        // A feasible zero-current point guarantees the priority path can run.
        // Regenerative fallback scales the whole request along a different path.
        if(a.feasible && a.alpha<.999f && power(zero,w,poly_coeffs)<limit-.01f)
        {
            float more[4];
            for(unsigned i=0;i<4;++i)
                more[i]=a.beta*rot[i]+a.gamma*s[i]+(a.alpha+.001f)*trans[i];
            bool over_current=false;
            for(float value:more) over_current |= std::fabs(value)>20;
            assert(over_current || power(more,w,poly_coeffs)>limit-.005f);
        }
    }
    std::puts("PASS: shared yaw/sync priority, current/power limits, regeneration, infeasible budgets, 20000 dynamic cases");
}
