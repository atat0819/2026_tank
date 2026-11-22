#include "../user/core/Alg/PID/pid.hpp"
#include <cstdio>
#include <initializer_list>

int main()
{
    using ALG::PID::PID;
    int failures = 0;
    const auto expect = [&failures](const char* name, float actual, float expected)
    {
        if (actual != expected)
        {
            std::fprintf(stderr, "%s: expected %g, got %g\n", name, expected, actual);
            ++failures;
        }
    };

    PID initial(0, 1, 1, 10, 0, 0);
    expect("new PID has zero output", initial.getOutput(), 0);
    expect("new PID has zero error", initial.getError(), 0);
    expect("first update starts with zero integral and previous error", initial.UpDate(2, 0), 4);

    for (const float direction : {1.0f, -1.0f})
    {
        PID separated(5, 1, 0, 20, 10, 5);
        separated.reset();
        separated.UpDate(2 * direction, 0);
        expect("separated error still clips output", separated.UpDate(5 * direction, 0), 20 * direction);
        expect("separated error preserves integral", separated.UpDate(0, 0), 2 * direction);

        PID limited(1, 1, 0, 20, 10, 0);
        limited.reset();
        limited.UpDate(9 * direction, 0);
        expect("saturated output remains clipped", limited.UpDate(12 * direction, 0), 20 * direction);
        expect("rollback restores pre-clamp integral", limited.UpDate(0, 0), 9 * direction);

        PID unwinding(6, -1, 0, 10, 10, 0);
        unwinding.reset();
        expect("unwinding can still leave output clipped", unwinding.UpDate(3 * direction, 0), 10 * direction);
        expect("unwinding integral is retained", unwinding.UpDate(0, 0), -3 * direction);
    }

    PID at_limit(1, 1, 0, 10, 10, 0);
    at_limit.reset();
    expect("output can reach its limit", at_limit.UpDate(5, 0), 10);
    expect("exactly reaching the limit retains integral", at_limit.UpDate(0, 0), 5);

    return failures == 0 ? 0 : 1;
}
