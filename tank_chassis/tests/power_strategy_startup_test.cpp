#include <cstdint>
#include "../user/core/Alg/PowerControl/PowerControl.hpp"
#include <cassert>

int main() {
    ALG::PowerControl::PowerControlStrategy strategy(1600.0f);

    // 启动就离线时使用默认 60 W；在线标志先于有效裁判帧出现时也保持默认值。
    strategy.Update(false, false, 0.0f, 0.0f, 0.0f);
    assert(strategy.GetInputLimit() == 60.0f);
    strategy.Update(false, true, 0.0f, 0.0f, 0.0f);
    assert(strategy.GetInputLimit() == 60.0f);
    strategy.Update(false, false, 0.0f, 0.0f, 0.0f);
    assert(strategy.GetInputLimit() == 60.0f);

    strategy.Update(false, true, 80.0f, 0.0f, 0.0f);
    assert(strategy.GetInputLimit() == 80.0f);
    // 收到过有效上限后，掉线或收到无效 0 W 都沿用最后的 80 W。
    strategy.Update(false, false, 0.0f, 0.0f, 0.0f);
    assert(strategy.GetInputLimit() == 80.0f);
    strategy.Update(false, true, 0.0f, 0.0f, 0.0f);
    assert(strategy.GetInputLimit() == 80.0f);
    strategy.Update(true, false, 0.0f, 0.0f, 500.0f);
    assert(strategy.GetInputLimit() == 80.0f);
}
