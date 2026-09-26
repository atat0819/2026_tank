#include "../fsm/control_input_selector.hpp"
#include <cassert>

int main() {
  assert(SelectControlInputSource(1000, true, 950, true, 990) ==
         ControlInputSource::LOCAL);
  assert(SelectControlInputSource(1000, true, 899, true, 990) ==
         ControlInputSource::GIMBAL);
  assert(SelectControlInputSource(1000, false, 999, true, 899) ==
         ControlInputSource::NONE);
  assert(SelectControlInputSource(1000, false, 999, false, 999) ==
         ControlInputSource::NONE);
}
