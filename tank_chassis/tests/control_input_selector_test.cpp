#include "../fsm/control_input_selector.hpp"
#include <cassert>

int main() {
  assert(SelectControlInputSource(1000, true, 901) ==
         ControlInputSource::LOCAL);
  assert(SelectControlInputSource(1000, true, 900) ==
         ControlInputSource::NONE);
  assert(SelectControlInputSource(1000, false, 999) ==
         ControlInputSource::NONE);
  assert(SelectControlInputSource(0, true, UINT32_MAX) ==
         ControlInputSource::LOCAL);
}
