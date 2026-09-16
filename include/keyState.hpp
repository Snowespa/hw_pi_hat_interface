#ifndef __KEY_STATE_HPP__
#define __KEY_STATE_HPP__

#include "hwPkt.hpp"
#include <cstdint>

struct key_state {
  bool value; // True means the button is in pressed mode.
  uint64_t time;
  PktEvent type;
};

#endif // !__KEY_STATE_HPP__
