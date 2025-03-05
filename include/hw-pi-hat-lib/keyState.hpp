#ifndef __KEY_STATE_HPP__
#define __KEY_STATE_HPP__

#include "hwPkt.hpp"
#include <cstdint>
typedef struct {
  bool value; // True means the button is in pressed mode.
  uint64_t time;
  PktEvent type;
} key_state;

#endif // !__KEY_STATE_HPP__
