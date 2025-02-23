#ifndef __KEY_STATE_HPP__
#define __KEY_STATE_HPP__

#include <cstdint>
typedef struct {
  bool value;
  uint64_t time;
  uint8_t clicks;
  uint8_t long_clicks;
} key_state;

#endif // !__KEY_STATE_HPP__
