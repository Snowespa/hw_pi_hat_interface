#ifndef __HW_PKT_HPP__
#define __HW_PKT_HPP__

#include <cstdint>
enum class PktContState : uint8_t {
  STARTBYTE0 = 0,
  STARTBYTE1 = 1,
  LENGTH = 2,
  FUNC = 3,
  DATA = 4,
  CHECKSUM = 5,
};

enum class PktFunc : uint8_t {
  SYS = 0,
  LED = 1,
  BUZ = 2,
  BUS_SERVO = 5,
  KEY = 6,
  SBUS = 9,
  RGB = 11,
  NONE = 12,
};

enum class PktEvent : uint8_t {
  PRESSED = 0x01,
  LONGPRESS = 0x02,
  LONGPRESS_REPEAT = 0x04,
  RELEASE_FROM_LONGPRESS = 0x08,
  RELEASE_FROM_PRESS = 0x10,
  CLICK = 0x20,
  DOUBLE_CLICK = 0x40,
  TRIPLE_CLICK = 0x80
};
#endif  // __HW_PKT_HPP__
