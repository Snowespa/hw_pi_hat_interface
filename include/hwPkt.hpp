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
  NONE = 0x00,
  PRESSED = 0x01,
  LONGPRESS = 0x02,
};

#endif // __HW_PKT_HPP__
