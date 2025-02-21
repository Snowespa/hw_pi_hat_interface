#ifndef __HW_PI_HAT_BOARD_HPP__
#define __HW_PI_HAT_BOARD_HPP__

#include <atomic>
#include <queue>
#include <string>
#include <thread>
#include <vector>

class Board {
 public:
  Board(const std::string &device, int baud_rate, int timout);
  ~Board();

 private:
  bool openPort();
  void closePort();

  std::queue<std::vector<unsigned char>> sys_q;
  std::queue<std::vector<unsigned char>> servo_q;
  std::queue<std::vector<unsigned char>> key_q;
  std::queue<std::vector<unsigned char>> sbus_q;
  std::queue<std::vector<unsigned char>> led_q;

  std::thread rcvThread;
  std::atomic<bool> rcv;
  std::string dev;
  int br;
  int timeout;
  int fd;
};
#endif  // !__HW_PI_HAT_BOARD_HPP__
