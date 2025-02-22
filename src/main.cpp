#include <termios.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>

#include "../include/board.hpp"

int main(int argc, char *argv[]) {
  Board board("/dev/ttyAMA0", B1000000, 500);
  sleep(2);

  std::cout << board.getBattery() << std::endl;
  uint16_t freq[3] = {1000, 720, 950};
  float time[3] = {0.4, 0.6, 0.7};

  for (int i = 0; i < 3; i++) {
    board.setBuzzer(time[i], 0., freq[i]);
    usleep(time[i] * 1000000);
  }
  board.setBuzzer(1., 1., 0.);

  return 0;
}
