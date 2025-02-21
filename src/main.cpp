#include <termios.h>

#include "../include/board.hpp"

int main(int argc, char *argv[]) {
  Board board("/dev/ttyAMA0", B1000000, 500);

  board.setBuzzer(1.0, 0.1, 1000, 3);

  return 0;
}
