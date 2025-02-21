#include <termios.h>

#include "../include/board.hpp"

int main(int argc, char *argv[]) {
  Board board("/dev/ttyAMA0", B1000000, 500);

  return 0;
}
