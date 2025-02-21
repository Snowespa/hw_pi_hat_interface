#include "../include/board.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

Board::Board(const std::string &device, int baud_rate, int timeout)
    : dev(device), br(baud_rate), timeout(timeout), fd(-1) {
  openPort();
}

Board::~Board() {
  rcv = false;
  if (rcvThread.joinable()) rcvThread.join();
  closePort();
}

bool Board::openPort() {
  fd = open(dev.c_str(), O_RDWR | O_NOCTTY);
  if (fd == -1) {
    std::cerr << "Error: could not open serial port: " << strerror(errno)
              << std::endl;
    return false;
  }

  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  if (tcgetattr(fd, &tty) != 0) {
    std::cerr << "Error: getting attributes: " << strerror(errno) << std::endl;
    close(fd);
    return false;
  }

  cfsetospeed(&tty, br);
  cfsetispeed(&tty, br);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  int status;
  ioctl(fd, TIOCMGET, &status);
  status &= ~(TIOCM_DTR, TIOCM_RTS);
  ioctl(fd, TIOCMSET, &status);

  tty.c_cc[VTIME] = timeout / 100;
  tty.c_cc[VMIN] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    std::cerr << "Error: setting attributes: " << strerror(errno) << std::endl;
    close(fd);
    return false;
  }
  std::cout << "Serial port opened successfully !" << std::endl;
  return true;
}

void Board::closePort() {
  if (fd != -1) {
    close(fd);
    fd = -1;
  }
}
