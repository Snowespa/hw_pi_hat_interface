#include "../include/board.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <ios>
#include <iostream>
#include <vector>

#include "../include/hwPkt.hpp"

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

unsigned char Board::checksumCRC8(const std::vector<unsigned char> &data) {
  unsigned char chk = 0;
  for (auto d : data) {
    chk = CRC8_TABLE[chk ^ d];
  }
  return chk;
}

void Board::sendPkt(unsigned char func,
                    const std::vector<unsigned char> &data) {
  std::vector<unsigned char> buf{0xAA, 0x55, func};
  buf.push_back(static_cast<unsigned char>(data.size()));
  buf.insert(buf.end(), data.begin(), data.end());
  unsigned char crc8 =
      checksumCRC8(std::vector<unsigned char>(buf.begin() + 2, buf.end()));
  buf.push_back(crc8);
  write(fd, buf.data(), sizeof(buf));
}

void Board::setBuzzer(const float on_time, const float off_time,
                      const uint16_t freq, const uint16_t repeat) {
  uint16_t on_t = static_cast<uint16_t>(on_time * 1000);
  uint16_t off_t = static_cast<uint16_t>(off_time * 1000);

  std::vector<uint8_t> data = {
      static_cast<uint8_t>(freq & 0x00FF),
      static_cast<uint8_t>((freq & 0xFF00) >> 8),
      static_cast<uint8_t>(on_t & 0x00FF),
      static_cast<uint8_t>((on_t & 0xFF00) >> 8),
      static_cast<uint8_t>(off_t & 0x00FF),
      static_cast<uint8_t>((off_t & 0xFF00) >> 8),
      static_cast<uint8_t>(repeat & 0x00FF),
      static_cast<uint8_t>((repeat & 0xFF00) >> 8),
  };
  for (auto d : data) {
    std::cout << std::hex << unsigned(d) << " ";
  }
  std::cout << std::endl;
  sendPkt(static_cast<uint8_t>(PktFunc::BUZ), data);
}
