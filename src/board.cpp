#include "../include/board.hpp"

#include <bits/types/struct_timeval.h>
#include <fcntl.h>
#include <gpiod.hpp>
#include <iterator>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <optional>
#include <ostream>
#include <thread>
#include <utility>
#include <vector>

#include "../include/hwPkt.hpp"

Board::Board(const std::string &device, const std::string &chip, int baud_rate,
             int timeout)
    : dev(device), br(baud_rate), timeout(timeout), fd(-1), rcvSerial(false),
      chip(chip), rcvIO(false) {
  openPort();
  openGPIO();
  rcvSerialThread = std::thread(&Board::rcvPkt, this);
  rcvIOThread = std::thread(&Board::rcvGPIO, this);
}

Board::~Board() {
  rcvSerial = false;
  rcvIO = false;
  if (rcvSerialThread.joinable())
    rcvSerialThread.join();
  if (rcvIOThread.joinable())
    rcvIOThread.join();
  closePort();
  closeGPIO();
}

/* PRIVATE FUNCTIONS */
bool Board::openPort() {
  std::cout << "Opening serial port and initializing GPIO" << std::endl;
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

bool Board::openGPIO() {
  rcvIO = true;
  return true;
}

void Board::closePort() {
  if (fd != -1) {
    close(fd);
    fd = -1;
  }
}

void Board::closeGPIO() { chip.close(); }

uint8_t Board::checksumCRC8(const std::vector<uint8_t> &data) {
  unsigned char chk = 0;
  for (auto d : data) {
    chk = CRC8_TABLE[chk ^ d];
  }
  return chk;
}

void Board::rcvPkt() {
  std::vector<uint8_t> frame;
  PktContState state = PktContState::STARTBYTE0;
  size_t rcv_count = 0;

  fcntl(fd, F_SETFL, O_NONBLOCK);

  fd_set read_fds;
  struct timeval timeout;
  std::cout << "Started Serial Thread! " << std::endl;
  while (rcvSerial) {
    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);

    timeout.tv_sec = 5;
    timeout.tv_usec = 0;

    int result = select(fd + 1, &read_fds, nullptr, nullptr, &timeout);

    if (result > 0) {
      if (FD_ISSET(fd, &read_fds)) {
        uint8_t buf[256];
        int bytes_read = read(fd, buf, sizeof(buf));

        if (bytes_read > 0) {
          frame.clear();
          rcv_count = 0;

          for (int i = 0; i < bytes_read; i++) {
            uint8_t byte = buf[i];

            switch (state) {
            case PktContState::STARTBYTE0:
              if (byte == 0xAA)
                state = PktContState::STARTBYTE1;
              break;
            case PktContState::STARTBYTE1:
              if (byte == 0x55) {
                state = PktContState::FUNC;
              } else {
                state = PktContState::STARTBYTE0;
              }
              break;
            case PktContState::FUNC:
              if (byte < static_cast<uint8_t>(PktFunc::NONE)) {
                frame.push_back(byte);
                state = PktContState::LENGTH;
              } else {
                state = PktContState::STARTBYTE0;
              }
              break;
            case PktContState::LENGTH:
              frame.push_back(byte);
              state = (byte == 0) ? PktContState::CHECKSUM : PktContState::DATA;
              break;
            case PktContState::DATA:
              frame.push_back(byte);
              rcv_count++;
              if (rcv_count >= frame[1])
                state = PktContState::CHECKSUM;
              break;
            case PktContState::CHECKSUM:
              if (checksumCRC8(frame) != byte) {
                std::cerr << "Checksum Failed!" << std::endl;
                state = PktContState::STARTBYTE0;
                break;
              }
              PktFunc func = static_cast<PktFunc>(frame[0]);
              std::vector<uint8_t> pkt_data(frame.begin() + 2, frame.end());
              switch (func) {
              case PktFunc::SYS:
                sysQ.push(pkt_data);
                break;
              case PktFunc::LED:
                ledQ.push(pkt_data);
                break;
              case PktFunc::KEY:
                keyQ.push(pkt_data);
                break;
              case PktFunc::SBUS:
                sbusQ.push(pkt_data);
                break;
              case PktFunc::BUS_SERVO:
                servoQ.push(pkt_data);
              default:
                std::cerr << "Unknown packet type" << std::endl;
                break;
              }
              state = PktContState::STARTBYTE0;
              break;
            }
          }
        }
      }
    } else if (result == 0) {
      // Timeout reached, no data recived
      continue;
    } else {
      std::cerr << "Error in select()!" << std::endl;
    }
  }
}

void Board::sendPkt(uint8_t func, const std::vector<uint8_t> &data) {
  std::vector<uint8_t> buf{0xAA, 0x55, func};
  buf.push_back(static_cast<uint8_t>(data.size()));
  buf.insert(buf.end(), data.begin(), data.end());
  uint8_t crc8 = checksumCRC8(std::vector<uint8_t>(buf.begin() + 2, buf.end()));
  buf.push_back(crc8);
  write(fd, buf.data(), sizeof(buf));
}

/* SETTERS */
void Board::setRecieve(const bool enable) {
  rcvSerial = enable;
  rcvIO = enable;
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
  sendPkt(static_cast<uint8_t>(PktFunc::BUZ), data);
}

void Board::setLed(const float on_time, const float off_time,
                   const uint16_t repeat, const uint8_t id) {
  uint16_t on_t = static_cast<uint16_t>(on_time * 1000);
  uint16_t off_t = static_cast<uint16_t>(off_time * 1000);

  std::vector<uint8_t> data = {
      id,
      static_cast<uint8_t>(on_t & 0x00FF),
      static_cast<uint8_t>((on_t & 0xFF00) >> 8),
      static_cast<uint8_t>(off_t & 0x00FF),
      static_cast<uint8_t>((off_t & 0xFF00) >> 8),
      static_cast<uint8_t>(repeat & 0x00FF),
      static_cast<uint8_t>((repeat & 0xFF00) >> 8),
  };
  sendPkt(static_cast<uint8_t>(PktFunc::LED), data);
}

void Board::setRGB(
    const std::vector<std::tuple<uint8_t, uint8_t, uint8_t, uint8_t>> &pixels) {
  std::vector<uint8_t> data = {0x01, static_cast<uint8_t>(pixels.size())};
  for (const auto &[id, r, g, b] : pixels) {
    data.push_back(id);
    data.push_back(r);
    data.push_back(g);
    data.push_back(b);
  }
  sendPkt(static_cast<uint8_t>(PktFunc::RGB), data);
}

void Board::setServoTorque(const uint8_t id, const bool enable) {
  uint8_t mode = enable ? 0x0B : 0x0C;
  std::vector<uint8_t> data = {mode, id};
  sendPkt(static_cast<uint8_t>(PktFunc::BUS_SERVO), data);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void Board::setServoId(const uint8_t old_id, const uint8_t new_id) {
  std::vector<uint8_t> data = {0x10, old_id, new_id};
  sendPkt(static_cast<uint8_t>(PktFunc::BUS_SERVO), data);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void Board::setServoOffset(const uint8_t id, const int8_t offset) {
  std::vector<uint8_t> data = {0x20, id, static_cast<uint8_t>(offset)};
  sendPkt(static_cast<uint8_t>(PktFunc::BUS_SERVO), data);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void Board::setServoAngleLimit(const uint8_t id,
                               const std::pair<uint16_t, uint16_t> &lim) {
  std::vector<uint8_t> data = {
      0x30,
      id,
      static_cast<uint8_t>(lim.first & 0x00FF),
      static_cast<uint8_t>((lim.first & 0xFF00) >> 8),
      static_cast<uint8_t>(lim.second & 0x00FF),
      static_cast<uint8_t>((lim.second & 0xFF00) >> 8),
  };
  sendPkt(static_cast<uint8_t>(PktFunc::BUS_SERVO), data);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void Board::setServoVinLim(const uint8_t id,
                           const std::pair<uint16_t, uint16_t> &lim) {
  std::vector<uint8_t> data = {
      0x34,
      id,
      static_cast<uint8_t>(lim.first & 0x00FF),
      static_cast<uint8_t>((lim.first & 0xFF00) >> 8),
      static_cast<uint8_t>(lim.second & 0x00FF),
      static_cast<uint8_t>((lim.second & 0xFF00) >> 8),
  };
  sendPkt(static_cast<uint8_t>(PktFunc::BUS_SERVO), data);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void Board::setServoTempLim(const uint8_t id, const int8_t temp) {
  std::vector<uint8_t> data = {0x38, id, static_cast<uint8_t>(temp)};
  sendPkt(static_cast<uint8_t>(PktFunc::BUS_SERVO), data);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void Board::setServoPos(const std::vector<uint8_t> &ids,
                        const std::vector<uint16_t> &angles,
                        const float duration) {
  uint16_t dur = static_cast<uint16_t>(duration * 1000);
  std::vector<uint8_t> data{0x01, static_cast<uint8_t>(dur & 0x00FF),
                            static_cast<uint8_t>((dur & 0xFF00) >> 8),
                            static_cast<uint8_t>(angles.size())};

  for (int i = 0; i < angles.size(); i++) {
    data.push_back(ids[i]);
    data.push_back(static_cast<uint8_t>(angles[i] & 0x00FF));
    data.push_back(static_cast<uint8_t>((angles[i] & 0xFF00) >> 8));
  }
  sendPkt(static_cast<uint8_t>(PktFunc::BUS_SERVO), data);
}

/* GETTERS */
std::optional<uint16_t> Board::getBattery() {
  if (!rcvSerial) {
    std::cerr << "Enable Message Reception First!" << std::endl;
    return std::nullopt;
  }

  if (sysQ.empty()) {
    std::cerr << "No Battery message available " << std::endl;
    return std::nullopt;
  }

  std::vector<uint8_t> data = sysQ.front();
  sysQ.pop();
  if (data[0] == 0x04) {
    uint16_t battery =
        static_cast<uint16_t>(data[1]) | (static_cast<uint16_t>(data[2]) << 8);
    return battery;
  }

  std::cerr << "Did not recognize the message " << std::endl;
  return std::nullopt;
}

std::optional<std::pair<uint8_t, uint8_t>> Board::getButton() {
  if (!rcvSerial) {
    std::cerr << "Enable Message Reception First!" << std::endl;
    return std::nullopt;
  }

  if (keyQ.empty()) {
    std::cerr << "No Button message available " << std::endl;
    return std::nullopt;
  }

  std::vector<uint8_t> data = keyQ.front();
  keyQ.pop();
  uint8_t key = data[0];
  uint8_t event = data[1];
  return std::pair<uint8_t, uint8_t>(key, event);
}

void Board::rcvGPIO() {
  bool edge(false);
  try {
    gpiod::line_settings config;
    config.set_direction(gpiod::line::direction::INPUT);
    config.set_bias(gpiod::line::bias::PULL_UP);
    config.set_edge_detection(gpiod::line::edge::BOTH);

    gpiod::line_request request = chip.prepare_request()
                                      .set_consumer("key_buttons")
                                      .add_line_settings(key1_pin, config)
                                      .add_line_settings(key2_pin, config)
                                      .do_request();
    std::cout << "Start GPIO thread" << std::endl;
    gpiod::edge_event_buffer buf(2);
    while (rcvIO) {
      edge = request.wait_edge_events(std::chrono::milliseconds(100));
      if (edge) {
        int read = request.read_edge_events(buf);
        for (gpiod::edge_event_buffer::const_iterator it = buf.begin();
             it != buf.end(); it++) {
          buttonCB(*it);
        }
      } else {
        std::cout << "No IO activated!" << std::endl;
      }
    }

    request.release();
  } catch (const std::exception &e) {
    std::cerr << "Failed to aquire GPIO Pins: " << e.what() << std::endl;
  }
}

void Board::buttonCB(gpiod::edge_event e) {
  uint8_t key = e.line_offset();
  uint64_t time = e.timestamp_ns();
  bool type(e.type() == gpiod::edge_event::event_type::FALLING_EDGE);
}
