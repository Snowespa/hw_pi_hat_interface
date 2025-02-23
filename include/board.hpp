#ifndef __HW_PI_HAT_BOARD_HPP__
#define __HW_PI_HAT_BOARD_HPP__

#include <gpiod.hpp>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

// CRC-8 Lookup Table
constexpr uint8_t CRC8_TABLE[256] = {
    0,   94,  188, 226, 97,  63,  221, 131, 194, 156, 126, 32,  163, 253, 31,
    65,  157, 195, 33,  127, 252, 162, 64,  30,  95,  1,   227, 189, 62,  96,
    130, 220, 35,  125, 159, 193, 66,  28,  254, 160, 225, 191, 93,  3,   128,
    222, 60,  98,  190, 224, 2,   92,  223, 129, 99,  61,  124, 34,  192, 158,
    29,  67,  161, 255, 70,  24,  250, 164, 39,  121, 155, 197, 132, 218, 56,
    102, 229, 187, 89,  7,   219, 133, 103, 57,  186, 228, 6,   88,  25,  71,
    165, 251, 120, 38,  196, 154, 101, 59,  217, 135, 4,   90,  184, 230, 167,
    249, 27,  69,  198, 152, 122, 36,  248, 166, 68,  26,  153, 199, 37,  123,
    58,  100, 134, 216, 91,  5,   231, 185, 140, 210, 48,  110, 237, 179, 81,
    15,  78,  16,  242, 172, 47,  113, 147, 205, 17,  79,  173, 243, 112, 46,
    204, 146, 211, 141, 111, 49,  178, 236, 14,  80,  175, 241, 19,  77,  206,
    144, 114, 44,  109, 51,  209, 143, 12,  82,  176, 238, 50,  108, 142, 208,
    83,  13,  239, 177, 240, 174, 76,  18,  145, 207, 45,  115, 202, 148, 118,
    40,  171, 245, 23,  73,  8,   86,  180, 234, 105, 55,  213, 139, 87,  9,
    235, 181, 54,  104, 138, 212, 149, 203, 41,  119, 244, 170, 72,  22,  233,
    183, 85,  11,  136, 214, 52,  106, 43,  117, 151, 201, 74,  20,  246, 168,
    116, 42,  200, 150, 21,  75,  169, 247, 182, 232, 10,  84,  215, 137, 107,
    53};

class Board {
public:
  /*
   * Consturctor.
   *
   * arguments:
   * ----------
   *    - const std::string &device: the path to the device. (default
   * "/dev/ttyAMA0")
   *    - const int baud_rate: the board baud_rate. (default B1000000)
   *    - const int timeout: the timeout for each message.
   */
  Board(const std::string &device = "/dev/ttyAMA0",
        const std::string &chip = "/dev/gpiochip4", int baud_rate = B1000000,
        int timeout = 500);

  /*
   * Destructor. disable reception, stops the thread and closes the port
   */
  ~Board();

  /*
   * Set pkt reception.
   *
   * arguments:
   * ----------
   *    - const bool enable: if true, enables pkt reception. False to disable
   * them.
   */
  void setRecieve(const bool enable);

  /* SETTERS */
  /*
   * Set the buzzer on the board. Non-blocking, reseting the buzzer before the
   * sound is stopped resets the buzzer.
   *
   * arguments:
   * ----------
   *    - const float on_time: time the buzzer stays on in seconds.
   *    - const float off_time: timer the buzzer stay off in seconds.
   *    - const uint16_t freq: the frequency off the buzzer.
   *    - const uint16_t repeat: the number of cycles that will be executed.
   * default 1.
   */
  void setBuzzer(const float on_time, const float off_time, const uint16_t freq,
                 const uint16_t repeat = 1);
  /*
   * Set Led id on for on_time seconds and off for off_time seconds and repeats
   * the process for repeat itterations.
   *
   * arguments:
   * ----------
   *    - const float on_time: time the led stays on in seconds.
   *    - const float off_time: time the led stays off in seconds.
   *    - const uint16_t repeat: the number of cycles the led will be activated.
   *    - const uint8_t id: the led id. range: [0, 1, 2].
   */
  void setLed(const float on_time, const float off_time,
              const uint16_t repeat = 1, const uint8_t id = 0);
  /*
   * Set the rgb lights on for on_time seconds and off for off_time seconds and
   * repates the process for repeat itterations.
   *
   * arguments:
   * ----------
   *    - const std::vector<std::tuple<uint8_t, uint8_t, uint8_t uint8_t>>
   * pixels: vector representing the rgb led and its rbg content. pixels[i,0]
   * represents the led id. pixels[i, 1:3] is the rgb value.
   */
  void setRGB(const std::vector<std::tuple<uint8_t, uint8_t, uint8_t, uint8_t>>
                  &pixels);

  /*
   * Enable or disable torque mode for servo id.
   *
   * arguments:
   * ----------
   *    - const uint8_t id: the servo id to edit.
   *    - const bool enable: true, enable torque mode. flase, disable torque
   * mode.
   */
  void setServoTorque(const uint8_t id, const bool enable);

  /*
   * Sets the id of a servo.
   *
   * arguments:
   * ----------
   *    - const uint8_t old_id: the current id of the servo.
   *    - const uint8_t new_id: the new desired servo id.
   */
  void setServoId(const uint8_t old_id, const uint8_t new_id);

  /*
   * Sets the servo offset angle on servo id.
   *
   * arguments:
   * ----------
   *    - const uint8_t id: the servo id.
   *    - const int8_t offset: the angle offset.
   */
  void setServoOffset(const uint8_t id, const int8_t offset);

  /*
   * Set servo angle limit onf servo id.
   *
   * arguments:
   * ----------
   *    - const uint8_t id: the servo id.
   *    - const std::pair<uint16_t, uint16_t> lim: the min and max limit.
   */
  void setServoAngleLimit(const uint8_t id,
                          const std::pair<uint16_t, uint16_t> &lim);

  /*
   * Set servo Volt input limit.
   *
   * arguments:
   * ----------
   *    - const uint8_t id: the servo id.
   *    - const std::tuple<uint16_t, uint16_t> lim: the min and max limit.
   */
  void setServoVinLim(const uint8_t id,
                      const std::pair<uint16_t, uint16_t> &lim);

  /*
   * Set servo temperature limit.
   *
   * arguments:
   * ----------
   *    - const uint8_t id: the servo id.
   *    - const uint8_t temp: the temperature limit.
   */
  void setServoTempLim(const uint8_t id, const int8_t temp);

  /*
   * Set servos position.
   *
   * arguments:
   * ----------
   *    - const std::vector<uint8_t> &ids: the servo ids.
   *    - const std::vector<uint16_t> &angles: the angles of each servo.
   *    - const float duration: the desired time to move to the desired angle.
   * If set to 0, the servo will move as fast as possible.
   */
  void setServoPos(const std::vector<uint8_t> &ids,
                   const std::vector<uint16_t> &angles, const float duration);

  /*
   * Stops all servo in ids.
   *
   * arguments:
   * ----------
   *    - const std::vector<uint8_t> &ids: vector containing the servo ids.
   */
  void stopServo(const std::vector<uint8_t> &ids);

  /*
   * Save the servo offset of servo id.
   *
   * arguments:
   * ----------
   *    - const uint8_t id: the servo id.
   */
  void saveServoOffset(const uint8_t id);

  /* GETTERS */

  /*
   * get battery status if available.
   *
   * returns:
   * --------
   *    - std::optional<uint16_t>: battery voltage.
   */
  std::optional<uint16_t> getBattery();

  /*
   * get button status if available.
   *
   * returns:
   * --------
   *    - std::optional<std::pair<uint8_t, uint8_t>>: button id and event.
   */
  std::optional<std::pair<uint8_t, uint8_t>> getButton();

  /*
   * get servo id. Requests servo id (id) on the bus. If available returns id.
   * If not available returns empty optional. Id 254 corresponds to the
   * broadcasting message and shoould return all servo available on the bus.
   * (Not working at the moment.)
   *
   * arguments:
   * ----------
   *    - const uint8_t id. The servo id (default 254).
   *
   * returns:
   * --------
   *    - std::optional<uint8_t>: the servo id.
   */
  std::optional<uint8_t> getServoId(const uint8_t id = 254);

  /*
   * get servo id's offset.
   *
   * arguments:
   * ----------
   *    - const uint8_t id. The servo id.
   *
   * returns:
   * --------
   *    - std::optional<int8_t>: the servo offset.
   */
  std::optional<int8_t> getServoOffset(const uint8_t id);

  /*
   * get servo id's position.
   *
   * arguments:
   * ----------
   *    - const uint8_t id: the servo id.
   *
   * returns:
   * --------
   *    - std::optional<uint16_t> the servo position.
   */
  std::optional<uint16_t> getServoPos(const uint8_t id);

  /*
   * get servo angle limit.
   *
   * arguments:
   * ----------
   *     const uint8_t id: the servo id.
   *
   * returns:
   * --------
   *    - std::optional<std::pair<uint16_t, uint16_t>>: the high and low limit.
   */
  std::optional<std::pair<uint16_t, uint16_t>>
  getServoAngleLim(const uint8_t id);

  /*
   * get servo voltage limit.
   *
   * arguments:
   * ----------
   *    - const uint8_t id: the servo id.
   *
   * returns:
   * --------
   *    - std::optional<std::pair<uint16_t, uint16_t>>: the high and low limit.
   */
  std::optional<std::tuple<uint16_t, uint16_t>>
  getServoVinLim(const uint8_t id);

  /*
   * get servo voltage.
   *
   * arguments:
   * ----------
   *    - const uint8_t id: the servo id.
   *
   * returns:
   * --------
   *    - std::optional<uint16_t>: the servo voltage.
   */
  std::optional<uint16_t> getServoVin(const uint8_t id);

  /*
   * get the servo temperature.
   *
   * arguments:
   * ----------
   *    - const uint8_t id: the servo id.
   *
   * returns:
   * --------
   *    - std::optional<uint16_t>: the servo temperature.
   */
  std::optional<uint8_t> getServoTemp(const uint8_t id);

  /*
   * get servo temperature limit.
   *
   * arguments:
   * ----------
   *    - const uint8_t id: the servo id.
   *
   * returns:
   * --------
   *    - std::optional<uint8_t>: the serrvo temperature limit.
   */
  std::optional<uint8_t> getServoTempLim(const uint8_t id);

  /*
   * get the servo torque state.
   *
   * arguments:
   * ----------
   *    - const uint8_t id: the servo id.
   *
   * returns:
   * --------
   *    - std::optional<bool>: true if servo in torque mode, false otherwise.
   */
  std::optional<bool> getServoTorque(const uint8_t id);

private:
  /*
   * opens device port.
   */
  bool openPort();

  /*
   * Closes device port.
   */
  void closePort();

  /*
   * Sends pkt from raspberry pi to the hw-pi-hat board.
   *
   * arguments:
   * ----------
   *    - uint8_t func: the board function to use. All functions are defined in
   * the PktFunc enum class.
   */
  void sendPkt(uint8_t func, const std::vector<uint8_t> &data);

  /*
   * Recieve pkt function. Listenst on the device, parses packets and puts the
   * results in the right queue.
   */
  void rcvPkt();

  /*
   * compute the CRC8 checksum of a given data vector.
   *
   * arguments:
   * ----------
   *    - const std::vector<uint8_t> &data: the data vector.
   *
   * returns:
   * --------
   *    - uint8_t: the checksum value.
   */
  uint8_t checksumCRC8(const std::vector<uint8_t> &data);

  /*
   *
   */
  void listentButton();

  std::queue<std::vector<uint8_t>> sysQ;
  std::queue<std::vector<uint8_t>> servoQ;
  std::queue<std::vector<uint8_t>> keyQ;
  std::queue<std::vector<uint8_t>> sbusQ;
  std::queue<std::vector<uint8_t>> ledQ;

  std::thread rcvThread;
  std::atomic<bool> rcv;
  std::string dev;
  gpiod::chip chip;
  gpiod::line_request request;

  const int key1_pin = 13;
  const int key2_pin = 23;

  int br;
  int timeout;
  int fd;
};
#endif // !__HW_PI_HAT_BOARD_HPP__
