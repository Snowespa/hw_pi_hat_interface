#include <iterator>
#include <termios.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>
#include <optional>
#include <ostream>
#include <tuple>
#include <vector>

#include "../include/board.hpp"
#include "../third_party/CLI11.hpp"

void print_key_event(std::pair<uint8_t, uint8_t> key_event) {
  std::cout << unsigned(key_event.first)
            << " event: " << unsigned(key_event.second) << std::endl;
}

int main(int argc, char **argv) {
  CLI::App app{"Test Board Interface"};
  bool startup = false;
  app.add_flag("-s,--startup", startup, "Emits the startup sound");
  bool button = false;
  app.add_flag("-b,--button", button, "Test the button callback");
  bool status = false;
  app.add_flag("-r,--read", status, "Report the servo's status");

  CLI11_PARSE(app, argc, argv);

  Board board;
  board.setRecieve(true);
  sleep(1);

  std::optional<uint16_t> battery = board.getBattery();
  if (battery) {
    std::cout << "Battery : " << unsigned(*battery) << std::endl;
  } else {
    std::cout << "No battery message available !" << std::endl;
  }

  if (startup) {
    uint16_t freq[3] = {550, 650, 850};
    float time[3] = {0.2, 0.2, 0.2};
    std::vector<std::vector<std::tuple<uint8_t, uint8_t, uint8_t, uint8_t>>>
        pixels = {{{0, 255, 0, 255}}, {{0, 255, 255, 0}}, {{0, 0, 255, 255}}

        };
    for (int i = 0; i < 3; i++) {
      board.setBuzzer(time[i], 0., freq[i]);
      board.setRGB(pixels[i]);
      usleep(time[i] * 1000000);
    }
    board.setBuzzer(1., 1., 0.);
    board.setRGB({{0, 0, 0, 0}});
    std::cout << "Good Morning!" << std::endl;
  }

  if (button) {
    std::optional<std::pair<uint8_t, uint8_t>> key_event = board.getButton();
    if (key_event) {
      print_key_event(*key_event);
    } else {
      std::cout << "No value returned" << std::endl;
    }
    sleep(5);
    key_event = board.getButton();
    if (key_event) {
      print_key_event(*key_event);
    } else {
      std::cout << "No button pressed!" << std::endl;
    }
  }

  if (status) {
    std::vector<uint8_t> expected_ids{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

    std::optional<uint8_t> id;
    std::vector<uint8_t> ids;

    std::optional<uint8_t> offset;
    std::vector<int8_t> offsets;

    std::optional<uint8_t> pos;
    std::vector<uint16_t> positions;

    std::optional<std::pair<uint16_t, uint16_t>> angle_lims;
    std::vector<std::pair<uint16_t, uint16_t>> angles_lims;

    std::optional<std::pair<uint16_t, uint16_t>> vin_lims;
    std::vector<std::pair<uint16_t, uint16_t>> vins_lims;

    std::optional<uint16_t> vin;
    std::vector<uint16_t> vins;

    std::optional<uint8_t> temp;
    std::vector<uint8_t> temps;

    std::optional<uint8_t> temp_lims;
    std::vector<uint8_t> temps_lims;

    std::optional<bool> torque;
    std::vector<bool> torques;

    for (std::vector<uint8_t>::iterator it = expected_ids.begin();
         it != expected_ids.end(); it++) {
      id = board.getServoId(*it);
      if (id) {
        ids.push_back(id.value());

        offset = board.getServoOffset(id.value());
        if (offset)
          offsets.push_back(offset.value());

        pos = board.getServoPos(id.value());
        if (pos)
          positions.push_back(pos.value());

        angle_lims = board.getServoAngleLim(id.value());
        if (angle_lims)
          angles_lims.push_back(angle_lims.value());

        vin_lims = board.getServoVinLim(id.value());
        if (vin_lims)
          vins_lims.push_back(vin_lims.value());

        vin = board.getServoVin(id.value());
        if (vin)
          vins.push_back(vin.value());

        temp = board.getServoTemp(id.value());
        if (temp)
          temps.push_back(temp.value());

        temp_lims = board.getServoTempLim(id.value());
        if (temp_lims)
          temps_lims.push_back(temp_lims.value());

        torque = board.getServoTorque(id.value());
        if (torque)
          torques.push_back(torque.value());
      }
    }
    std::cout << "IDs detected: " << ids.size() << std::endl;
  }

  return 0;
}
