#include <termios.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>
#include <optional>
#include <ostream>
#include <tuple>
#include <vector>

#include "../include/CLI11.hpp"
#include "../include/board.hpp"

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
  button = true;
  CLI11_PARSE(app, argc, argv);

  Board board("/dev/ttyAMA0", B1000000, 500);
  board.setRecieve(true);
  sleep(2);

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
  } else if (button) {
    std::cout << "Test button!" << std::endl;
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

  return 0;
}
