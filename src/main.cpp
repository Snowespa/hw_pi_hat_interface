#include <termios.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>
#include <tuple>
#include <vector>

#include "../include/CLI11.hpp"
#include "../include/board.hpp"

int main(int argc, char **argv) {
  CLI::App app{"Test Board Interface"};
  bool startup = false;
  app.add_flag("-s,--startup", startup, "Emits the startup sound");
  CLI11_PARSE(app, argc, argv);

  Board board("/dev/ttyAMA0", B1000000, 500);

  if (startup) {
    uint16_t freq[3] = {450, 550, 820};
    float time[3] = {0.2, 0.15, 0.25};
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

  return 0;
}
