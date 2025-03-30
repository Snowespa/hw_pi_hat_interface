#include "../include/board.hpp"

int main(int argc, char *argv[]) {

  Board board;
  sleep(1);
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

  return 0;
}
