#include <gpiod.hpp>
#include <sys/types.h>
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
#include "../third_party/tabulate/tabulate.hpp"

void displayServoData(
    const std::vector<uint8_t> &ids, const std::vector<int8_t> &offsets,
    const std::vector<int16_t> &positions,
    const std::vector<std::pair<uint16_t, uint16_t>> &angles_lims,
    const std::vector<std::pair<uint16_t, uint16_t>> &vins_lims,
    const std::vector<uint16_t> &vins, const std::vector<uint8_t> &temps,
    const std::vector<uint8_t> &temps_lims, const std::vector<bool> &torques) {

  tabulate::Table servo_table;
  servo_table.add_row({"ID", "Position", "Position Limits", "Position Offset",
                       "Vin", "Vin Limits", "Temp", "Temp Limit", "Torque"});

  for (size_t i = 0; i < ids.size(); i++) {
    std::string angle_limit =
        angles_lims.size() > i ? std::to_string(angles_lims[i].first) + " - " +
                                     std::to_string(angles_lims[i].second)
                               : "N/A";

    std::string vin_limit = vins_lims.size() > i
                                ? std::to_string(vins_lims[i].first) + " - " +
                                      std::to_string(vins_lims[i].second)
                                : "N/A";

    servo_table.add_row(
        {std::to_string(ids[i]),
         positions.size() > i ? std::to_string(positions[i]) : "N/A",
         angle_limit, offsets.size() > i ? std::to_string(offsets[i]) : "N/A",
         vins.size() > i ? std::to_string(vins[i]) : "N/A", vin_limit,
         temps.size() > i ? std::to_string(temps[i]) : "N/A",
         temps_lims.size() > i ? std::to_string(temps_lims[i]) : "N/A",
         torques.size() > i ? (torques[i] ? "ON" : "OFF") : "N/A"});
  }

  servo_table[0]
      .format()
      .font_style({tabulate::FontStyle::bold, tabulate::FontStyle::underline})
      .font_color(tabulate::Color::green)
      .border_color(tabulate::Color::blue)
      .corner_color(tabulate::Color::blue)
      .corner_bottom_left("")
      .corner_bottom_right("")
      .corner_top_left("")
      .corner_top_right("")
      .border_top("")
      .border_bottom("")
      .border_right("|")
      .border_left("|");

  servo_table.format()
      .border_color(tabulate::Color::blue)
      .corner_color(tabulate::Color::blue)
      .corner_bottom_left("")
      .corner_bottom_right("")
      .corner_top_left("")
      .corner_top_right("")
      .border_top("")
      .border_bottom("")
      .border_right("|")
      .border_left("|");
  std::cout << servo_table << std::endl;
}

void print_key_event(std::pair<uint8_t, uint8_t> key_event) {
  std::cout << unsigned(key_event.first)
            << " event: " << unsigned(key_event.second) << std::endl;
}

void info(Board &board) {
  std::vector<uint8_t> expected_ids{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

  std::optional<uint8_t> id;
  std::vector<uint8_t> ids;

  std::optional<uint8_t> offset;
  std::vector<int8_t> offsets;

  std::optional<int16_t> pos;
  std::vector<int16_t> positions;

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
  displayServoData(ids, offsets, positions, angles_lims, vins_lims, vins, temps,
                   temps_lims, torques);
  for (std::vector<uint8_t>::iterator it = ids.begin(); it != ids.end(); it++) {
    board.setServoTorque(*it, true);
  }
  std::cout << "IMU: " << std::endl;
  std::optional<float*> imu = board.getIMU();

  if (!imu) {
    std::cout << "[ERROR] Could not read IMU entry" << std::endl;
  } else {
    for (size_t i = 0; i < 6; i++) {
      std::cout << imu.value()[i];
    }
    std::cout << std::endl;
  }
}

void actionB() {
  std::cout << "Action B executed: Doing something else!" << std::endl;
}

void actionC() {
  std::cout << "Action C executed: Yet another action!" << std::endl;
}

void offset(Board &board) {
  char id_number;
  char offset;
  while (true) {
    std::cout << "\n===== Servo Offset Menu ====" << std::endl;
    std::cout << "id - choose servo id or type q to quit" << std::endl;

    std::cin >> id_number;

    switch (id_number)
    case 'q':
    case 'Q':
      return;

    std::optional<uint8_t> id;
    id = board.getServoId(
        static_cast<uint8_t>(std::stoi(std::string(1, id_number))));
    if (!id) {
      std::cout << "[ERROR] servo not found" << std::endl;
      continue;
    }
    std::cout << "value - choose offset value, has to be between [0-1000]"
              << std::endl;
    std::cin >> offset;

    board.setServoOffset(id.value(), static_cast<uint8_t>(offset));
  }
}

void id(Board &board) {
  char old_id_str[8];
  char new_id_str[8];
  std::cout << "\n==== Set Servo Id ====" << std::endl;
  std::cout << "Board diagnostic: " << std::endl;

  info(board);
  std::cout << "Old Id: ";
  std::cin >> old_id_str;

  std::cout << "New Id: ";
  std::cin >> new_id_str;

  uint8_t old_id = static_cast<uint8_t>(std::stoi(old_id_str));
  uint8_t new_id = static_cast<uint8_t>(std::stoi(new_id_str));

  board.setServoId(old_id, new_id);
}

void showMenu() {
  std::cout << "\n===== User Interface Menu =====" << std::endl;
  std::cout << "r - Read infromation from board" << std::endl;
  std::cout << "t - set servo max temperature" << std::endl;
  std::cout << "v - set servo max voltage" << std::endl;
  std::cout << "o - set servo offset angle" << std::endl;
  std::cout << "i - set servo id" << std::endl;
  std::cout << "q - Quit" << std::endl;
  std::cout << "================================" << std::endl;
  std::cout << "Choose an option: ";
}

int main(int argc, char **argv) {
  Board board;
  board.setRecieve(true);
  sleep(1);

  char choice;
  while (true) {
    showMenu();
    std::cin >> choice;

    switch (choice) {
    case 'r':
    case 'R':
      info(board);
      break;
    case 't':
    case 'T':
      actionB();
      break;
    case 'v':
    case 'V':
      actionC();
      break;
    case 'o':
    case 'O':
      offset(board);
      break;
    case 'i':
    case 'I':
      id(board);
      break;
    case 'q':
    case 'Q':
      std::cout << "Exiting..." << std::endl;
      return 0;
    default:
      std::cout << "Invalid option! Please try again." << std::endl;
    }
  }
  return 0;
}
