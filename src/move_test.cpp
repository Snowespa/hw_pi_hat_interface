#include "../include/board.hpp"
#include <cstdint>
#include <optional>
#include <vector>


std::vector<int16_t> read_positions(Board &board, std::vector<uint8_t> &ids) {
    std::vector<int16_t> pos;
    for (std::vector<uint8_t>::iterator it = ids.begin(); it != ids.end(); it++) {
        std::optional<int16_t> read = board.getServoPos(*it);
        if (read)
            pos.push_back(read.value());
    }
    return pos;
}

void print_pos(const std::vector<uint8_t> &ids, const std::vector<int16_t> pos) {
    for (size_t i = 0; i < ids.size(); i++) {
        std::cout << "ID: " << ids[i] << ", " << pos[i] << std::endl;
    }
}


int main(int argc, char *argv[]) {
    Board board;
    board.setRecieve(true);

    // Test move all servos to 500 position
    std::vector<uint8_t> ids{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    std::vector<uint16_t> angles{500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500};
    board.setServoPos(ids, angles, 0);
    
    // Move 4 servos at a time.
    // std::vector<uint8_t> ids_hip{0, 3, 6, 9};
    // std::vector<uint16_t> angles_hip{500, 500, 500, 500};
    // board.setServoPos(ids_hip, angles_hip, 0);

    // std::vector<uint8_t> ids_knee{1, 4, 7, 10};
    // std::vector<uint16_t> angles_knee{500, 500, 500, 500};
    // board.setServoPos(ids_knee, angles_knee, 0);

    // std::vector<uint8_t> ids_ankle{2, 5, 8, 11};
    // std::vector<uint16_t> angles_ankle{500, 500, 500, 500};
    // board.setServoPos(ids_ankle, angles_ankle, 0);

    sleep(2);
    std::cout << "Reset servo torque" << std::endl;
    for (std::vector<uint8_t>::iterator it = ids.begin(); it != ids.end(); it++) {
        board.setServoTorque(*it, true);
    }
}