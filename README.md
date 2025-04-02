# HW-PI-HAT-LIBRARY

This repo contains a c++ implementation of an interface between the raspberry pi and the [hw-pi-hat-extension-board](https://www.hiwonder.com/collections/expansion-board/products/expansion-board-for-raspberry-pi-5?variant=40939498766423).

## Building

### Dependencies

- `gpiod` version `2.3`

## TODO

- [x] keep only the last message in the queues.
- [x] code and test the servo reading and writing.
- [x] wrap the board in a ros node.
- [ ] implement IMU reading.
    - [ ] Find out the message type for the IMU in the FD.
    - [ ] parse the IMU message.
    - [ ] implement the code in c++.

## BUGS

- [x] Understand why after some time the c++ rcv pkt fails.
- [x] ~Why were the libraries corrupted.~ Poor quality sd card
