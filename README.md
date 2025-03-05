# ROS2 HW-PI-HAT-LIBRARY

This repo contains a ros package with a c++ implementation of an interface between the raspberry pi and the [hw-pi-hat-extension-board](https://www.hiwonder.com/collections/expansion-board/products/expansion-board-for-raspberry-pi-5?variant=40939498766423).

## Building

### Dependencies

- `gpiod` version `2.3`


Build the package with colcon.


## Topics

- `/battery` publishes the currenty voltage input to the system.
- `/temperature` publishes a list of all the connected bus servo's temperature at `~20Hz`.
- `/vin` publishes the input voltage of each servo at `~20Hz`.
- `/position` publishes the position of each servo at `~20Hz`.

## Subscribers

- `/stop` if any message is recieved on this topic all the servos stop.
- `/set_servos_pos` Subscribes to message type `servo_msgs` a costum message type.

## Services

- `/set_servos_temperature_limit`
- `/set_servos_voltage_limit`
- `/set_servos_position_limit`
- `/set_servo_offsets`


## TODO

- [x] keep only the last message in the queues.
- [x] code and test the servo reading and writing.
- [x] wrap the board in a ros node.
- [ ] implement services
- [ ] convert postion to angles [0, 1000] -> [0, 270]

## BUGS

- [x] Understand why after some time the c++ rcv pkt fails.
- [ ] Why were the libraries corrupted.
- [ ] Why do i need to launch the python code once for the requests to work.
    - look at the file descriptor configuration before and after launching the python code.
    - look at the file descriptor configuration before and after launching the cpp code.


