# HW-PI-HAT-LIBRARY

This repo contains a c++ implementation of an interface between the raspberry pi and the [hw-pi-hat-extension-board](https://www.hiwonder.com/collections/expansion-board/products/expansion-board-for-raspberry-pi-5?variant=40939498766423).


### Dependencies

- `gpiod` version `2.3`

`sudo apt install libgpiod-dev`


## Building & Install

```bash
git clone https://github.com/Snowespa/hw_pi_hat_interface.git
cd hw_pi_hat_interface
mkdir build && cd build
cmake ..
make
make install
```


## TODO

- [x] keep only the last message in the queues.
- [x] code and test the servo reading and writing.
- [x] wrap the board in a ros node.
- [x] implement IMU reading.
    - [x] Find out the message type for the IMU in the FD.
    - [x] parse the IMU message.
    - [x] implement the code in c++.
- [x] make setServoPos read more than 5 servo message
    `sizeof(buf)` returns 24. Use `buf.size()` when writing to the file descriptor.
- [x] Compile library.