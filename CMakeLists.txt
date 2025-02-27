cmake_minimum_required(VERSION 3.16)

# Project Name
project(hw_pi_hat_interface)

option(DEBUG_MODE "Enable Debug Mode" OFF)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(DEBUG_MODE)
  message(STATUS "Compile in Debug Mode")
  add_definitions(-DDEBUG)
  set(CMAKE_BUILD_TYPE Debug)
else()
  set(CMAKE_BUILD_TYPE Release)
endif()

set(SOURCES
    src/board.cpp
    src/main.cpp
)

add_executable(board_test ${SOURCES})
# Add include directories
target_include_directories(board_test
    PRIVATE ${CMAKE_SOURCE_DIR}/include
    PRIVATE ${CMAKE_SOURCE_DIR}/third_party)


target_link_libraries(board_test pthread gpiodcxx)

