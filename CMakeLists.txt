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

find_library(GPIOD_LIBRARY gpiod)
if(NOT GPIOD_LIBRARY)
  message(FATAL_ERROR "libgpiod not found")
endif()

set(SOURCES
    src/board.cpp
    src/main.cpp
)

add_executable(monitor_board ${SOURCES})
# Add include directories

if(DEBUG_MODE)
  target_compile_definitions(monitor_board PRIVATE HW_DEBUG_LOG=1)
endif()

target_include_directories(monitor_board
    PRIVATE ${CMAKE_SOURCE_DIR}/include
    PRIVATE ${CMAKE_SOURCE_DIR}/third_party
)

target_link_libraries(monitor_board pthread gpiodcxx)


add_executable(startup src/board.cpp src/startup.cpp)
if(DEBUG_MODE)
  target_compile_definitions(startup PRIVATE HW_DEBUG_LOG=1)
endif()
target_include_directories(startup
  PRIVATE ${CMAKE_SOURCE_DIR}/include
  PRIVATE ${CMAKE_SOURCE_DIR}/third_party
)

target_link_libraries(startup pthread gpiodcxx)

# add_library(hw_pi_hat_interface src/board.cpp)