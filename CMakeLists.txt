cmake_minimum_required(VERSION 3.16)

# Project Name
project(hw_pi_hat_interface VERSION 1.0.0 DESCRIPTION "hw_pi_hat_interface_library")
include(GNUInstallDirs)
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

# Executable to monitor the board and test the different operations.
add_executable(monitor_board ${SOURCES})
if(DEBUG_MODE)
  target_compile_definitions(monitor_board PRIVATE HW_DEBUG_LOG=1)
endif()

target_include_directories(monitor_board
    PRIVATE ${CMAKE_SOURCE_DIR}/include
    PRIVATE ${CMAKE_SOURCE_DIR}/third_party
)

target_link_libraries(monitor_board pthread gpiodcxx)

# Executable that plays a sound at startup.
add_executable(startup src/board.cpp src/startup.cpp)
if(DEBUG_MODE)
  target_compile_definitions(startup PRIVATE HW_DEBUG_LOG=1)
endif()
target_include_directories(startup
  PRIVATE ${CMAKE_SOURCE_DIR}/include
  PRIVATE ${CMAKE_SOURCE_DIR}/third_party
)

target_link_libraries(startup pthread gpiodcxx)

install(TARGETS startup monitor_board
    DESTINATION ${CMAKE_INSTALL_BINDIR})

# Library
add_library(hw_pi_hat_interface SHARED src/board.cpp)

if(DEBUG_MODE)
  target_compile_definitions(hw_pi_hat_interface PRIVATE HW_DEBUG_LOG=1)
endif()

set_target_properties(hw_pi_hat_interface PROPERTIES
              VERSION ${PROJECT_VERSION}
              SOVERSION ${PROJECT_VERSION_MAJOR}
              PUBLIC_HEADER include/board.hpp)
configure_file(hw_pi_hat_interface.pc.in hw_pi_hat_interface.pc @ONLY)
target_include_directories(hw_pi_hat_interface PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(hw_pi_hat_interface pthread gpiodcxx)
install(TARGETS hw_pi_hat_interface
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hw_pi_hat_interface)
install(FILES ${CMAKE_BINARY_DIR}/hw_pi_hat_interface.pc
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)