cmake_minimum_required(VERSION 3.16)

# Project Name
project(
  hw_pi_hat_interface 
  VERSION 1.0.0
  DESCRIPTION "A C++ project to communicate with the hiwonder pi hat extension board."
  LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

option(DEBUG_MODE "Enable Debug Mode" OFF)


#---------------------------------------------------------#
# Debug Configuration                                     #
#---------------------------------------------------------#

if(DEBUG_MODE)
  message(STATUS "Compile in Debug Mode")
  add_definitions(-DDEBUG)
  set(CMAKE_BUILD_TYPE Debug)
else()
  set(CMAKE_BUILD_TYPE Release)
endif()

#---------------------------------------------------------#
# Dependencies                                            #
#---------------------------------------------------------#

find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

# The library requires libgpiod >= 2.3
pkg_check_modules(
  GPIOD
  REQUIRED
  IMPORTED_TARGET
  libgpiodcxx>=2.3
)


#---------------------------------------------------------#
# Library                                                 #
#---------------------------------------------------------#
add_library(hw_pi_hat_interface)
add_library(hw_pi_hat_interface::hw_pi_hat_interface ALIAS hw_pi_hat_interface)
target_sources(hw_pi_hat_interface PRIVATE src/board.cpp)

if(DEBUG_MODE)
  target_compile_definitions(hw_pi_hat_interface PRIVATE HW_DEBUG_LOG=1)
endif()

set_target_properties(
  hw_pi_hat_interface PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  PUBLIC_HEADER include/board.hpp
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(
  hw_pi_hat_interface
  PRIVATE ${CMAKE_SOURCE_DIR}/src
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# board.hpp currently includes <gpiod.hpp>, so libgpiod is part of the
# public compile-time interface.
target_link_libraries(
  hw_pi_hat_interface
  PUBLIC 
    PkgConfig::GPIOD
  PRIVATE
    Threads::Threads
)


#---------------------------------------------------------#
# Executables                                             #
#---------------------------------------------------------#

# Executable to monitor the board and test the different operations.
add_executable(monitor_board src/main.cpp)

target_include_directories(monitor_board
    PRIVATE ${CMAKE_SOURCE_DIR}/include
    PRIVATE ${CMAKE_SOURCE_DIR}/third_party
)
target_link_libraries(monitor_board
  PRIVATE hw_pi_hat_interface
)

# Executable that plays a sound at startup.
add_executable(startup src/startup.cpp)

target_include_directories(startup
  PRIVATE ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(startup
  PRIVATE hw_pi_hat_interface
)


#---------------------------------------------------------#
# Installation                                            #
#---------------------------------------------------------#

# Install the shared library and public header.
install(
  TARGETS hw_pi_hat_interface
  EXPORT hw_pi_hat_interfaceTargets
  LIBRARY
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    COMPONENT Runtime
  PUBLIC_HEADER
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT Development
)

# Install executable
install(
  TARGETS
    startup
    monitor_board
  RUNTIME
    DESTINATION ${CMAKE_INSTALL_BINDIR}
)


#---------------------------------------------------------#
# CMake package configuration                             #
#---------------------------------------------------------#

# Generate the exported target file:
#   hw_pi_hat_interfaceTargets.cmake
install(
  EXPORT hw_pi_hat_interfaceTargets
  FILE hw_pi_hat_interfaceTargets.cmake
  NAMESPACE hw_pi_hat_interface::
  DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/hw_pi_hat_interface
)

# Generate:
#   hw_pi_hat_interfaceConfig.cmake
#   hw_pi_hat_interfaceConfigVersion.cmake
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/hw_pi_hat_interfaceConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/hw_pi_hat_interfaceConfig.cmake
  INSTALL_DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/hw_pi_hat_interface
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/hw_pi_hat_interfaceConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/hw_pi_hat_interfaceConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/hw_pi_hat_interfaceConfigVersion.cmake
  DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/hw_pi_hat_interface
)


#---------------------------------------------------------#
# pkg-config                                              #
#---------------------------------------------------------#

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/hw_pi_hat_interface.pc.in
  ${CMAKE_CURRENT_BINARY_DIR}/hw_pi_hat_interface.pc
  @ONLY
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/hw_pi_hat_interface.pc
  DESTINATION 
    ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig
)


#---------------------------------------------------------#
# Install-time linker cache update                        #
#---------------------------------------------------------#

# On systems such as Raspberry Pi OS / Ubuntu, update the dynamic linker
# cache after installing the shared library.
#
# This is useful when installing into /usr/local/lib.
# install(
#   CODE
#   "execute_process(COMMAND ldconfig)"
# )