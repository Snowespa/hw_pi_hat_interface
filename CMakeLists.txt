cmake_minimum_required(VERSION 3.16)

# Project Name
project(hwpihat)


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

find_package(ament_cmake REQUIRED)

find_library(GPIOD_LIBRARY gpiod)
if(NOT GPIOD_LIBRARY)
    message(FATAL_ERROR "libgpiod not found")
endif()

set(SOURCES
    src/hw-pi-hat-lib.cpp
)

add_library(hwpihat STATIC ${SOURCES})
# Add include directories
target_include_directories(hwpihat
    PRIVATE ${CMAKE_SOURCE_DIR}/include
    PRIVATE ${CMAKE_SOURCE_DIR}/third_party)


target_link_libraries(hwpihat pthread gpiodcxx)

# Export include directories
target_include_directories(hwpihat PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install the library and headers
install(
    TARGETS hwpihat
    EXPORT hwpihatTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include
)

# Generate CMake config files for this library
ament_export_include_directories(include)
ament_export_libraries(hwpihat)

# Register the package with ament
ament_package()

