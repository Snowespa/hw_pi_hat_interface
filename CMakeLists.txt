cmake_minimum_required(VERSION 3.16)

# Project Name
project(quaro_ros_interface)
option(DEBUG_MODE "Enable Debug Mode" OFF)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(DEBUG_MODE)
  message(STATUS "Compile in Debug Mode")
  add_definitions(-DDEBUG)
  set(CMAKE_BUILD_TYPE Debug)
else()
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_library(GPIOD_LIBRARY gpiod)
if(NOT GPIOD_LIBRARY)
    message(FATAL_ERROR "libgpiod not found")
endif()

# Include directories
include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/third_party
)

# Build library
add_library(hwpihat STATIC src/hw-pi-hat-lib.cpp)
target_include_directories(hwpihat PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>)
target_link_libraries(hwpihat PUBLIC pthread gpiodcxx)

# Build node
add_executable(my_node src/ros2-hw-pi-hat.cpp)
target_include_directories(my_node PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include> ${std_msgs_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS})
target_link_libraries(my_node PRIVATE ${std_msgs_LIBRARIES} ${rclcpp_LIBRARIES} hwpihat)

# Install the library and headers
install(
    TARGETS my_node hwpihat 
    EXPORT quaro_ros_interfaceTargets 
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME} 
)

install(
    DIRECTORY include/
    DESTINATION include
)

# Generate CMake config files for this library
ament_export_include_directories(include)
ament_export_libraries(hwpihat)
ament_export_dependencies(rclcpp gpiod)

# Register the package with ament
ament_package()

