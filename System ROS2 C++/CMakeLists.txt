cmake_minimum_required(VERSION 3.8)
project(akku_monitoring_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2-Abhängigkeiten
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Für die Robotino-Spannungs-Node, falls sie REST/libcurl nutzt
find_package(CURL REQUIRED)

# Arduino State Reader
add_executable(
  arduino_battery_state_reader
  src/arduino_battery_state_reader.cpp
)

ament_target_dependencies(
  arduino_battery_state_reader
  rclcpp
  std_msgs
)

# Robotino Voltage Reader
add_executable(
  robotino_voltage_state_reader
  src/robotino_voltage_state_reader.cpp
)

ament_target_dependencies(
  robotino_voltage_state_reader
  rclcpp
  std_msgs
)

target_link_libraries(
  robotino_voltage_state_reader
  CURL::libcurl
)

# Battery State Estimator
add_executable(
  battery_state_estimator
  src/battery_state_estimator.cpp
)

ament_target_dependencies(
  battery_state_estimator
  rclcpp
  std_msgs
)

# Installiere die Nodes, damit ros2 run sie findet
install(
  TARGETS
  arduino_battery_state_reader
  robotino_voltage_state_reader
  battery_state_estimator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()