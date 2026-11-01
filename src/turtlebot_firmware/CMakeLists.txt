cmake_minimum_required(VERSION 3.8)
project(turtlebot_firmware)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_library(SERIAL_LIB NAMES serial REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(${PROJECT_NAME} SHARED 
  src/turtlebot_interface.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE 
  include
)

target_link_libraries(${PROJECT_NAME}
  ${SERIAL_LIB}
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface turtlebot_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()