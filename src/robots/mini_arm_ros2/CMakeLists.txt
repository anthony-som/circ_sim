cmake_minimum_required(VERSION 3.8)
project(mini_arm_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)

add_library(mini_arm_ros2 SHARED
  src/arm_pca9685_system.cpp
)

ament_target_dependencies(mini_arm_ros2
  rclcpp
  std_msgs
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

install(TARGETS mini_arm_ros2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY urdf config launch meshes
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fk_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ikpy_ik_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cartesian_ik_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

