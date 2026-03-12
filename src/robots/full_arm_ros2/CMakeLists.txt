cmake_minimum_required(VERSION 3.16)
project(full_arm_ros2)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Windows compatibility
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Find build system
find_package(ament_cmake REQUIRED)

###################################
# INSTALL DESCRIPTION FILES
###################################

# URDF and Xacro files
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Meshes
install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY urdf meshes launch config
  DESTINATION share/${PROJECT_NAME}
)


###################################
# EXPORT PACKAGE
###################################

ament_package()


