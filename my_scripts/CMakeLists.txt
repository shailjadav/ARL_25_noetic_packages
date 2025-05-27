# filepath: /root/catkin_ws/src/my_scripts/CMakeLists.txt
cmake_minimum_required(VERSION 3.0.2)
project(my_scripts)

## Find catkin macros and libraries
## Components listed here are find_package()-ed automatically
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Service files
# The DIRECTORY path is relative to this CMakeLists.txt (i.e., package root)
add_service_files(
  FILES
  TowerMove.srv
  DIRECTORY assignment_3/Docker_volume/srv
)

## Generate messages (services are a type of message)
# Ensure all dependencies of your messages/services are listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare catkin package
# CATKIN_DEPENDS are runtime dependencies for this package
# and also build dependencies for packages that depend on this package.
catkin_package(
#  INCLUDE_DIRS include  # Uncomment if you have C++ headers in an 'include' directory
#  LIBRARIES ${PROJECT_NAME} # Uncomment if you build C++ libraries
  CATKIN_DEPENDS rospy std_msgs message_runtime
#  DEPENDS system_lib # Uncomment if you have other system dependencies
)

## Installing Python scripts (optional, but good practice for ROS nodes)
#
# If you place your executable ROS Python scripts in a 'scripts' subfolder
# (e.g., /root/catkin_ws/src/my_scripts/scripts/hanoi_service_provider.py),
# you can install them so they are found by rosrun.
# Make sure they are executable (chmod +x).
#
# Example:
# install(PROGRAMS
#   scripts/hanoi_service_provider.py
#   # scripts/another_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
#
# If your hanoi_service_provider.py remains in assignment_3/Docker_volume/,
# and you want to install it:
# install(PROGRAMS
#   assignment_3/Docker_volume/hanoi_service_provider.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Installing Python library modules (if you have any)
#
# If your package provides Python modules others can import, use catkin_install_python.
# Example:
# catkin_install_python(PROGRAMS
#   src/${PROJECT_NAME}/your_python_module.py
#   DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
# )