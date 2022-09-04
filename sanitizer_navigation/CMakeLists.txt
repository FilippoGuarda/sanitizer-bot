cmake_minimum_required(VERSION 3.0.2)
project(sanitizer_navigation)


find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
  uv_visualization
  localization
)

catkin_package(
  CATKIN_DEPENDS 
  rospy
  message_runtime
)

include_directories(
# include
  launch
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/disinfection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


