cmake_minimum_required(VERSION 3.0.2)
project(uv_visualization)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  std_msgs
  image_transport
  cv_bridge
  message_generation
  
  grid_map_demos
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
  geometry_msgs
  sensor_msgs
  octomap_msgs
  filters
)

add_message_files(
  FILES
  lowestIrradiation.msg
  subMapCoords.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# If another package depends on this, it will need also message_runtime
catkin_package(
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(uv_visualization src/uv_grid_map.cpp)
target_link_libraries(uv_visualization ${catkin_LIBRARIES})

# Mark executables and/or libraries for installation
install(
  TARGETS 
    uv_visualization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY config launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


