cmake_minimum_required(VERSION 2.8.3)
project(mbot_scan_map)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  roscpp
  sensor_msgs
  tf
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_map_node src/scan_map_node.cpp)
target_link_libraries(scan_map_node ${catkin_LIBRARIES})
add_dependencies(scan_map_node ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS scan_map_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

