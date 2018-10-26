file(GLOB common_folder_source src/*.cpp)

cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

project(people_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  tf
  tf_conversions
  message_generation
  interactive_markers
  nav_msgs
  scout_msgs
  people_msgs
)

find_package(OpenCV REQUIRED)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../binAndDataFiles)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
)

SET(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -Wall") #the -g option is implicit...
SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O2 -finline-functions -fpredictive-commoning -fgcse-after-reload -ftree-slp-vectorize -ftree-loop-distribute-patterns -fipa-cp-clone -funswitch-loops -fvect-cost-model -ftree-partial-pre -g")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE}") #Default build mode is release mode

include_directories(ros/include include)
include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

source_group("Tracker Source Files" FILES ${tracker_folder_source})
source_group("Tracker Header Files" FILES ${tracker_folder_header})

add_executable(after src/tracker/after.cpp)
target_link_libraries(after ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(follower src/follower/follower.cpp)
target_link_libraries(follower ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
