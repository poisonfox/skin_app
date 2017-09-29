cmake_minimum_required(VERSION 2.8.3)
project(final_pkg)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  tld_msgs
  sensor_msgs
  geometry_msgs
  luh_youbot_controller_api
  tf
  cv_bridge
  image_transport
)




# core highgui imgproc ocl
find_package( OpenCV REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(final_statemachine
  src/final_main_sm.cpp
  src/final_statemachine.cpp
  include/final_statemachine.h
)
add_executable(final_angle
  src/final_angle.cpp
  include/final_pkg/final_angle.h
)

target_link_libraries(final_statemachine
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


target_link_libraries(final_angle
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
