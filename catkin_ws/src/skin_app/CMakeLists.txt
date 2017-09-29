cmake_minimum_required(VERSION 2.8.3)
project(skin_app)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
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

add_executable(skin
  src/skin.cpp
  include/skin_app/skin.h
)


target_link_libraries(skin
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
