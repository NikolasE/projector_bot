cmake_minimum_required(VERSION 2.8.3)
project(projector_bot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  laser_geometry
  roscpp
  tf2_ros
  tf2
)

find_package(OpenCV REQUIRED core imgproc highgui)


catkin_package()

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(image_screen
  src/image_screen/image_screen.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(image_screen
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  image_screen
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)