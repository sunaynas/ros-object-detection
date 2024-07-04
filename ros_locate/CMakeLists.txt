cmake_minimum_required(VERSION 3.0.2)
project(ros_locate)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  ros_locate_msgs
)
#set(OpenCV_DIR "/usr/share/opencv4"
   #CACHE PATH "The path where OpenCVConfig.cmake is placed")
## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

#set(Torch_DIR "/home/robofob/Lib/libtorch/share/cmake/Torch/")    
#find_package(Torch REQUIRED)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros-object-detection
#  CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS rospy sensor_msgs cv_bridge ros_locate_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
#include_directories(${Torch_INSTALL_INCLUDE}
#${Torch_DIR})


# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  src/obj_location.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


link_libraries(${OpenCV_LIBRARIES})
link_libraries(${catkin_LIBRARIES})
#link_libraries(${Torch_INSTALL_LIB})

