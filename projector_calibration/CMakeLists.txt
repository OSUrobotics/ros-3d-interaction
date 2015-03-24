# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(projector_calibration)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS rospy std_msgs cv_bridge message_generation)
catkin_python_setup()

add_message_files(
  DIRECTORY msg
  FILES
  Homography.msg
)

generate_messages(
    DEPENDENCIES std_msgs
)

catkin_package(
    CATKIN_DEPENDS rospy message_runtime std_msgs sensor_msgs cv_bridge
)

install(PROGRAMS nodes/calibrate.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})