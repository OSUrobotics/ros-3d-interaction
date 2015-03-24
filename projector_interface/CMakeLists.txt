# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(projector_interface)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS
    rospy
    projector_calibration
    tf
    tf_conversions
    sensor_msgs
    geometry_msgs
    image_geometry
    pr2_python
    dynamic_reconfigure
    message_generation
)
catkin_python_setup()

add_service_files(
    DIRECTORY srv
    FILES
    CircleInhibit.srv
    ClearHilights.srv
    ClearPolygons.srv
    DrawPolygon.srv
    GetCursorStats.srv
    HilightObject.srv
    SetSelectionMethod.srv
)

generate_messages(
   DEPENDENCIES geometry_msgs sensor_msgs
)

generate_dynamic_reconfigure_options(cfg/Interface.cfg)

catkin_package(
    CATKIN_DEPENDS rospy projector_calibration tf tf_conversions sensor_msgs geometry_msgs image_geometry pr2_python dynamic_reconfigure message_runtime
)

install(
    PROGRAMS
        nodes/adjust_pose.py
        nodes/click.py
        nodes/find_objects.py
        nodes/object_circler.py
        nodes/object_manipulation.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)