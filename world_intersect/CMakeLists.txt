# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(world_intersect)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_ros
    pcl_conversions
    geometry_msgs
    sensor_msgs
    std_msgs
    message_filters
    tf pr2_python
)

find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

catkin_package(
    CATKIN_DEPENDS
        roscpp
        pcl_ros
        pcl_conversions
        geometry_msgs
        sensor_msgs
        std_msgs
        message_filters
        tf
        pr2_python
)

add_executable(world_intersect src/world_intersect.cpp)
target_link_libraries(world_intersect ${catkin_LIBRARIES})

install(DIRECTORY nodes/  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS world_intersect DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS nodes/intersect_plane.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)