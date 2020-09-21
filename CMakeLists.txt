cmake_minimum_required(VERSION 3.0.2)
project(moveit_pybind)

find_package(catkin REQUIRED COMPONENTS
    pybind11_catkin
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_warehouse
    moveit_ros_manipulation
    moveit_ros_move_group
    geometry_msgs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    roscpp
    actionlib
    rospy
    rosconsole
)

catkin_python_setup()

include_directories(${PROJECT_SOURCE_DIR}
                    ${PYTHON_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})
                    
link_directories(${catkin_LIBRARY_DIRS})

set(MOVEIT_SOURCE_DIR "src/moveit_pybind")

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${catkin_INCLUDE_DIRS})
include_directories(${MOVEIT_SOURCE_DIR})

catkin_package(
    LIBRARIES
    CATKIN_DEPENDS pybind11_catkin
                   moveit_msgs
                   moveit_ros_planning
                   moveit_ros_planning_interface
                   moveit_ros_warehouse
                   moveit_ros_manipulation
                   moveit_ros_move_group
                   geometry_msgs
                   tf2
                   tf2_eigen
                   tf2_geometry_msgs
                   tf2_ros
                   roscpp
                   actionlib
                   rospy
                   rosconsole
)


pybind_add_module(moveit_pybind MODULE "${MOVEIT_SOURCE_DIR}/moveit_pybind.cpp")

target_link_libraries(moveit_pybind PRIVATE ${catkin_LIBRARIES})

install(TARGETS moveit_pybind
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)