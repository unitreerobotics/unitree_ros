cmake_minimum_required(VERSION 3.0.1)
project(unitree_legged_control)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    realtime_tools
    unitree_legged_msgs
        urdf
)

catkin_package(
    CATKIN_DEPENDS
    unitree_legged_msgs
    controller_interface
    hardware_interface
    pluginlib
    roscpp
        urdf
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

link_directories(${catkin_LIB_DIRS} lib)

add_library( unitree_legged_control 
    src/joint_controller.cpp
)
add_dependencies(${PROJECT_NAME} unitree_legged_msgs_gencpp)
# target_link_libraries(unitree_legged_control ${catkin_LIBRARIES} unitree_joint_control_tool_arm64)
target_link_libraries(unitree_legged_control ${catkin_LIBRARIES} unitree_joint_control_tool)
