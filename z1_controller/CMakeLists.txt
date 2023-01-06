cmake_minimum_required(VERSION 3.0.2)
project(z1_controller)

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    controller_manager
    std_msgs
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    geometry_msgs
    unitree_legged_msgs
    tf
)

catkin_package(
    CATKIN_DEPENDS
    unitree_legged_msgs 
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME}
    src/unitreeArm.cpp 
)

add_dependencies(${PROJECT_NAME} unitree_legged_msgs_gencpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${EXTRA_LIBS}
)

add_executable(unitree_move_z1 src/basic.cpp)
target_link_libraries(unitree_move_z1 ${PROJECT_NAME} ${catkin_LIBRARIES})