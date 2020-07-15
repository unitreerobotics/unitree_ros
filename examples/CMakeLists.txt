cmake_minimum_required(VERSION 2.8.3)
project(unitree_examples)

find_package(catkin REQUIRED COMPONENTS
    roscpp 
    std_msgs 
)

find_package(gazebo REQUIRED)

catkin_package(
    CATKIN_DEPENDS
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

add_executable(laikago_move src/move_publisher.cpp)
target_link_libraries(laikago_move ${catkin_LIBRARIES})
