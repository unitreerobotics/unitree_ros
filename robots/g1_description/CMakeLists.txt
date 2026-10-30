cmake_minimum_required(VERSION 2.8.3)
project(g1_description)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS
)

include_directories(
    # include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)
