cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_real)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    roscpp
    tf
    geometry_msgs
    unitree_legged_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
)

include_directories(
    # /home/unitree/unitree_legged_sdk/include
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    # /home/bian/unitree_legged_sdk/include/unitree_legged_sdk
    $ENV{UNITREE_LEGGED_SDK_PATH}/include
)

link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_$ENV{UNITREE_PLATFORM}.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

set(CMAKE_CXX_FLAGS "-O3")

add_executable(position_lcm src/exe/position_mode.cpp)
target_link_libraries(position_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(velocity_lcm src/exe/velocity_mode.cpp)
target_link_libraries(velocity_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(torque_lcm src/exe/torque_mode.cpp)
target_link_libraries(torque_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(walk_lcm src/exe/walk_mode.cpp)
target_link_libraries(walk_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(lcm_server $ENV{UNITREE_LEGGED_SDK_PATH}/examples/lcm_server.cpp)
target_link_libraries(lcm_server ${EXTRA_LIBS} ${catkin_LIBRARIES})