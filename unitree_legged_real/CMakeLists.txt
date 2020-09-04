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
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    $ENV{UNITREE_LEGGED_SDK_PATH}/include
    $ENV{ALIENGO_SDK_PATH}/include
)

link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
link_directories($ENV{ALIENGO_SDK_PATH}/lib)
string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_$ENV{UNITREE_PLATFORM}.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME} libaliengo_comm.so lcm)

set(CMAKE_CXX_FLAGS "-O3")

add_executable(position_lcm src/exe/position_mode.cpp)
target_link_libraries(position_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(position_lcm ${${PEOJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(velocity_lcm src/exe/velocity_mode.cpp)
target_link_libraries(velocity_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(velocity_lcm ${${PEOJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(torque_lcm src/exe/torque_mode.cpp)
target_link_libraries(torque_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(torque_lcm ${${PEOJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(walk_lcm src/exe/walk_mode.cpp)
target_link_libraries(walk_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(walk_lcm ${${PEOJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(lcm_server_3_2 $ENV{UNITREE_LEGGED_SDK_PATH}/examples/lcm_server.cpp)
target_link_libraries(lcm_server_3_2 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(lcm_server_3_2 ${${PEOJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lcm_server_3_1 $ENV{ALIENGO_SDK_PATH}/examples/lcm_server.cpp)
target_link_libraries(lcm_server_3_1 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(lcm_server_3_1 ${${PEOJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})