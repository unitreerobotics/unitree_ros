cmake_minimum_required(VERSION 2.8.3)
project(unitree_controller)

add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")
add_compile_options(-std=c++11)

## Set the path to include model headers. Needs to be modified after using different model.
set(MODEL_HEADER /usr/local/include/UnitreeA1/rcg CACHE PATH "Path to robot model headers")

## Set the path to Pinocchio.
set(PINOCCHIO_HEADER /opt/openrobots/include CACHE PATH "Path to Pinocchio")

## Set the path to InEKF.
set(inekf_INCLUDE_DIRS /home/bmtsz/invariant-ekf/include CACHE PATH "Path to InEKF")

## Set the sources (C++) of your robot models. 
set(SOURCES ../model/transforms.cpp
            ../model/jacobians.cpp
            ../model/jsim.cpp
            ../model/inverse_dynamics.cpp
            ../model/forward_dynamics.cpp
            ../model/inertia_properties.cpp
            ../model/miscellaneous.cpp)

## Set the sources (C++) of InEKF. 
set(INEKF /home/bmtsz/invariant-ekf/src/InEKF.cpp
          /home/bmtsz/invariant-ekf/src/LieGroup.cpp
          /home/bmtsz/invariant-ekf/src/NoiseParams.cpp
          /home/bmtsz/invariant-ekf/src/RobotState.cpp)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    std_msgs
    tf
    geometry_msgs
    unitree_legged_msgs
    rosbag
    urdf
    kdl_parser
)

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(PINOCCHIO pinocchio REQUIRED)

find_package(pinocchio REQUIRED)

find_package(gazebo REQUIRED)

find_package(octomap REQUIRED)

find_package(inekf REQUIRED) 


catkin_package(
    CATKIN_DEPENDS 
    unitree_legged_msgs 
    DEPENDS EIGEN3 PINOCCHIO OCTOMAP
)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${MODEL_HEADER}
    ${PINOCCHIO_HEADER}
    ${PINOCCHIO_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${inekf_INCLUDE_DIRS}
)

# link_directories(${GAZEBO_LIBRARY_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${PINOCCHIO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


# Declare a C++ library
add_library(${PROJECT_NAME}
    src/body.cpp 
    src/a1body.cpp
    src/functions.cpp
    ${SOURCES}
    ${INEKF}
)

add_dependencies(${PROJECT_NAME} unitree_legged_msgs_gencpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${EXTRA_LIBS} ${PINOCCHIO_LIBRARIES} ${OCTOMAP_LIBRARIES}
)

# add_library(unitreeFootContactPlugin SHARED plugin/foot_contact_plugin.cc)
# target_link_libraries(unitreeFootContactPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# add_library(unitreeDrawForcePlugin SHARED plugin/draw_force_plugin.cc)
# target_link_libraries(unitreeDrawForcePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(unitree_external_force src/external_force.cpp)
target_link_libraries(unitree_external_force ${catkin_LIBRARIES})

add_executable(unitree_servo src/servo.cpp)
target_link_libraries(unitree_servo ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(position_mode src/position_exe.cpp)
target_link_libraries(position_mode ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(model_test_FK src/model_test_FK.cpp)
target_link_libraries(model_test_FK ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(pinocchio_test src/pinocchio_test.cpp)
target_link_libraries(pinocchio_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES})
#target_compile_options(RobCoGen_VS_Pinocchio PUBLIC ${PINOCCHIO_CFLAGS_OTHER})

add_executable(velocity_mode src/velocity_exe.cpp)
target_link_libraries(velocity_mode ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(torque_mode src/torque_exe.cpp)
target_link_libraries(torque_mode ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(unitree_move_kinetic src/move_publisher.cpp)
target_link_libraries(unitree_move_kinetic ${catkin_LIBRARIES})
