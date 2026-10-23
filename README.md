# Introduction
Here are the ROS simulation packages for Unitree robots, You can load robots and joint controllers in Gazebo, so you can perform low-level control (control the torque, position and angular velocity) of the robot joints. Please be aware that the Gazebo simulation cannot do high-level control, namely walking. Aside from these simulation functions, you can also control your real robots in ROS with the [unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real) packages. For real robots, you can do high-level and low-level control using our ROS packages.

## Packages

### Robot descriptions

Every robot model lives under [`robots/`](robots). See [`robots/README.md`](robots/README.md) for the model files, variants and launch files of each package.

| Robot | Type | Package / model | DOF |
| :--- | :--- | :--- | ---: |
| A1 | Quadruped | [`a1_description`](robots/a1_description) | 12 |
| A2 | Quadruped | [`a2_description`](robots/a2_description) | 12 |
| AlienGo | Quadruped | [`aliengo_description`](robots/aliengo_description) | 12 |
| As2 | Quadruped | [`as2_description`](robots/as2_description) | 12 |
| B1 | Quadruped | [`b1_description`](robots/b1_description) | 12 |
| B2 | Quadruped | [`b2_description`](robots/b2_description) | 12 |
| Go1 | Quadruped | [`go1_description`](robots/go1_description) | 12 |
| Go2 | Quadruped | [`go2_description`](robots/go2_description) | 12 |
| Laikago | Quadruped | [`laikago_description`](robots/laikago_description) | 12 |
| As2-W | Wheeled quadruped | [`as2w_description`](robots/as2w_description) | 12 + 4 |
| B2-W | Wheeled quadruped | [`b2w_description`](robots/b2w_description) | 12 + 4 |
| Go2-W | Wheeled quadruped | [`go2w_description`](robots/go2w_description) | 12 + 4 |
| G1 | Humanoid | [`g1_description`](robots/g1_description) | 23 / 29 |
| G1+ | Humanoid | [`g1_pl.urdf`](robots/g1_description/g1_pl.urdf) | 31 |
| G1 with BrainCo Hand | Humanoid | [`g1_with_brainco_hand`](robots/g1_with_brainco_hand) | 61 |
| H1 | Humanoid | [`h1_description`](robots/h1_description) | 19 (45) |
| H1-2 | Humanoid | [`h1_2_description`](robots/h1_2_description) | 27 (51) |
| H2 | Humanoid | [`h2_description`](robots/h2_description) | 31 |
| H2 Plus | Humanoid | [`h2_plus`](robots/h2_plus) | 31 (75) |
| H2-X | Humanoid | [`h2x_description`](robots/h2x_description) | 31 (37) |
| R1 | Humanoid | [`r1_description`](robots/r1_description) | 26 |
| R1 Air | Humanoid | [`r1_air_description`](robots/r1_air_description) | 20 |
| G1-D | Wheeled humanoid | [`g1_d_description`](robots/g1_d_description) | 17 / 19 |
| H2-D | Wheeled humanoid | [`h2d_description`](robots/h2d_description) | 19 |
| R1-A7-D | Wheeled mobile manipulator | [`r1d_description`](robots/r1d_description) | 21 |
| AlienGo with Z1 | Mobile manipulator | [`aliengoZ1_description`](robots/aliengoZ1_description) | 12 + 6 |
| R1-7a | Robotic arm | [`r1_7a_description`](robots/r1_7a_description) | 7 |
| R1-A5 | Dual-arm upper body | [`r1_a5_description`](robots/r1_a5_description) | 13 |
| R1-A7 | Dual-arm upper body | [`r1_a7_description`](robots/r1_a7_description) | 17 |
| Z1 | Robotic arm | [`z1_description`](robots/z1_description) | 6 |
| Dex1-1 / Dex2/5 / Dex3-1 / Dex5-1 | Dexterous hands | [`dexterous_hand_description`](robots/dexterous_hand_description) | 2 / 10 / 7 / 20 |
| B2 (MuJoCo) | Quadruped | [`b2_description_mujoco`](robots/b2_description_mujoco) | 12 |

`DOF` counts the actuated joints in the model files: `23 / 29` denotes model variants, `19 (45)` is the body DOF with dexterous hands, `31 (37)` adds transmission joints, `12 + 4` adds the wheel joints, and the hand figures are actuated joints per hand. `G1+` is a URDF variant (`g1_pl.urdf`) inside `g1_description` rather than a separate package.

### Controllers, simulation and messages

| Package | Description |
| :--- | :--- |
| [`unitree_controller`](unitree_controller) | Stand/servo nodes and disturbance or pose publishers: `unitree_servo`, `unitree_external_force`, `unitree_move_kinetic` |
| [`unitree_gazebo`](unitree_gazebo) | Gazebo launch files, worlds and plugins (foot contact force, force drawing) |
| [`unitree_legged_control`](unitree_legged_control) | `controller_manager` plugin `UnitreeJointController` for position, velocity and torque control |
| [`unitree_ros_to_real`](unitree_ros_to_real) | Submodule with `unitree_legged_msgs` and the real-robot ROS packages |

# Dependencies
* [ROS](https://www.ros.org/) Melodic or ROS Kinetic (has not been tested)
* [Gazebo8](http://gazebosim.org/)
* [unitree_legged_msgs](https://github.com/unitreerobotics/unitree_ros_to_real): `unitree_legged_msgs` is a package under [unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real).
# Build
<!-- If you would like to fully compile the `unitree_ros`, please run the following command to install relative packages. -->

For ROS Melodic:
```
sudo apt-get install ros-melodic-controller-interface  ros-melodic-gazebo-ros-control ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller
```
For ROS Kinetic:
```
sudo apt-get install ros-kinetic-controller-manager ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-joint-state-controller ros-kinetic-effort-controllers ros-kinetic-velocity-controllers ros-kinetic-position-controllers ros-kinetic-robot-controllers ros-kinetic-robot-state-publisher ros-kinetic-gazebo8-ros ros-kinetic-gazebo8-ros-control ros-kinetic-gazebo8-ros-pkgs ros-kinetic-gazebo8-ros-dev
```

And open the file `unitree_gazebo/worlds/stairs.world`. At the end of the file:
```
<include>
    <uri>model:///home/unitree/catkin_ws/src/unitree_ros/unitree_gazebo/worlds/building_editor_models/stairs</uri>
</include>
```
Please change the path of `building_editor_models/stairs` to the real path on your PC.

Then you can use catkin_make to build:
```
cd ~/catkin_ws
catkin_make
```

If you face a dependency problem, you can just run `catkin_make` again.

# Detail of Packages
## unitree_legged_control:
It contains the joints controllers for Gazebo simulation, which allows users to control joints with position, velocity and torque. Refer to "[unitree_ros/unitree_controller/src/servo.cpp](https://github.com/unitreerobotics/unitree_ros/blob/master/unitree_controller/src/servo.cpp)" for joint control examples in different modes.

## The description of robots:
Each robot has a description package under `robots/`, containing meshes plus URDF and Xacro files (and MJCF where available). The [robot table](#robot-descriptions) lists every model with its DOF; see [`robots/README.md`](robots/README.md) for variants and launch files. Take Laikago for example, you can check the model in Rviz by:
```
roslaunch laikago_description laikago_rviz.launch
```

## unitree_gazebo & unitree_controller:
You can launch the Gazebo simulation with the following command:
```
roslaunch unitree_gazebo normal.launch rname:=a1 wname:=stairs
```
Where the `rname` means robot name. Robots with a complete Gazebo controller setup (`xacro/robot.xacro` and `config/robot_control.yaml`) are `laikago`, `aliengo`, `a1`, `b1`, `b2`, `go1` and `go2`; the remaining descriptions ship RViz or display launch files only. The `wname` means world name, which can be `earth`, `space` or `stairs`. And the default value of `rname` is `laikago`, while the default value of `wname` is `earth`. In Gazebo, the robot should be lying on the ground with joints not activated.

### 1. Stand controller
After launching the gazebo simulation, you can start to control the robot:
```
rosrun unitree_controller unitree_servo
```

And you can add external disturbances, like a push or a kick:
```
rosrun unitree_controller unitree_external_force
```
### 2. Position and pose publisher
Here we demonstrated how to control the position and pose of robot without a controller, which should be useful in SLAM or visual development.

Then run the position and pose publisher in another terminal:
```
rosrun unitree_controller unitree_move_kinetic
```
The robot will turn around the origin, which is the movement under the world coordinate frame. And inside of the source file [move_publisher.cpp](https://github.com/unitreerobotics/unitree_ros/blob/master/unitree_controller/src/move_publisher.cpp), we also provide the method to move using the robot coordinate frame. You can change the value of `def_frame` to `coord::ROBOT` and run the catkin_make again, then the `unitree_move_publisher` will move robot under its own coordinate frame.

## Z1

You can launch the z1 Gazebo simulation with the following command:

```
roslaunch unitree_gazebo z1.launch
```

After launching the gazebo simulation, you can start to control the z1 robot by z1_sdk.  
see [z1_documentation](https://dev-z1.unitree.com)