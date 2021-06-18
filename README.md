# Introduction
Here are the ROS packages of Unitree robots, namely Laikago, Aliengo and A1. You can load robots and joint controllers in Gazebo, so you can do low-level control(control the torque, position and angular velocity) on the robot joints. Please watch out that the Gazebo simulation cannot do high-level control, namely walking. Besides of these simulation functions, you can also control your real robots in ROS by the `unitree_legged_real`. For real robots, you can do high-level and low-level control by our ROS packages.

## Packages:
Robot description: `a1_description`, `aliengo_description`, `laikago_description`

Robot and joints controller: `unitree_controller`

Basic function: `unitree_legged_msgs`

Simulation related: `unitree_gazebo`, `unitree_legged_control`

Real robot control related: `unitree_legged_real`

# Dependencies
* [ROS](https://www.ros.org/) melodic or ROS kinetic(has not been tested)
* [Gazebo8](http://gazebosim.org/)
* [unitree_legged_sdk](https://github.com/unitreerobotics): If your robot is suitable for `unitree_legged_sdk`, then you do not need `aliengo_sdk`.
* [aliengo_sdk](https://github.com/unitreerobotics): If your robot is suitable for `aliengo_sdk`, then you do not need `unitree_legged_sdk`.

# Configuration
Make sure the following exist in your `~/.bashrc` file or export them in terminal. `melodic`, `gazebo-8`, `~/catkin_ws`, `amd64` and the paths to `unitree_legged_sdk` should be replaced in your own case. 
If your use `unitree_legged_sdk`, then you need to set `UNITREE_SDK_VERSION=3_2` and the path `UNITREE_LEGGED_SDK_PATH`.
Otherwise, if you use `aliengo_sdk`, you need to set `UNITREE_SDK_VERSION=3_1` and the path `ALIENGO_SDK_PATH`.

```
source /opt/ros/melodic/setup.bash
source /usr/share/gazebo-8/setup.sh
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=~/catkin_ws:${ROS_PACKAGE_PATH}
export GAZEBO_PLUGIN_PATH=~/catkin_ws/devel/lib:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=~/catkin_ws/devel/lib:${LD_LIBRARY_PATH}
# 3_1, 3_2
export UNITREE_SDK_VERSION=3_2
export UNITREE_LEGGED_SDK_PATH=~/unitree_legged_sdk
export ALIENGO_SDK_PATH=~/aliengo_sdk
# amd64, arm32, arm64
export UNITREE_PLATFORM="amd64"
```

# Build
If you just need to send message from the ROS on robot's NX or TX2, you can just copy `unitree_legged_msgs` and `unitree_legged_real` to `catkin_ws/src` folder and run `catkin_make` to compile.

If you would like to fully compile the `unitree_ros`, please run the following command to install relative packages.

If your ROS is melodic:
```
sudo apt-get install ros-melodic-controller-interface  ros-melodic-gazebo-ros-control ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller
```
Else if your ROS is kinetic:
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
It contains the joints controllers for Gazebo simulation, which allows user to control joints with position, velocity and torque.

## unitree_legged_msgs:
ros-type message, including command and state of high-level and low-level control.
It would be better if it be compiled firstly, otherwise you may have dependency problems (such as that you can't find the header file).

## The description of robots:
Namely the description of A1, Aliengo and Laikago. Each package include mesh, urdf and xacro files of robot. Take Laikago as an example, you can check the model in Rviz by:
```
roslaunch laikago_description laikago_rviz.launch
```

## unitree_gazebo & unitree_controller:
You can launch the Gazebo simulation by the following command:
```
roslaunch unitree_gazebo normal.launch rname:=a1 wname:=stairs
```
Where the `rname` means robot name, which can be `laikago`, `aliengo` or `a1`. The `wname` means world name, which can be `earth`, `space` or `stairs`. And the default value of `rname` is `laikago`, while the default value of `wname` is `earth`. In Gazebo, the robot should be lying on the ground with joints not activated.

### Stand controller
After launching the gazebo simulation, you can start to control the robot:
```
rosrun unitree_controller unitree_servo
```

And you can add external disturbances, like a push or a kick:
```
rosrun unitree_controller unitree_external_force
```
### Position and pose publisher
Here we showed how to control the position and pose of robot without a controller, which should be useful in SLAM or visual development.

Then run the position and pose publisher in another terminal:
```
rosrun unitree_controller unitree_move_kinetic
```
The robot will turn around the origin, which is the movement under the world coordinate. And inside of the source file `move_publisher`, we also offered the method to move robot under robot coordinate. You can change the value of `def_frame` to `coord::ROBOT` and run the catkin_make again, then the `unitree_move_publisher` will move robot under its own coordinate.

## unitree_legged_real
### Setup the net connection
First, please connect the network cable between your PC and robot. Then run `ifconfig` in a terminal, you will find your port name. For example, `enx000ec6612921`.

Then, open the `ipconfig.sh` file under the folder `unitree_legged_real`, modify the port name to your own. And run the following commands:
```
sudo chmod +x ipconfig.sh
sudo ./ipconfig.sh
```
If you run the `ifconfig` again, you will find that port has `inet` and `netmask` now.
In order to set your port automatically, you can modify `interfaces`:
```
sudo gedit /etc/network/interfaces
```
And add the following 4 lines at the end:
```
auto enx000ec6612921
iface enx000ec6612921 inet static
address 192.168.123.162
netmask 255.255.255.0
```
Where the port name have to change to your own.


### Run the package
You can control your real robot(only A1 and Aliengo) from ROS by this package.

First you have to run the `real_launch` under root account:
```
sudo su
source /home/yourUserName/catkin_ws/devel/setup.bash
roslaunch unitree_legged_real real.launch rname:=a1 ctrl_level:=highlevel firmwork:=3_2
```
Please watchout that the `/home/yourUserName` means the home directory of yourself. These commands will launch a LCM server. The `rname` means robot name, which can be `a1` or `aliengo`(case does not matter), and the default value is `a1`. And the `ctrl_level` means the control level, which can be `lowlevel` or `highlevel`(case does not matter), and the default value is `highlevel`. Under the low level, you can control the joints directly. And under the high level, you can control the robot to move or change its pose. The `firmwork` means the firmwork version of the robot. The default value is `3_2` Now all the A1's firmwork version is `3_2`, and most Aliengo's firmwork version is `3_1`.(will update in the future)

To do so, you need to run the controller in another terminal(also under root account):
```
rosrun unitree_legged_real position_lcm
```
We offered some examples. When you run the low level controller, please make sure the robot is hanging up. The low level contains:
```
position_lcm
velocity_lcm
torque_lcm
```
The `velocity_lcm` and `torque_lcm` have to run under root account too. Please use the same method as runing `real_launch`.

And when you run the high level controller, please make sure the robot is standing on the ground. The high level only has `walk_lcm`.