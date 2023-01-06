#ifndef __UNITREEARM_H
#define __UNITREEARM_H

#include <ros/ros.h>
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"

namespace unitreeArm {

class IOROS
{
public:
    IOROS();
    ~IOROS();

    void sendCmd(double* targetPos, double duration);

private:
    void _joint01Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint02Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint03Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint04Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint05Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint06Callback(const unitree_legged_msgs::MotorState& msg);
    void _gripperCallback(const unitree_legged_msgs::MotorState& msg);

    ros::NodeHandle _nm;
    ros::Publisher  _servo_pub[7];
    ros::Subscriber _servo_sub[7];
    unitree_legged_msgs::MotorCmd   _joint_cmd[7];
    unitree_legged_msgs::MotorState _joint_state[7];

};

}

#endif
