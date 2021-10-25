/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef __A1BODY_H__
#define __A1BODY_H__

#include "ros/ros.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/HighState.h"
#define PosStopF (2.146E+9f)
#define VelStopF (16000.f)

namespace a1_model {

extern ros::Publisher servo_pub[12];
extern ros::Publisher highState_pub;
extern unitree_legged_msgs::LowCmd lowCmd;
extern unitree_legged_msgs::LowState lowState;

void paramInit();
void stand();
void motion_init();
void sendServoCmd();
void moveAllPosition(double* jointPositions, double duration);

// definition of each leg and joint
constexpr int FR_ = 0;       // leg index
constexpr int FL_ = 1;
constexpr int RR_ = 2;
constexpr int RL_ = 3;

constexpr int FR_0 = 0;      // joint index
constexpr int FR_1 = 1;      
constexpr int FR_2 = 2;

constexpr int FL_0 = 3;
constexpr int FL_1 = 4;
constexpr int FL_2 = 5;

constexpr int RR_0 = 6;
constexpr int RR_1 = 7;
constexpr int RR_2 = 8;

constexpr int RL_0 = 9;
constexpr int RL_1 = 10;
constexpr int RL_2 = 11;
}

#endif
