/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef _CONVERT_H_
#define _CONVERT_H_

#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/IMU.h>

#ifdef SDK3_1
#include "aliengo_sdk/aliengo_sdk.hpp"

unitree_legged_msgs::IMU ToRos(aliengo::IMU& lcm){
    unitree_legged_msgs::IMU ros;
    ros.quaternion[0] = lcm.quaternion[0];
    ros.quaternion[1] = lcm.quaternion[1];
    ros.quaternion[2] = lcm.quaternion[2];
    ros.quaternion[3] = lcm.quaternion[3];
    ros.gyroscope[0] = lcm.gyroscope[0];
    ros.gyroscope[1] = lcm.gyroscope[1];
    ros.gyroscope[2] = lcm.gyroscope[2];
    ros.accelerometer[0] = lcm.acceleration[0];
    ros.accelerometer[1] = lcm.acceleration[1];
    ros.accelerometer[2] = lcm.acceleration[2];
    // ros.rpy[0] = lcm.rpy[0];
    // ros.rpy[1] = lcm.rpy[1];
    // ros.rpy[2] = lcm.rpy[2];
    ros.temperature = lcm.temp;
    return ros;
}

unitree_legged_msgs::MotorState ToRos(aliengo::MotorState& lcm){
    unitree_legged_msgs::MotorState ros;
    ros.mode = lcm.mode;
    ros.q = lcm.position;
    ros.dq = lcm.velocity;
    ros.ddq = 0;
    ros.tauEst = lcm.torque;
    ros.q_raw = 0;
    ros.dq_raw = 0;
    ros.ddq_raw = 0;
    ros.temperature = lcm.temperature;
    return ros;
}

aliengo::MotorCmd ToLcm(unitree_legged_msgs::MotorCmd& ros, aliengo::MotorCmd lcmType){
    aliengo::MotorCmd lcm;
    lcm.mode = ros.mode;
    lcm.position = ros.q;
    lcm.velocity = ros.dq;
    lcm.positionStiffness = ros.Kp;
    lcm.velocityStiffness = ros.Kd;
    lcm.torque = ros.tau;
    return lcm;
}

unitree_legged_msgs::LowState ToRos(aliengo::LowState& lcm){
    unitree_legged_msgs::LowState ros;
    ros.levelFlag = lcm.levelFlag;
    ros.commVersion = 0;
    ros.robotID = 0;
    ros.SN = 0;
    ros.bandWidth = 0;
    ros.imu = ToRos(lcm.imu);
    for(int i = 0; i < 20; i++){
        ros.motorState[i] = ToRos(lcm.motorState[i]);
    }
    for(int i = 0; i < 4; i++){
        ros.footForce[i] = lcm.footForce[i];
        ros.footForceEst[i] = 0;
    }
    ros.tick = lcm.tick;
    for(int i = 0; i < 40; i++){
        ros.wirelessRemote[i] = lcm.wirelessRemote[i];
    }
    ros.crc = lcm.crc;

    return ros;
}

aliengo::LowCmd ToLcm(unitree_legged_msgs::LowCmd& ros, aliengo::LowCmd lcmType){
    aliengo::LowCmd lcm;
    lcm.levelFlag = ros.levelFlag;
    for(int i = 0; i < 20; i++){
        lcm.motorCmd[i] = ToLcm(ros.motorCmd[i], lcm.motorCmd[i]);
    }
    for(int i = 0; i < 4; i++){
        lcm.led[i].r = ros.led[i].r;
        lcm.led[i].g = ros.led[i].g;
        lcm.led[i].b = ros.led[i].b;
    }
    for(int i = 0; i < 40; i++){
        lcm.wirelessRemote[i] = ros.wirelessRemote[i];
    }
    lcm.crc = ros.crc;
    return lcm;
}

unitree_legged_msgs::HighState ToRos(aliengo::HighState& lcm){
    unitree_legged_msgs::HighState ros;
    ros.levelFlag = lcm.levelFlag;
    ros.commVersion = 0;
    ros.robotID = 0;
    ros.SN = 0;
    ros.bandWidth = 0;
    ros.mode = lcm.mode;
    ros.imu = ToRos(lcm.imu);
    ros.forwardSpeed = lcm.forwardSpeed;
    ros.sideSpeed = lcm.sideSpeed;
    ros.rotateSpeed = lcm.rotateSpeed;
    ros.bodyHeight = lcm.bodyHeight;
    ros.updownSpeed = lcm.updownSpeed;
    ros.forwardPosition = lcm.forwardPosition.x;
    ros.sidePosition = lcm.sidePosition.y;
    for(int i = 0; i < 4; i++){
        ros.footPosition2Body[i].x = lcm.footPosition2Body[i].x;
        ros.footPosition2Body[i].y = lcm.footPosition2Body[i].y;
        ros.footPosition2Body[i].z = lcm.footPosition2Body[i].z;
        ros.footSpeed2Body[i].x = lcm.footSpeed2Body[i].x;
        ros.footSpeed2Body[i].y = lcm.footSpeed2Body[i].y;
        ros.footSpeed2Body[i].z = lcm.footSpeed2Body[i].z;
        ros.footForce[i] = lcm.footForce[i];
        ros.footForceEst[i] = 0;
    }
    ros.tick = lcm.tick;
    for(int i = 0; i<40; i++){
        ros.wirelessRemote[i] = lcm.wirelessRemote[i];
    }
    ros.reserve = 0;
    ros.crc = lcm.crc;
    return ros;
}

aliengo::HighCmd ToLcm(unitree_legged_msgs::HighCmd& ros, aliengo::HighCmd lcmType){
    aliengo::HighCmd lcm;
    lcm.levelFlag = ros.levelFlag;
    lcm.mode = ros.mode;
    lcm.forwardSpeed = ros.forwardSpeed;
    lcm.sideSpeed = ros.sideSpeed;
    lcm.rotateSpeed = ros.rotateSpeed;
    lcm.bodyHeight = ros.bodyHeight;
    lcm.footRaiseHeight = ros.footRaiseHeight;
    lcm.yaw = ros.yaw;
    lcm.pitch = ros.pitch;
    lcm.roll = ros.roll;
    for(int i = 0; i < 4; i++){
        lcm.led[i].r = ros.led[i].r;
        lcm.led[i].g = ros.led[i].g;
        lcm.led[i].b = ros.led[i].b;
    }
    for(int i = 0; i < 40; i++){
        lcm.wirelessRemote[i] = ros.wirelessRemote[i];
    }
    lcm.crc = ros.crc;
    return lcm;
}
#endif

#ifdef SDK3_2
#include "unitree_legged_sdk/unitree_legged_sdk.h"

unitree_legged_msgs::IMU ToRos(UNITREE_LEGGED_SDK::IMU& lcm)
{
    unitree_legged_msgs::IMU ros;
    ros.quaternion[0] = lcm.quaternion[0];
    ros.quaternion[1] = lcm.quaternion[1];
    ros.quaternion[2] = lcm.quaternion[2];
    ros.quaternion[3] = lcm.quaternion[3];
    ros.gyroscope[0] = lcm.gyroscope[0];
    ros.gyroscope[1] = lcm.gyroscope[1];
    ros.gyroscope[2] = lcm.gyroscope[2];
    ros.accelerometer[0] = lcm.accelerometer[0];
    ros.accelerometer[1] = lcm.accelerometer[1];
    ros.accelerometer[2] = lcm.accelerometer[2];
    // ros.rpy[0] = lcm.rpy[0];
    // ros.rpy[1] = lcm.rpy[1];
    // ros.rpy[2] = lcm.rpy[2];
    ros.temperature = lcm.temperature;
    return ros;
}

unitree_legged_msgs::MotorState ToRos(UNITREE_LEGGED_SDK::MotorState& lcm)
{
    unitree_legged_msgs::MotorState ros;
    ros.mode = lcm.mode;
    ros.q = lcm.q;
    ros.dq = lcm.dq;
    ros.ddq = lcm.ddq;
    ros.tauEst = lcm.tauEst;
    ros.q_raw = lcm.q_raw;
    ros.dq_raw = lcm.dq_raw;
    ros.ddq_raw = lcm.ddq_raw;
    ros.temperature = lcm.temperature;
    ros.reserve[0] = lcm.reserve[0];
    ros.reserve[1] = lcm.reserve[1];
    return ros;
}

UNITREE_LEGGED_SDK::MotorCmd ToLcm(unitree_legged_msgs::MotorCmd& ros, UNITREE_LEGGED_SDK::MotorCmd lcmType)
{
    UNITREE_LEGGED_SDK::MotorCmd lcm;
    lcm.mode = ros.mode;
    lcm.q = ros.q;
    lcm.dq = ros.dq;
    lcm.tau = ros.tau;
    lcm.Kp = ros.Kp;
    lcm.Kd = ros.Kd;
    lcm.reserve[0] = ros.reserve[0];
    lcm.reserve[1] = ros.reserve[1];
    lcm.reserve[2] = ros.reserve[2];
    return lcm;
}

unitree_legged_msgs::LowState ToRos(UNITREE_LEGGED_SDK::LowState& lcm)
{
    unitree_legged_msgs::LowState ros;
    ros.levelFlag = lcm.levelFlag;
    ros.commVersion = lcm.commVersion;
    ros.robotID = lcm.robotID;
    ros.SN = lcm.SN;
    ros.bandWidth = lcm.bandWidth;
    ros.imu = ToRos(lcm.imu);
    for(int i = 0; i<20; i++){
        ros.motorState[i] = ToRos(lcm.motorState[i]);
    }
    for(int i = 0; i<4; i++){
        ros.footForce[i] = lcm.footForce[i];
        ros.footForceEst[i] = lcm.footForceEst[i];
    }
    ros.tick = lcm.tick;
    for(int i = 0; i<40; i++){
        ros.wirelessRemote[i] = lcm.wirelessRemote[i];
    }
    ros.reserve = lcm.reserve;
    ros.crc = lcm.crc;
    return ros;
}

UNITREE_LEGGED_SDK::LowCmd ToLcm(unitree_legged_msgs::LowCmd& ros, UNITREE_LEGGED_SDK::LowCmd lcmType)
{
    UNITREE_LEGGED_SDK::LowCmd lcm;
    lcm.levelFlag = ros.levelFlag;
    lcm.commVersion = ros.commVersion;
    lcm.robotID = ros.robotID;
    lcm.SN = ros.SN;
    lcm.bandWidth = ros.bandWidth;
    for(int i = 0; i<20; i++){
        lcm.motorCmd[i] = ToLcm(ros.motorCmd[i], lcm.motorCmd[i]);
    }
    for(int i = 0; i<4; i++){
        lcm.led[i].r = ros.led[i].r;
        lcm.led[i].g = ros.led[i].g;
        lcm.led[i].b = ros.led[i].b;
    }
    for(int i = 0; i<40; i++){
        lcm.wirelessRemote[i] = ros.wirelessRemote[i];
    }
    lcm.reserve = ros.reserve;
    lcm.crc = ros.crc;
    return lcm;
}

unitree_legged_msgs::HighState ToRos(UNITREE_LEGGED_SDK::HighState& lcm)
{
    unitree_legged_msgs::HighState ros;
    ros.levelFlag = lcm.levelFlag;
    ros.commVersion = lcm.commVersion;
    ros.robotID = lcm.robotID;
    ros.SN = lcm.SN;
    ros.bandWidth = lcm.bandWidth;
    ros.mode = lcm.mode;
    ros.imu = ToRos(lcm.imu);
    ros.forwardSpeed = lcm.forwardSpeed;
    ros.sideSpeed = lcm.sideSpeed;
    ros.rotateSpeed = lcm.rotateSpeed;
    ros.bodyHeight = lcm.bodyHeight;
    ros.updownSpeed = lcm.updownSpeed;
    ros.forwardPosition = lcm.forwardPosition;
    ros.sidePosition = lcm.sidePosition;
    for(int i = 0; i<4; i++){
        ros.footPosition2Body[i].x = lcm.footPosition2Body[i].x;
        ros.footPosition2Body[i].y = lcm.footPosition2Body[i].y;
        ros.footPosition2Body[i].z = lcm.footPosition2Body[i].z;
        ros.footSpeed2Body[i].x = lcm.footSpeed2Body[i].x;
        ros.footSpeed2Body[i].y = lcm.footSpeed2Body[i].y;
        ros.footSpeed2Body[i].z = lcm.footSpeed2Body[i].z;
        ros.footForce[i] = lcm.footForce[i];
        ros.footForceEst[i] = lcm.footForceEst[i];
    }
    ros.tick = lcm.tick;
    for(int i = 0; i<40; i++){
        ros.wirelessRemote[i] = lcm.wirelessRemote[i];
    }
    ros.reserve = lcm.reserve;
    ros.crc = lcm.crc;
    return ros;
}

UNITREE_LEGGED_SDK::HighCmd ToLcm(unitree_legged_msgs::HighCmd& ros, UNITREE_LEGGED_SDK::HighCmd lcmType)
{
    UNITREE_LEGGED_SDK::HighCmd lcm;
    lcm.levelFlag = ros.levelFlag;
    lcm.commVersion = ros.commVersion;
    lcm.robotID = ros.robotID;
    lcm.SN = ros.SN;
    lcm.bandWidth = ros.bandWidth;
    lcm.mode = ros.mode;
    lcm.forwardSpeed = ros.forwardSpeed;
    lcm.sideSpeed = ros.sideSpeed;
    lcm.rotateSpeed = ros.rotateSpeed;
    lcm.bodyHeight = ros.bodyHeight;
    lcm.footRaiseHeight = ros.footRaiseHeight;
    lcm.yaw = ros.yaw;
    lcm.pitch = ros.pitch;
    lcm.roll = ros.roll;
    for(int i = 0; i<4; i++){
        lcm.led[i].r = ros.led[i].r;
        lcm.led[i].g = ros.led[i].g;
        lcm.led[i].b = ros.led[i].b;
    }
    for(int i = 0; i<40; i++){
        lcm.wirelessRemote[i] = ros.wirelessRemote[i];
        lcm.AppRemote[i] = ros.AppRemote[i];
    }
    lcm.reserve = ros.reserve;
    lcm.crc = ros.crc;
    return lcm;
}
#endif

#endif  // _CONVERT_H_