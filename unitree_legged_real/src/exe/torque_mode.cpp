/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

void* update_loop(void* param)
{
    LCM *data = (LCM *)param; 
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

int main(int argc, char *argv[])
{
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::init(argc, argv, "torque_ros_mode");
    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    std::string robot_name;
    LeggedType rname;
    ros::param::get("/robot_name", robot_name);
    if(strcasecmp(robot_name.c_str(), "A1") == 0)
        rname = LeggedType::A1;
    else if(strcasecmp(robot_name.c_str(), "Aliengo") == 0)
        rname = LeggedType::Aliengo;

    Control control(rname, LOWLEVEL);

    long motiontime=0;
    float torque = 0;
    LowCmd SendLowLCM = {0};
    LowState RecvLowLCM = {0};
    unitree_legged_msgs::LowCmd SendLowROS;
    unitree_legged_msgs::LowState RecvLowROS;
    LCM roslcm;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop, &roslcm);

    SendLowROS.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
    }

    while (ros::ok()){
        motiontime++;
        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);
        printf("FL_1 position: %f\n",  RecvLowROS.motorState[FL_1].q);

        // gravity compensation
        SendLowROS.motorCmd[FR_0].tau = -0.65f;
        SendLowROS.motorCmd[FL_0].tau = +0.65f;
        SendLowROS.motorCmd[RR_0].tau = -0.65f;
        SendLowROS.motorCmd[RL_0].tau = +0.65f;

        if( motiontime >= 500){
            torque = (0 - RecvLowROS.motorState[FL_1].q)*10.0f + (0 - RecvLowROS.motorState[FL_1].dq)*1.0f;
            if(torque > 5.0f) torque = 5.0f;
            if(torque < -5.0f) torque = -5.0f;

            SendLowROS.motorCmd[FL_1].q = PosStopF;
            SendLowROS.motorCmd[FL_1].dq = VelStopF;
            SendLowROS.motorCmd[FL_1].Kp = 0;
            SendLowROS.motorCmd[FL_1].Kd = 0;
            SendLowROS.motorCmd[FL_1].tau = torque;
        }
        SendLowLCM = ToLcm(SendLowROS);
        roslcm.Send(SendLowLCM);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
