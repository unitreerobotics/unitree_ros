/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom() {}
    void LCMRecv();

    LCM roslcm;
};

void Custom::LCMRecv()
{
    roslcm.Recv();
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

int main(int argc, char *argv[])
{
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::init(argc, argv, "position_ros_mode");
    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    Custom custom;
    SetLevel(LOWLEVEL);    // must have this
    long motiontime = 0;
    int rate_count = 0;
    int sin_count = 0;
    float qInit[3]={0};
    float qDes[3]={0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};  
    float Kd[3] = {0};
    LowCmd SendLowLCM = {0};
    LowState RecvLowLCM = {0};
    unitree_legged_msgs::LowCmd SendLowROS;
    unitree_legged_msgs::LowState RecvLowROS;
    bool initiated_flag = false;  // initiate need time
    int count = 0;

    custom.roslcm.SubscribeState();

    LoopFunc loop_lcm("LCM_Recv", 0.002, 3, boost::bind(&Custom::LCMRecv, &custom));
    loop_lcm.start();

    SendLowROS.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
        SendLowROS.motorCmd[i].q = PosStopF;        // 禁止位置环
        SendLowROS.motorCmd[i].Kp = 0;
        SendLowROS.motorCmd[i].dq = VelStopF;        // 禁止速度环
        SendLowROS.motorCmd[i].Kd = 0;
        SendLowROS.motorCmd[i].tau = 0;
    }

    while (ros::ok()){
        custom.roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);

        if(initiated_flag == true){
            motiontime++;

            SendLowROS.motorCmd[FR_0].tau = -0.65f;
            SendLowROS.motorCmd[FL_0].tau = +0.65f;
            SendLowROS.motorCmd[RR_0].tau = -0.65f;
            SendLowROS.motorCmd[RL_0].tau = +0.65f;

            // printf("%d\n", motiontime);
            // printf("%d %f %f %f\n", FR_0, RecvLowROS.motorState[FR_0].q, RecvLowROS.motorState[FR_1].q, RecvLowROS.motorState[FR_2].q);
            // printf("%f %f \n",  RecvLowROS.motorState[FR_0].mode, RecvLowROS.motorState[FR_1].mode);
            if( motiontime >= 0){
                // first, get record initial position
                // if( motiontime >= 100 && motiontime < 500){
                if( motiontime >= 0 && motiontime < 10){
                    qInit[0] = RecvLowROS.motorState[FR_0].q;
                    qInit[1] = RecvLowROS.motorState[FR_1].q;
                    qInit[2] = RecvLowROS.motorState[FR_2].q;
                }
                if( motiontime >= 10 && motiontime < 400){
                    // printf("%f %f %f\n", );
                    rate_count++;
                    double rate = rate_count/200.0;                       // needs count to 200
                    Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0;
                    Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
                    
                    qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
                    qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
                    qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
                }
                double sin_joint1, sin_joint2;
                // last, do sine wave
                if( motiontime >= 1700){
                    sin_count++;
                    sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0);
                    sin_joint2 = -0.6 * sin(1.8*M_PI*sin_count/1000.0);
                    qDes[0] = sin_mid_q[0];
                    qDes[1] = sin_mid_q[1];
                    qDes[2] = sin_mid_q[2] + sin_joint2;
                    // qDes[2] = sin_mid_q[2];
                }

                SendLowROS.motorCmd[FR_0].q = qDes[0];
                SendLowROS.motorCmd[FR_0].dq = 0;
                SendLowROS.motorCmd[FR_0].Kp = Kp[0];
                SendLowROS.motorCmd[FR_0].Kd = Kd[0];
                SendLowROS.motorCmd[FR_0].tau = -0.65f;

                SendLowROS.motorCmd[FR_1].q = qDes[1];
                SendLowROS.motorCmd[FR_1].dq = 0;
                SendLowROS.motorCmd[FR_1].Kp = Kp[1];
                SendLowROS.motorCmd[FR_1].Kd = Kd[1];
                SendLowROS.motorCmd[FR_1].tau = 0.0f;

                SendLowROS.motorCmd[FR_2].q =  qDes[2];
                SendLowROS.motorCmd[FR_2].dq = 0;
                SendLowROS.motorCmd[FR_2].Kp = Kp[2];
                SendLowROS.motorCmd[FR_2].Kd = Kd[2];
                SendLowROS.motorCmd[FR_2].tau = 0.0f;

            }

        }

        SendLowLCM = ToLcm(SendLowROS);
        custom.roslcm.Send(SendLowLCM);
        ros::spinOnce();
        loop_rate.sleep();

        count++;
        if(count > 10){
            count = 10;
            initiated_flag = true;
        }
    }
    return 0;
}
