/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
#include <Eigen/Geometry> 

#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "convert.h"
#include <rosbag/bag.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#ifdef SDK3_1
using namespace aliengo;
#endif
#ifdef SDK3_2
using namespace UNITREE_LEGGED_SDK;
#endif

static void toEulerAngle(const Eigen::Quaternionf& q, float& roll, float& pitch, float& yaw)
{
// roll (x-axis rotation)
float sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
float cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
roll = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
float sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
// if (fabs(sinp) >= 1)
// pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
// else
pitch = asin(sinp);

// yaw (z-axis rotation)
float siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
float cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
yaw = atan2(siny_cosp, cosy_cosp);
}

template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    // SetLevel(HIGHLEVEL);
    long motiontime = 0;
    TCmd SendHighLCM = {0};
    TState RecvHighLCM = {0};
    unitree_legged_msgs::HighCmd SendHighROS;
    unitree_legged_msgs::HighState RecvHighROS;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    ros::Time begin_time = ros::Time::now();
    ros::Duration elapsed_Dur_time;

    rosbag::Bag bag;
    bag.open("/home/bmtsz/test_out/walk_real.bag", rosbag::bagmode::Write);

    Eigen::Quaternionf quat;    // Definition of quaternion
    std_msgs::Float32 euler_angle[3];
    std_msgs::Float32 sagittal_pos;
    std_msgs::Float32 lateral_pos; 

    while (ros::ok()){
        motiontime = motiontime+2;
        roslcm.Get(RecvHighLCM);
        RecvHighROS = ToRos(RecvHighLCM);
        // printf("%f\n",  RecvHighROS.forwardSpeed);

        SendHighROS.forwardSpeed = 0.0f;
        SendHighROS.sideSpeed = 0.0f;
        SendHighROS.rotateSpeed = 0.0f;
        SendHighROS.bodyHeight = 0.0f;

        SendHighROS.mode = 0;
        SendHighROS.roll  = 0;
        SendHighROS.pitch = 0;
        SendHighROS.yaw = 0;

        if(motiontime>1000 && motiontime<1500){
            SendHighROS.mode = 1;
            // SendHighROS.roll = 0.3f;
        }

        if(motiontime>1500 && motiontime<2000){
            SendHighROS.mode = 1;
            SendHighROS.pitch = 0.3f;
        }

        if(motiontime>2000 && motiontime<2500){
            SendHighROS.mode = 1;
            SendHighROS.yaw = 0.2f;
        }

        if(motiontime>2500 && motiontime<3000){
            SendHighROS.mode = 1;
            SendHighROS.bodyHeight = -0.3f;
        }

        if(motiontime>3000 && motiontime<3500){
            SendHighROS.mode = 1;
            SendHighROS.bodyHeight = 0.3f;
        }

        if(motiontime>3500 && motiontime<4000){
            SendHighROS.mode = 1;
            SendHighROS.bodyHeight = 0.0f;
        }

        if(motiontime>4000 && motiontime<5000){
            SendHighROS.mode = 2;
        }

        if(motiontime>5000 && motiontime<8500){
            SendHighROS.mode = 2;
            SendHighROS.forwardSpeed = 0.1f; // -1  ~ +1
        }

        if(motiontime>8500 && motiontime<12000){
            SendHighROS.mode = 2;
            SendHighROS.forwardSpeed = -0.2f; // -1  ~ +1
        }

        if(motiontime>12000 && motiontime<16000){
            SendHighROS.mode = 2;
            SendHighROS.rotateSpeed = 0.1f;   // turn
        }

        if(motiontime>16000 && motiontime<20000){
            SendHighROS.mode = 2;
            SendHighROS.rotateSpeed = -0.1f;   // turn
        }

        if(motiontime>20000 && motiontime<21000){
            SendHighROS.mode = 1;
        }

        elapsed_Dur_time = ros::Time::now() - begin_time;
        ros::Time elapsed_time(elapsed_Dur_time.toSec());
        
        // for(int i = 0; i<4; i++){
        //     quanternion[i].data = RecvHighROS.imu.quaternion[i];
        // }
        sagittal_pos.data = RecvHighROS.forwardPosition;
        lateral_pos.data = RecvHighROS.sidePosition;
        quat.w() = RecvHighROS.imu.quaternion[0];
        quat.x() = RecvHighROS.imu.quaternion[1];
        quat.y() = RecvHighROS.imu.quaternion[2];
        quat.z() = RecvHighROS.imu.quaternion[3];
        toEulerAngle(quat, euler_angle[0].data, euler_angle[1].data, euler_angle[2].data);
        bag.write("/trunk_imu/roll", elapsed_time, euler_angle[0]);
        bag.write("/trunk_imu/pitch", elapsed_time, euler_angle[1]);
        bag.write("/trunk_imu/yaw", elapsed_time, euler_angle[2]);
        bag.write("/sagittal_pos", elapsed_time, sagittal_pos);
        bag.write("/lateral_pos", elapsed_time, lateral_pos);
        std::cout << "Roll : " << euler_angle[0].data 
             << ", Pitch: " << euler_angle[1].data 
             << ", Yaw: " << euler_angle[2].data << std::endl;
        std::cout << "forwardPosition : " << sagittal_pos.data 
                  << ", sidePosition: " << lateral_pos.data << std::endl;

        SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
        roslcm.Send(SendHighLCM);
        ros::spinOnce();
        loop_rate.sleep(); 
    }
    return 0;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "walk_ros_mode");
    std::string firmwork;
    ros::param::get("/firmwork", firmwork);

    #ifdef SDK3_1
        aliengo::Control control(aliengo::HIGHLEVEL);
        aliengo::LCM roslcm;
        mainHelper<aliengo::HighCmd, aliengo::HighState, aliengo::LCM>(argc, argv, roslcm);
    #endif

    #ifdef SDK3_2
        std::string robot_name;
        UNITREE_LEGGED_SDK::LeggedType rname;
        ros::param::get("/robot_name", robot_name);
        if(strcasecmp(robot_name.c_str(), "A1") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::A1;
        else if(strcasecmp(robot_name.c_str(), "Aliengo") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::Aliengo;

        // UNITREE_LEGGED_SDK::InitEnvironment();
        UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
        mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    #endif
    
}