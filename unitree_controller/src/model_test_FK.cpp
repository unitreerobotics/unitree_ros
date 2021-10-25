/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include "a1body.h"
#include <rosbag/bag.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <iit/rbd/utils.h>
#include <iit/robcogen/utils.h>
#include <iit/robcogen/jacs.h>

#include <UnitreeA1/rcg/transforms.h>
#include <UnitreeA1/rcg/jacobians.h>
#include <UnitreeA1/rcg/traits.h>
#include <UnitreeA1/rcg/declarations.h>

#include <Eigen/Dense>
#include <Eigen/QR> 
#include <cppad/cppad.hpp>
#include <cppad/core/value.hpp>
#include <cppad/example/cppad_eigen.hpp>


using namespace std;
using namespace a1_model;

using namespace iit;
using namespace UnitreeA1::rcg;

using namespace Eigen;


bool start_up = true;

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

class multiThread
{
public:
    multiThread(string rname){
        robot_name = rname;
        imu_sub = nm.subscribe("/trunk_imu", 1, &multiThread::imuCallback, this);
        footForce_sub[0] = nm.subscribe("/visual/FR_foot_contact/the_force", 1, &multiThread::FRfootCallback, this);
        footForce_sub[1] = nm.subscribe("/visual/FL_foot_contact/the_force", 1, &multiThread::FLfootCallback, this);
        footForce_sub[2] = nm.subscribe("/visual/RR_foot_contact/the_force", 1, &multiThread::RRfootCallback, this);
        footForce_sub[3] = nm.subscribe("/visual/RL_foot_contact/the_force", 1, &multiThread::RLfootCallback, this);
        servo_sub[0] = nm.subscribe("/" + robot_name + "_gazebo/FR_hip_controller/state", 1, &multiThread::FRhipCallback, this);
        servo_sub[1] = nm.subscribe("/" + robot_name + "_gazebo/FR_thigh_controller/state", 1, &multiThread::FRthighCallback, this);
        servo_sub[2] = nm.subscribe("/" + robot_name + "_gazebo/FR_calf_controller/state", 1, &multiThread::FRcalfCallback, this);
        servo_sub[3] = nm.subscribe("/" + robot_name + "_gazebo/FL_hip_controller/state", 1, &multiThread::FLhipCallback, this);
        servo_sub[4] = nm.subscribe("/" + robot_name + "_gazebo/FL_thigh_controller/state", 1, &multiThread::FLthighCallback, this);
        servo_sub[5] = nm.subscribe("/" + robot_name + "_gazebo/FL_calf_controller/state", 1, &multiThread::FLcalfCallback, this);
        servo_sub[6] = nm.subscribe("/" + robot_name + "_gazebo/RR_hip_controller/state", 1, &multiThread::RRhipCallback, this);
        servo_sub[7] = nm.subscribe("/" + robot_name + "_gazebo/RR_thigh_controller/state", 1, &multiThread::RRthighCallback, this);
        servo_sub[8] = nm.subscribe("/" + robot_name + "_gazebo/RR_calf_controller/state", 1, &multiThread::RRcalfCallback, this);
        servo_sub[9] = nm.subscribe("/" + robot_name + "_gazebo/RL_hip_controller/state", 1, &multiThread::RLhipCallback, this);
        servo_sub[10] = nm.subscribe("/" + robot_name + "_gazebo/RL_thigh_controller/state", 1, &multiThread::RLthighCallback, this);
        servo_sub[11] = nm.subscribe("/" + robot_name + "_gazebo/RL_calf_controller/state", 1, &multiThread::RLcalfCallback, this);
    }

    void imuCallback(const sensor_msgs::Imu & msg)
    { 
        lowState.imu.quaternion[0] = msg.orientation.w;
        lowState.imu.quaternion[1] = msg.orientation.x;
        lowState.imu.quaternion[2] = msg.orientation.y;
        lowState.imu.quaternion[3] = msg.orientation.z;

        lowState.imu.gyroscope[0] = msg.angular_velocity.x;
        lowState.imu.gyroscope[1] = msg.angular_velocity.y;
        lowState.imu.gyroscope[2] = msg.angular_velocity.z;
        
        lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
        lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
        lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
        
    }

    void FRhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[0].mode = msg.mode;
        lowState.motorState[0].q = msg.q;
        lowState.motorState[0].dq = msg.dq;
        lowState.motorState[0].tauEst = msg.tauEst;
    }

    void FRthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[1].mode = msg.mode;
        lowState.motorState[1].q = msg.q;
        lowState.motorState[1].dq = msg.dq;
        lowState.motorState[1].tauEst = msg.tauEst;
    }

    void FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[2].mode = msg.mode;
        lowState.motorState[2].q = msg.q;
        lowState.motorState[2].dq = msg.dq;
        lowState.motorState[2].tauEst = msg.tauEst;
    }

    void FLhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[3].mode = msg.mode;
        lowState.motorState[3].q = msg.q;
        lowState.motorState[3].dq = msg.dq;
        lowState.motorState[3].tauEst = msg.tauEst;
    }

    void FLthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[4].mode = msg.mode;
        lowState.motorState[4].q = msg.q;
        lowState.motorState[4].dq = msg.dq;
        lowState.motorState[4].tauEst = msg.tauEst;
    }

    void FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[5].mode = msg.mode;
        lowState.motorState[5].q = msg.q;
        lowState.motorState[5].dq = msg.dq;
        lowState.motorState[5].tauEst = msg.tauEst;
    }

    void RRhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[6].mode = msg.mode;
        lowState.motorState[6].q = msg.q;
        lowState.motorState[6].dq = msg.dq;
        lowState.motorState[6].tauEst = msg.tauEst;
    }

    void RRthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[7].mode = msg.mode;
        lowState.motorState[7].q = msg.q;
        lowState.motorState[7].dq = msg.dq;
        lowState.motorState[7].tauEst = msg.tauEst;
    }

    void RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[8].mode = msg.mode;
        lowState.motorState[8].q = msg.q;
        lowState.motorState[8].dq = msg.dq;
        lowState.motorState[8].tauEst = msg.tauEst;
    }

    void RLhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[9].mode = msg.mode;
        lowState.motorState[9].q = msg.q;
        lowState.motorState[9].dq = msg.dq;
        lowState.motorState[9].tauEst = msg.tauEst;
    }

    void RLthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[10].mode = msg.mode;
        lowState.motorState[10].q = msg.q;
        lowState.motorState[10].dq = msg.dq;
        lowState.motorState[10].tauEst = msg.tauEst;
    }

    void RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[11].mode = msg.mode;
        lowState.motorState[11].q = msg.q;
        lowState.motorState[11].dq = msg.dq;
        lowState.motorState[11].tauEst = msg.tauEst;
    }

    void FRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[0].x = msg.wrench.force.x;
        lowState.eeForce[0].y = msg.wrench.force.y;
        lowState.eeForce[0].z = msg.wrench.force.z;
        lowState.footForce[0] = msg.wrench.force.z;
    }

    void FLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[1].x = msg.wrench.force.x;
        lowState.eeForce[1].y = msg.wrench.force.y;
        lowState.eeForce[1].z = msg.wrench.force.z;
        lowState.footForce[1] = msg.wrench.force.z;
    }

    void RRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[2].x = msg.wrench.force.x;
        lowState.eeForce[2].y = msg.wrench.force.y;
        lowState.eeForce[2].z = msg.wrench.force.z;
        lowState.footForce[2] = msg.wrench.force.z;
    }

    void RLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[3].x = msg.wrench.force.x;
        lowState.eeForce[3].y = msg.wrench.force.y;
        lowState.eeForce[3].z = msg.wrench.force.z;
        lowState.footForce[3] = msg.wrench.force.z;
    }

private:
    ros::NodeHandle nm;
    ros::Subscriber servo_sub[12], footForce_sub[4], imu_sub;
    string robot_name;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unitree_gazebo_position_exe");

    string robot_name;
    ros::param::get("/robot_name", robot_name);
    cout << "robot_name: " << robot_name << endl;

    multiThread listen_publish_obj(robot_name);
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    usleep(300000); // must wait 300ms, to get first state

    ros::NodeHandle n;
    ros::Publisher lowState_pub; //for rviz visualization
    // ros::Rate loop_rate(1000);
    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<unitree_legged_msgs::LowState>("/" + robot_name + "_gazebo/lowState/state", 1);
    servo_pub[0] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_calf_controller/command", 1);

    // cout << "Robot is standing up..." << endl;
    // motion_init();
    // cout << "Robot is ready." << endl;

    paramInit();

    long motiontime = 0;
    int rate_count = 0;
    int sin_count = 0;
    float qInit[3]={0};
    float qDes[3]={0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};   
    bool initiated_flag = false;  // initiate need time
    int count = 0; 

    // ros::Duration period;
    ros::Time begin_time = ros::Time::now();
    ros::Duration elapsed_Dur_time;

    rosbag::Bag bag;
    bag.open("/home/bmtsz/test_out/position_joint.bag", rosbag::bagmode::Write);

    std_msgs::Float32 joint_pos;

    JointState qj;
    JointState q_measured;
    robcogen::utils::rand_jstate<Traits>(qj);

    UnitreeA1::rcg::Jacobians jacs;

    typedef CppAD::AD<Scalar> AD;

    while (ros::ok()){

        // Own code goes here

        if(initiated_flag == true){
            motiontime++;

            lowCmd.motorCmd[FR_0].tau = -0.65f;
            lowCmd.motorCmd[FL_0].tau = +0.65f;
            lowCmd.motorCmd[RR_0].tau = -0.65f;
            lowCmd.motorCmd[RL_0].tau = +0.65f;

            // printf("%d\n", motiontime);
            // printf("%d %f %f %f\n", FR_0, lowState.motorState[FR_0].q, lowState.motorState[FR_1].q, lowState.motorState[FR_2].q);
            // printf("period = %f\n", e_time.toSec());
            // printf("%f %f \n",  lowState.motorState[FR_0].mode, lowState.motorState[FR_1].mode);
            if( motiontime >= 0){
                // first, get record initial position
                // if( motiontime >= 100 && motiontime < 500){
                if( motiontime >= 0 && motiontime < 10){
                    qInit[0] = lowState.motorState[FR_0].q;
                    qInit[1] = lowState.motorState[FR_1].q;
                    qInit[2] = lowState.motorState[FR_2].q;
                }
                if( motiontime >= 10 && motiontime < 400){
                    // printf("%f %f %f\n", );
                    rate_count++;
                    double rate = rate_count/200.0;                       // needs count to 200
                    
                    qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
                    qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
                    qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);

                    lowCmd.motorCmd[FR_0].q = qDes[0];
                    lowCmd.motorCmd[FR_0].dq = 0;
                    // lowCmd.motorCmd[FR_0].Kp = Kp[0];
                    // lowCmd.motorCmd[FR_0].Kd = Kd[0];
                    lowCmd.motorCmd[FR_0].tau = -0.65f;

                    lowCmd.motorCmd[FR_1].q = qDes[1];
                    lowCmd.motorCmd[FR_1].dq = 0;
                    // lowCmd.motorCmd[FR_1].Kp = Kp[1];
                    // lowCmd.motorCmd[FR_1].Kd = Kd[1];
                    lowCmd.motorCmd[FR_1].tau = 0.0f;

                    lowCmd.motorCmd[FR_2].q =  qDes[2];
                    lowCmd.motorCmd[FR_2].dq = 0;
                    // lowCmd.motorCmd[FR_2].Kp = Kp[2];
                    // lowCmd.motorCmd[FR_2].Kd = Kd[2];
                    lowCmd.motorCmd[FR_2].tau = 0.0f;                    
                }
                double sin_joint1, v_des;
                //VectorXd<Scalar, 6> v_foot;
                //v_foot.setZero();
                //MatrixXd<Scalar, 6, 3> J_FR_foot;
                //MatrixXd<Scalar, 3, 6> pinv_J;
                VectorXd v_foot = VectorXd::Zero(6);
                MatrixXd J_FR_foot = MatrixXd::Zero(6, 3);
                MatrixXd pinv_J = MatrixXd::Zero(3, 6);
                // last, do sine wave
                if( motiontime >= 1700){
                    sin_count++;
                    sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0);
                    v_des = 2 * sin(1.8*M_PI*sin_count/1000.0);
                    qDes[0] = sin_mid_q[0];
                    qDes[1] = sin_mid_q[1];
                    v_foot[4] = v_des;
                    for(int i=0; i<12; i++) {
                        q_measured[i] = lowState.motorState[i].q;
                    }
                    jacs.fr_base_J_fr_FR_foot(q_measured);
                    for(int i=0; i<6; i++) {
                       for(int j=0; j<3; j++) {
                           J_FR_foot(i, j) = CppAD::Value(CppAD::Var2Par(jacs.fr_base_J_fr_FR_foot(i, j)));
                       }
                    }
                    
                    pinv_J = J_FR_foot.completeOrthogonalDecomposition().pseudoInverse();
                    Vector3d q_des = pinv_J * v_foot;
                    std::cout << "RobCoGen:\n" << std::endl;
                    std::cout << jacs.fr_base_J_fr_FR_foot << "\n" << std::endl;
                    std::cout << "J_FR_foot:\n" << std::endl;
                    std::cout << J_FR_foot << "\n" << std::endl;

                    lowCmd.motorCmd[FR_0].q = PosStopF;
                    lowCmd.motorCmd[FR_0].dq = q_des[0];
                    lowCmd.motorCmd[FR_0].Kp = 0;
                    // lowCmd.motorCmd[FR_0].Kd = Kd[0];
                    lowCmd.motorCmd[FR_0].tau = -0.65f;

                    lowCmd.motorCmd[FR_1].q = PosStopF;
                    lowCmd.motorCmd[FR_1].dq = q_des[1];
                    lowCmd.motorCmd[FR_1].Kp = 0;
                    // lowCmd.motorCmd[FR_1].Kd = Kd[1];
                    lowCmd.motorCmd[FR_1].tau = 0.0f;

                    lowCmd.motorCmd[FR_2].q =  PosStopF;
                    lowCmd.motorCmd[FR_2].dq = q_des[2];
                    lowCmd.motorCmd[FR_2].Kp = 0;
                    // lowCmd.motorCmd[FR_2].Kd = Kd[2];
                    lowCmd.motorCmd[FR_2].tau = 0.0f;                    

                    // qDes[2] = sin_mid_q[2] + sin_joint2;
                    // qDes[2] = sin_mid_q[2];
                }


            }

        }

        count++;
        if(count > 10){
            count = 10;
            initiated_flag = true;
        }        

        elapsed_Dur_time = ros::Time::now() - begin_time;
        ros::Time elapsed_time(elapsed_Dur_time.toSec());
        joint_pos.data = lowState.motorState[FR_2].q;
        // bag.write("/a1_gazebo/FR_calf_controller/state", elapsed_time, joint_pos);
        // bag.close();

        jacs.fr_base_J_fr_RR_foot(qj);
        //std::cout << "RobCoGen Jacobian:" << std::endl;
        //std::cout << jacs.fr_base_J_fr_RR_foot << std::endl;
        //std::cout << qj << std::endl;        

        // unitree default codes for send commands and publish states.
        lowState_pub.publish(lowState);
        sendServoCmd();

    }
    return 0;
}
