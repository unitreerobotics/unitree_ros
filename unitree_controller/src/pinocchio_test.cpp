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
#include <iostream>
#include "functions.h"

// #include <iit/rbd/utils.h>
// #include <iit/robcogen/utils.h>
// #include <iit/robcogen/jacs.h>

// #include <UnitreeA1/rcg/transforms.h>
// #include <UnitreeA1/rcg/jacobians.h>
// #include <UnitreeA1/rcg/traits.h>
// #include <UnitreeA1/rcg/declarations.h>

#include <Eigen/Dense>
#include <Eigen/QR> 
#include <cppad/cppad.hpp>
#include <cppad/core/value.hpp>
#include <cppad/example/cppad_eigen.hpp>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"

#include <fstream>
#include <cstdlib>
#include <boost/algorithm/string.hpp>
#include "InEKF.h"

#include <chrono>

using namespace std;
using namespace a1_model;

// using namespace iit;
// using namespace UnitreeA1::rcg;

using namespace Eigen;

using namespace pinocchio;

#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "/home/bmtsz/catkin_ws/src/unitree_ros/robots/a1_description/urdf"
#endif

using namespace Central_Functions;

#define DT_MIN 1e-6
#define DT_MAX 1
using namespace inekf;


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
    ros::init(argc, argv, "RobCoGen_VS_Pinocchio");

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

    // JointState qj;
    // robcogen::utils::rand_jstate<Traits>(qj);

    // UnitreeA1::rcg::Jacobians jacs;

    // typedef CppAD::AD<Scalar> AD;

    // You should change here to set up your own URDF file or just pass it as an argument of this example.
    const std::string urdf_filename = (argc<=1) ? PINOCCHIO_MODEL_DIR + std::string("/a1.urdf") : argv[1];

    // Load the urdf model
    Model model;
    pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(), model);
    // pinocchio::urdf::buildModel(std::string("/home/bmtsz/catkin_ws/src/unitree_ros/robots/a1_description/urdf/a1.urdf"), pinocchio::JointModelFreeFlyer(), model);
    std::cout << "model name: " << model.name << std::endl;

    // Create data required by the algorithms
    Data data(model);

    // Eigen::VectorXd qj_pin = randomConfiguration(model);
    VectorXd qj_pin = VectorXd::Zero(model.nq);
    qj_pin[6] = 1;
    VectorXd q_measured_A1 = VectorXd::Zero(12);
    std::cout << "q: " << qj_pin.transpose() << std::endl;

    printf("body number: %d\n", model.nbodies);
    printf("joint number: %d\n", model.njoints);
    printf("number of general coordinate: %d\n", model.nq);
    printf("Dimension of control: %d\n", model.nv);

    // Perform the forward kinematics over the kinematic tree
    // forwardKinematics(model,data,qj_pin);   

    for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
    {
    std::cout << std::setw(24) << std::left
            << model.names[joint_id] << ": "
            << std::fixed << std::setprecision(2)
            << data.oMi[joint_id].translation().transpose()
            << std::endl; 
    }

    int idx_FL_foot = model.getFrameId("FL_foot");
    int idx_FR_foot = model.getFrameId("FR_foot");
    int idx_RL_foot = model.getFrameId("RL_foot");
    int idx_RR_foot = model.getFrameId("RR_foot");    
    int idx_base = model.getFrameId("base");

    VectorXd v_foot = VectorXd::Zero(6);
    MatrixXd I_J_FL_foot = MatrixXd::Zero(6, model.nv);
    MatrixXd I_J_FR_foot = MatrixXd::Zero(6, model.nv);
    MatrixXd I_J_RL_foot = MatrixXd::Zero(6, model.nv);
    MatrixXd I_J_RR_foot = MatrixXd::Zero(6, model.nv);
    MatrixXd I_J_c = MatrixXd::Zero(12, model.nv);
    MatrixXd I_J_Base = MatrixXd::Zero(6, model.nv);
    MatrixXd pinv_J = MatrixXd::Zero(model.nv, 6);
    Vector3d F_support = Vector3d::Zero(3);
    VectorXd tau_support = VectorXd::Zero(12);
    VectorXd feetContact_state = VectorXd::Zero(8);

    // motion_init();

    //  ---- Initialize invariant extended Kalman filter ----- //
    RobotState initial_state; 

    // Initialize state mean
    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, bg0, ba0;
    R0 << 1, 0, 0, // initial orientation
          0, 1, 0, // IMU frame is rotated 90deg about the x-axis
          0, 0, 1;
    v0 << 0,0,0; // initial velocity
    p0 << 0,0,0; // initial position
    bg0 << 0,0,0; // initial gyroscope bias
    ba0 << 0,0,0; // initial accelerometer bias
    initial_state.setRotation(R0);
    initial_state.setVelocity(v0);
    initial_state.setPosition(p0);
    initial_state.setGyroscopeBias(bg0);
    initial_state.setAccelerometerBias(ba0);

    // Initialize state covariance
    NoiseParams noise_params;
    noise_params.setGyroscopeNoise(0.01);
    noise_params.setAccelerometerNoise(0.1);
    noise_params.setGyroscopeBiasNoise(0.00001);
    noise_params.setAccelerometerBiasNoise(0.0001);
    noise_params.setContactNoise(0.01);
    noise_params.setJointNoise(0.01);

    // Initialize filter
    InEKF filter(initial_state, noise_params);
    cout << "Noise parameters are initialized to: \n";
    cout << filter.getNoiseParams() << endl;
    cout << "Robot's state is initialized to: \n";
    cout << filter.getState() << endl;

    Eigen::Matrix<double,6,1> imu_measurement = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Matrix<double,6,1> imu_measurement_prev = Eigen::Matrix<double,6,1>::Zero();
    auto t = std::chrono::high_resolution_clock::now();
    auto t_prev = std::chrono::high_resolution_clock::now();

    // Set the precision of printing the time
    std::cout << std::fixed << std::setprecision(10) << std::left;

    while (ros::ok()){

        // Own code goes here
        if (initiated_flag == false){
            imu_measurement_prev << lowState.imu.gyroscope[0], lowState.imu.gyroscope[1], lowState.imu.gyroscope[2],
                                    lowState.imu.accelerometer[0], lowState.imu.accelerometer[1], lowState.imu.accelerometer[2];

            t_prev = std::chrono::high_resolution_clock::now(); 

        }
        else{
            for(int i=0; i<12; i++) {
                q_measured_A1[i] = lowState.motorState[i].q;
            }

            UnitreeJoint2PinocchioJoint(q_measured_A1, qj_pin);
            framesForwardKinematics(model,data,qj_pin);
            data.oMf[idx_FL_foot].translation();
            

            // ---- State estimation using Invariant Extended Kalman Filter (InEKF) ---- //
            // Received IMU Data, propagating state
            imu_measurement << lowState.imu.gyroscope[0], lowState.imu.gyroscope[1], lowState.imu.gyroscope[2],
                               lowState.imu.accelerometer[0], lowState.imu.accelerometer[1], lowState.imu.accelerometer[2];            
            t = std::chrono::high_resolution_clock::now();            
            double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(t - t_prev).count()/1.0e9;
            cout << "chrono time: " << dt << "s\n";
            if (dt > DT_MIN && dt < DT_MAX) {
                filter.Propagate(imu_measurement_prev, dt);
            }            
         
            // Received CONTACT Data, setting filter's contact state
            feetContact_state.setZero();
            cout << "FL foot force: " << lowState.eeForce[1] << "s\n";
            feetContactChecker(lowState, feetContact_state);
            cout << "contact state: " << feetContact_state << "\n";
            vector<pair<int,bool> > contacts;
            int id;
            bool indicator; 
            for (int i=0; i<feetContact_state.size(); i+=2) {
                id = int(feetContact_state[i]);
                indicator = bool(feetContact_state[i+1]);
                contacts.push_back(pair<int,bool> (id, indicator));
            }       
            filter.setContacts(contacts);  

            // Received KINEMATIC observation, correcting state
            int id;
            Vector3d position = Vector3d::Zero(3);
            Matrix<double,3,3> covariance;
            vectorKinematics measured_kinematics;   
            // Read in kinematic data
            for (int i=2; i<measurement.size(); i+=44) {
                id = stoi98(measurement[i]); 
                q = Eigen::Quaternion<double> (stod98(measurement[i+1]),stod98(measurement[i+2]),stod98(measurement[i+3]),stod98(measurement[i+4]));
                q.normalize();
                p << stod98(measurement[i+5]),stod98(measurement[i+6]),stod98(measurement[i+7]);
                pose.block<3,3>(0,0) = q.toRotationMatrix();
                pose.block<3,1>(0,3) = p;
                for (int j=0; j<6; ++j) {
                    for (int k=0; k<6; ++k) {
                        covariance(j,k) = stod98(measurement[i+8 + j*6+k]);
                    }
                }
                Kinematics frame(id, pose, covariance);
                measured_kinematics.push_back(frame);
            }
            // Correct state using kinematic measurements
            filter.CorrectKinematics(measured_kinematics);                                

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
                // if( motiontime >= 0 && motiontime < 10){
                //     qInit[0] = lowState.motorState[FR_0].q;
                //     qInit[1] = lowState.motorState[FR_1].q;
                //     qInit[2] = lowState.motorState[FR_2].q;
                // }
                // if( motiontime >= 10 && motiontime < 400){
                //     // printf("%f %f %f\n", );
                //     rate_count++;
                //     double rate = rate_count/200.0;                       // needs count to 200
                    
                //     qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
                //     qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
                //     qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);

                //     lowCmd.motorCmd[FR_0].q = qDes[0];
                //     lowCmd.motorCmd[FR_0].dq = 0;
                //     // lowCmd.motorCmd[FR_0].Kp = Kp[0];
                //     // lowCmd.motorCmd[FR_0].Kd = Kd[0];
                //     lowCmd.motorCmd[FR_0].tau = -0.65f;

                //     lowCmd.motorCmd[FR_1].q = qDes[1];
                //     lowCmd.motorCmd[FR_1].dq = 0;
                //     // lowCmd.motorCmd[FR_1].Kp = Kp[1];
                //     // lowCmd.motorCmd[FR_1].Kd = Kd[1];
                //     lowCmd.motorCmd[FR_1].tau = 0.0f;

                //     lowCmd.motorCmd[FR_2].q =  qDes[2];
                //     lowCmd.motorCmd[FR_2].dq = 0;
                //     // lowCmd.motorCmd[FR_2].Kp = Kp[2];
                //     // lowCmd.motorCmd[FR_2].Kd = Kd[2];
                //     lowCmd.motorCmd[FR_2].tau = 0.0f;                    
                // }
                double sin_joint1, v_des;
                //VectorXd<Scalar, 6> v_foot;
                //v_foot.setZero();
                //MatrixXd<Scalar, 6, 3> I_J_FR_foot;
                //MatrixXd<Scalar, 3, 6> pinv_J;
                // MatrixXd pinv_I_J_Base = MatrixXd::Zero(model.nv, 6);
                // last, do sine wave
                if( motiontime >= 1700){
                    sin_count++;
                    sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0);
                    v_des = 0.04 * sin(1.8*M_PI*sin_count/1000.0);
                    qDes[0] = sin_mid_q[0];
                    qDes[1] = sin_mid_q[1];
                    v_foot[0] = v_des;

                    I_J_FL_foot.setZero();
                    I_J_FR_foot.setZero();
                    I_J_RL_foot.setZero();
                    I_J_RR_foot.setZero();
                    I_J_c.setZero();
                    I_J_Base.setZero();
                    F_support.setZero();
                    tau_support.setZero();
                    computeFrameJacobian(model, data, qj_pin, idx_FL_foot, LOCAL_WORLD_ALIGNED, I_J_FL_foot);
                    computeFrameJacobian(model, data, qj_pin, idx_FR_foot, LOCAL_WORLD_ALIGNED, I_J_FR_foot);
                    computeFrameJacobian(model, data, qj_pin, idx_RL_foot, LOCAL_WORLD_ALIGNED, I_J_RL_foot);
                    computeFrameJacobian(model, data, qj_pin, idx_RR_foot, LOCAL_WORLD_ALIGNED, I_J_RR_foot);
                    computeFrameJacobian(model, data, qj_pin, idx_base, LOCAL_WORLD_ALIGNED, I_J_Base);
                    computeGeneralizedGravity(model, data, qj_pin);
                    jacobianCenterOfMass(model, data, qj_pin, true);

                    I_J_c << I_J_FL_foot.topRows(3),
                             I_J_FR_foot.topRows(3),
                             I_J_RL_foot.topRows(3),
                             I_J_RR_foot.topRows(3);

                    F_support << 0, 0, -data.g[2];
                    // tau_support = data.Jcom.transpose() * F_support;
                    tau_support << I_J_FL_foot.block(0, 6, 3, 3).transpose() * (F_support / 4),
                                   I_J_FR_foot.block(0, 9, 3, 3).transpose() * (F_support / 4),
                                   I_J_RL_foot.block(0, 12, 3, 3).transpose() * (F_support / 4),
                                   I_J_RR_foot.block(0, 15, 3, 3).transpose() * (F_support / 4);
                    // jacs.fr_base_J_fr_FR_foot(q_measured_A1);
                    // for(int i=0; i<6; i++) {
                    //    for(int j=0; j<3; j++) {
                    //        I_J_FR_foot(i, j) = CppAD::Value(CppAD::Var2Par(jacs.fr_base_J_fr_FR_foot(i, j)));
                    //    }
                    // }

                    MatrixXd pinv_I_J_c = I_J_c.completeOrthogonalDecomposition().pseudoInverse();
                    MatrixXd N_I_J_c = MatrixXd::Identity(model.nv, model.nv) - pinv_I_J_c * I_J_c;
                    MatrixXd pinv_I_J_Base_x_N_I_J_c = (I_J_Base * N_I_J_c).completeOrthogonalDecomposition().pseudoInverse();
                    VectorXd q_des = N_I_J_c * pinv_I_J_Base_x_N_I_J_c * v_foot;                    

                    // MatrixXd pinv_I_J_Base = I_J_Base.completeOrthogonalDecomposition().pseudoInverse();
                    // MatrixXd N_I_J_Base = MatrixXd::Identity(model.nv, model.nv) - pinv_I_J_Base * I_J_Base;
                    // MatrixXd pinv_I_J_FR_foot_x_N_I_J_Base = (I_J_FR_foot * N_I_J_Base).completeOrthogonalDecomposition().pseudoInverse();
                    // VectorXd q_des = N_I_J_Base * pinv_I_J_FR_foot_x_N_I_J_Base * v_foot;
                    // pinv_J = I_J_FR_foot.completeOrthogonalDecomposition().pseudoInverse();
                    // VectorXd q_des = pinv_J * v_foot;
                    std::cout << "q_des:\n" << q_des.transpose() << std::endl;
                    // std::cout << "RobCoGen:\n" << std::endl;
                    // std::cout << jacs.fr_base_J_fr_FR_foot << "\n" << std::endl;
                    // std::cout << "I_J_FR_foot:\n" << std::endl;
                    // std::cout << I_J_FR_foot << "\n" << std::endl;



                    lowCmd.motorCmd[FL_0].q = PosStopF;
                    lowCmd.motorCmd[FL_0].dq = q_des[6];
                    lowCmd.motorCmd[FL_0].Kp = 0;
                    // lowCmd.motorCmd[FL_0].Kd = Kd[0];
                    lowCmd.motorCmd[FL_0].tau = tau_support[0];

                    lowCmd.motorCmd[FL_1].q = PosStopF;
                    lowCmd.motorCmd[FL_1].dq = q_des[7];
                    lowCmd.motorCmd[FL_1].Kp = 0;
                    // lowCmd.motorCmd[FL_1].Kd = Kd[1];
                    lowCmd.motorCmd[FL_1].tau = tau_support[1];

                    lowCmd.motorCmd[FL_2].q =  PosStopF;
                    lowCmd.motorCmd[FL_2].dq = q_des[8];
                    lowCmd.motorCmd[FL_2].Kp = 0;
                    // lowCmd.motorCmd[FL_2].Kd = Kd[2];
                    lowCmd.motorCmd[FL_2].tau = tau_support[2];  
                    

                    lowCmd.motorCmd[FR_0].q = PosStopF;
                    lowCmd.motorCmd[FR_0].dq = q_des[9];
                    lowCmd.motorCmd[FR_0].Kp = 0;
                    // lowCmd.motorCmd[FR_0].Kd = Kd[0];
                    lowCmd.motorCmd[FR_0].tau = tau_support[3];

                    lowCmd.motorCmd[FR_1].q = PosStopF;
                    lowCmd.motorCmd[FR_1].dq = q_des[10];
                    lowCmd.motorCmd[FR_1].Kp = 0;
                    // lowCmd.motorCmd[FR_1].Kd = Kd[1];
                    lowCmd.motorCmd[FR_1].tau = tau_support[4];

                    lowCmd.motorCmd[FR_2].q =  PosStopF;
                    lowCmd.motorCmd[FR_2].dq = q_des[11];
                    lowCmd.motorCmd[FR_2].Kp = 0;
                    // lowCmd.motorCmd[FR_2].Kd = Kd[2];
                    lowCmd.motorCmd[FR_2].tau = tau_support[5];


                    lowCmd.motorCmd[RL_0].q = PosStopF;
                    lowCmd.motorCmd[RL_0].dq = q_des[12];
                    lowCmd.motorCmd[RL_0].Kp = 0;
                    // lowCmd.motorCmd[RL_0].Kd = Kd[0];
                    lowCmd.motorCmd[RL_0].tau = tau_support[6];

                    lowCmd.motorCmd[RL_1].q = PosStopF;
                    lowCmd.motorCmd[RL_1].dq = q_des[13];
                    lowCmd.motorCmd[RL_1].Kp = 0;
                    // lowCmd.motorCmd[RL_1].Kd = Kd[1];
                    lowCmd.motorCmd[RL_1].tau = tau_support[7];

                    lowCmd.motorCmd[RL_2].q =  PosStopF;
                    lowCmd.motorCmd[RL_2].dq = q_des[14];
                    lowCmd.motorCmd[RL_2].Kp = 0;
                    // lowCmd.motorCmd[RL_2].Kd = Kd[2];
                    lowCmd.motorCmd[RL_2].tau = tau_support[8];


                    lowCmd.motorCmd[RR_0].q = PosStopF;
                    lowCmd.motorCmd[RR_0].dq = q_des[15];
                    lowCmd.motorCmd[RR_0].Kp = 0;
                    // lowCmd.motorCmd[RR_0].Kd = Kd[0];
                    lowCmd.motorCmd[RR_0].tau = tau_support[9];

                    lowCmd.motorCmd[RR_1].q = PosStopF;
                    lowCmd.motorCmd[RR_1].dq = q_des[16];
                    lowCmd.motorCmd[RR_1].Kp = 0;
                    // lowCmd.motorCmd[RR_1].Kd = Kd[1];
                    lowCmd.motorCmd[RR_1].tau = tau_support[10];

                    lowCmd.motorCmd[RR_2].q =  PosStopF;
                    lowCmd.motorCmd[RR_2].dq = q_des[17];
                    lowCmd.motorCmd[RR_2].Kp = 0;
                    // lowCmd.motorCmd[RR_2].Kd = Kd[2];
                    lowCmd.motorCmd[RR_2].tau = tau_support[11];


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
        // joint_pos.data = lowState.motorState[FR_2].q;
        // bag.write("/a1_gazebo/FR_calf_controller/state", elapsed_time, joint_pos);
        // bag.close();

        // jacs.fr_base_J_fr_RR_foot(qj);

        //Eigen::VectorXd qj_pin(12);
        //qj_pin << CppAD::Value(CppAD::Var2Par(qj(3))), CppAD::Value(CppAD::Var2Par(qj(4))), CppAD::Value(CppAD::Var2Par(qj(5))), 
        //          CppAD::Value(CppAD::Var2Par(qj(0))), CppAD::Value(CppAD::Var2Par(qj(1))), CppAD::Value(CppAD::Var2Par(qj(2))),
        //          CppAD::Value(CppAD::Var2Par(qj(9))), CppAD::Value(CppAD::Var2Par(qj(10))), CppAD::Value(CppAD::Var2Par(qj(11))), 
        //          CppAD::Value(CppAD::Var2Par(qj(6))), CppAD::Value(CppAD::Var2Par(qj(7))), CppAD::Value(CppAD::Var2Par(qj(8)));
        //Eigen::VectorXd qj_pin = randomConfiguration(model);

        //Eigen::MatrixXd J_pin = MatrixXd::Zero(6, 12);
        //computeFrameJacobian(model, data, qj_pin, 44, WORLD, J_pin);

        //for(int i=0; i<model.nv; i++) {
        //    q_ = CppAD::Value(CppAD::Var2Par(jacs.fr_base_J_fr_FR_foot(i, j)));
        //}
        // std::cout << "RobCoGen Jacobian:\n" << std::endl;
        // std::cout << jacs.fr_base_J_fr_RR_foot << "\n" << std::endl;

        // Print out the placement of each joint of the kinematic tree
        // for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
        //     std::cout << std::setw(24) << std::left
        //             << model.names[joint_id] << ": "
        //             << std::fixed << std::setprecision(2)
        //             << data.oMi[joint_id].translation().transpose()
        //             << std::endl;
        

        //std::cout << "Pinocchio Jacobian:\n" << std::endl;
        //std::cout << J_pin << "\n" << std::endl;        
        //std::cout << qj << std::endl;        

        // unitree default codes for send commands and publish states.
        // lowState_pub.publish(lowState);

        // Store previous timestamp
        imu_measurement_prev = imu_measurement;
        t_prev = t;   

        // Send command to motors
        sendServoCmd();
        // loop_rate.sleep();
    }
    return 0;
}
