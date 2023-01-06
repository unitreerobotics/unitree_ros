#include "unitreeArm.h"

namespace unitreeArm {

IOROS::IOROS()
{
    // publisher init
    _servo_pub[0] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/Joint01_controller/command", 1);
    _servo_pub[1] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/Joint02_controller/command", 1);
    _servo_pub[2] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/Joint03_controller/command", 1);
    _servo_pub[3] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/Joint04_controller/command", 1);
    _servo_pub[4] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/Joint05_controller/command", 1);
    _servo_pub[5] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/Joint06_controller/command", 1);
    _servo_pub[6] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/gripper_controller/command", 1);

    // subscriber init
    _servo_sub[0] = _nm.subscribe("/z1_gazebo/Joint01_controller/state", 1, &IOROS::_joint01Callback, this);
    _servo_sub[1] = _nm.subscribe("/z1_gazebo/Joint02_controller/state", 1, &IOROS::_joint02Callback, this);
    _servo_sub[2] = _nm.subscribe("/z1_gazebo/Joint03_controller/state", 1, &IOROS::_joint03Callback, this);
    _servo_sub[3] = _nm.subscribe("/z1_gazebo/Joint04_controller/state", 1, &IOROS::_joint04Callback, this);
    _servo_sub[4] = _nm.subscribe("/z1_gazebo/Joint05_controller/state", 1, &IOROS::_joint05Callback, this);
    _servo_sub[5] = _nm.subscribe("/z1_gazebo/Joint06_controller/state", 1, &IOROS::_joint06Callback, this);
    _servo_sub[6] = _nm.subscribe("/z1_gazebo/gripper_controller/state", 1, &IOROS::_gripperCallback, this);

    // parameter init
    for(size_t i=0; i<7; ++i)
    {
        _joint_cmd[i].mode = 10;
        _joint_cmd[i].Kp   = 300.0;
        _joint_cmd[i].Kd   = 5;
        _joint_cmd[i].q    = 0;
        _joint_cmd[i].dq   = 0;
        _joint_cmd[i].tau  = 0;
    }
}

IOROS::~IOROS()
{

}

void IOROS::_joint01Callback(const unitree_legged_msgs::MotorState& msg)
{
    _joint_state[0].mode   = msg.mode;
    _joint_state[0].q      = msg.q;
    _joint_state[0].dq     = msg.dq;
    _joint_state[0].ddq    = msg.ddq;
    _joint_state[0].tauEst = msg.tauEst;
}

void IOROS::_joint02Callback(const unitree_legged_msgs::MotorState& msg)
{
    _joint_state[1].mode   = msg.mode;
    _joint_state[1].q      = msg.q;
    _joint_state[1].dq     = msg.dq;
    _joint_state[1].ddq    = msg.ddq;
    _joint_state[1].tauEst = msg.tauEst;
}

void IOROS::_joint03Callback(const unitree_legged_msgs::MotorState& msg)
{
    _joint_state[2].mode   = msg.mode;
    _joint_state[2].q      = msg.q;
    _joint_state[2].dq     = msg.dq;
    _joint_state[2].ddq    = msg.ddq;
    _joint_state[2].tauEst = msg.tauEst;
}

void IOROS::_joint04Callback(const unitree_legged_msgs::MotorState& msg)
{
    _joint_state[3].mode   = msg.mode;
    _joint_state[3].q      = msg.q;
    _joint_state[3].dq     = msg.dq;
    _joint_state[3].ddq    = msg.ddq;
    _joint_state[3].tauEst = msg.tauEst;
}

void IOROS::_joint05Callback(const unitree_legged_msgs::MotorState& msg)
{
    _joint_state[4].mode   = msg.mode;
    _joint_state[4].q      = msg.q;
    _joint_state[4].dq     = msg.dq;
    _joint_state[4].ddq    = msg.ddq;
    _joint_state[4].tauEst = msg.tauEst;
}

void IOROS::_joint06Callback(const unitree_legged_msgs::MotorState& msg)
{
    _joint_state[5].mode   = msg.mode;
    _joint_state[5].q      = msg.q;
    _joint_state[5].dq     = msg.dq;
    _joint_state[5].ddq    = msg.ddq;
    _joint_state[5].tauEst = msg.tauEst;
}

void IOROS::_gripperCallback(const unitree_legged_msgs::MotorState& msg)
{
    _joint_state[6].mode   = msg.mode;
    _joint_state[6].q      = msg.q;
    _joint_state[6].dq     = msg.dq;
    _joint_state[6].ddq    = msg.ddq;
    _joint_state[6].tauEst = msg.tauEst;
}

void IOROS::sendCmd(double* targetPos, double duration)
{
    double pos[7], lastPos[7], percent;
    for(size_t i=0; i<7; ++i) lastPos[i] = _joint_state[i].q;
    for(int i=1; i<=duration; i++)
    {
        if(!ros::ok()) break;
        percent = (double)i/duration;
        for(size_t j=0; j<7; ++j)
        {
            _joint_cmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent;
            _servo_pub[j].publish(_joint_cmd[j]);
        }
        ros::spinOnce();
        usleep(1000);
    }
}

}