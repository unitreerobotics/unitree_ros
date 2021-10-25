#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR> 

#include "a1body.h"

using namespace a1_model;
using namespace Eigen;

namespace Central_Functions {

void UnitreeJoint2PinocchioJoint(const Ref<const VectorXd> q_A1, Ref<VectorXd> q_pinocchio);

void feetContactChecker(const unitree_legged_msgs::LowState& lowState_input, Ref<VectorXd> feetContact_state);

}