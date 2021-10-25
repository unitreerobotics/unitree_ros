#include "functions.h"

namespace Central_Functions {

void UnitreeJoint2PinocchioJoint(const Ref<const VectorXd> q_A1, Ref<VectorXd> q_pinocchio)
{
    q_pinocchio[10] = q_A1[FR_0];
    q_pinocchio[11] = q_A1[FR_1];
    q_pinocchio[12] = q_A1[FR_2];

    q_pinocchio[7] = q_A1[FL_0];
    q_pinocchio[8] = q_A1[FL_1];
    q_pinocchio[9] = q_A1[FL_2];

    q_pinocchio[16] = q_A1[RR_0];
    q_pinocchio[17] = q_A1[RR_1];
    q_pinocchio[18] = q_A1[RR_2];

    q_pinocchio[13] = q_A1[RL_0];
    q_pinocchio[14] = q_A1[RL_1];
    q_pinocchio[15] = q_A1[RL_2];

}

void feetContactChecker(const unitree_legged_msgs::LowState& lowState_input, Ref<VectorXd> feetContact_state)
{
    double FL_FootForce = sqrt (lowState_input.eeForce[1].x * lowState_input.eeForce[1].x 
                                + lowState_input.eeForce[1].y * lowState_input.eeForce[1].y 
                                + lowState_input.eeForce[1].z * lowState_input.eeForce[1].z);

    double FR_FootForce = sqrt (lowState_input.eeForce[0].x * lowState_input.eeForce[0].x 
                                + lowState_input.eeForce[0].y * lowState_input.eeForce[0].y 
                                + lowState_input.eeForce[0].z * lowState_input.eeForce[0].z);

    double RL_FootForce = sqrt (lowState_input.eeForce[3].x * lowState_input.eeForce[3].x 
                                + lowState_input.eeForce[3].y * lowState_input.eeForce[3].y 
                                + lowState_input.eeForce[3].z * lowState_input.eeForce[3].z);

    double RR_FootForce = sqrt (lowState_input.eeForce[2].x * lowState_input.eeForce[2].x 
                                + lowState_input.eeForce[2].y * lowState_input.eeForce[2].y 
                                + lowState_input.eeForce[2].z * lowState_input.eeForce[2].z);       

    double force_threshold = 1; // TODO: Need to determine later

    // Foot ID
    feetContact_state[0] = 0; // FL
    feetContact_state[2] = 1; // FR
    feetContact_state[4] = 2; // RL
    feetContact_state[6] = 3; // RR

    if (FL_FootForce > force_threshold){feetContact_state[1] = 1;}
    if (FR_FootForce > force_threshold){feetContact_state[3] = 1;}
    if (RL_FootForce > force_threshold){feetContact_state[5] = 1;}
    if (RR_FootForce > force_threshold){feetContact_state[7] = 1;}                    

}

}