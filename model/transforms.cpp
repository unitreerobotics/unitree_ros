#include "transforms.h"

using namespace UnitreeA1::rcg;

// Constructors

MotionTransforms::MotionTransforms()
 :     fr_base_X_fr_FL_calf(),
    fr_FL_calf_X_fr_base(),
    fr_base_X_fr_FL_hip(),
    fr_FL_hip_X_fr_base(),
    fr_base_X_fr_FL_thigh(),
    fr_FL_thigh_X_fr_base(),
    fr_base_X_fr_FR_calf(),
    fr_FR_calf_X_fr_base(),
    fr_base_X_fr_FR_hip(),
    fr_FR_hip_X_fr_base(),
    fr_base_X_fr_FR_thigh(),
    fr_FR_thigh_X_fr_base(),
    fr_base_X_fr_RL_calf(),
    fr_RL_calf_X_fr_base(),
    fr_base_X_fr_RL_hip(),
    fr_RL_hip_X_fr_base(),
    fr_base_X_fr_RL_thigh(),
    fr_RL_thigh_X_fr_base(),
    fr_base_X_fr_RR_calf(),
    fr_RR_calf_X_fr_base(),
    fr_base_X_fr_RR_hip(),
    fr_RR_hip_X_fr_base(),
    fr_base_X_fr_RR_thigh(),
    fr_RR_thigh_X_fr_base(),
    fr_base_X_fr_FL_calf_COM(),
    fr_FL_calf_COM_X_fr_base(),
    fr_base_X_fr_FL_foot(),
    fr_FL_foot_X_fr_base(),
    fr_base_X_fr_FL_hip_COM(),
    fr_FL_hip_COM_X_fr_base(),
    fr_base_X_fr_FL_thigh_COM(),
    fr_FL_thigh_COM_X_fr_base(),
    fr_base_X_fr_FL_thigh_shoulder(),
    fr_FL_thigh_shoulder_X_fr_base(),
    fr_base_X_fr_FR_calf_COM(),
    fr_FR_calf_COM_X_fr_base(),
    fr_base_X_fr_FR_foot(),
    fr_FR_foot_X_fr_base(),
    fr_base_X_fr_FR_hip_COM(),
    fr_FR_hip_COM_X_fr_base(),
    fr_base_X_fr_FR_thigh_COM(),
    fr_FR_thigh_COM_X_fr_base(),
    fr_base_X_fr_FR_thigh_shoulder(),
    fr_FR_thigh_shoulder_X_fr_base(),
    fr_base_X_fr_RL_calf_COM(),
    fr_RL_calf_COM_X_fr_base(),
    fr_base_X_fr_RL_foot(),
    fr_RL_foot_X_fr_base(),
    fr_base_X_fr_RL_hip_COM(),
    fr_RL_hip_COM_X_fr_base(),
    fr_base_X_fr_RL_thigh_COM(),
    fr_RL_thigh_COM_X_fr_base(),
    fr_base_X_fr_RL_thigh_shoulder(),
    fr_RL_thigh_shoulder_X_fr_base(),
    fr_base_X_fr_RR_calf_COM(),
    fr_RR_calf_COM_X_fr_base(),
    fr_base_X_fr_RR_foot(),
    fr_RR_foot_X_fr_base(),
    fr_base_X_fr_RR_hip_COM(),
    fr_RR_hip_COM_X_fr_base(),
    fr_base_X_fr_RR_thigh_COM(),
    fr_RR_thigh_COM_X_fr_base(),
    fr_base_X_fr_RR_thigh_shoulder(),
    fr_RR_thigh_shoulder_X_fr_base(),
    fr_base_X_fr_base_COM(),
    fr_base_COM_X_fr_base(),
    fr_base_X_fr_imu_link(),
    fr_imu_link_X_fr_base(),
    fr_base_X_fr_trunk(),
    fr_trunk_X_fr_base(),
    fr_base_X_fr_FL_hip_joint(),
    fr_base_X_fr_FL_thigh_joint(),
    fr_base_X_fr_FL_calf_joint(),
    fr_base_X_fr_FR_hip_joint(),
    fr_base_X_fr_FR_thigh_joint(),
    fr_base_X_fr_FR_calf_joint(),
    fr_base_X_fr_RL_hip_joint(),
    fr_base_X_fr_RL_thigh_joint(),
    fr_base_X_fr_RL_calf_joint(),
    fr_base_X_fr_RR_hip_joint(),
    fr_base_X_fr_RR_thigh_joint(),
    fr_base_X_fr_RR_calf_joint(),
    fr_FR_thigh_X_fr_FR_hip(),
    fr_FR_hip_X_fr_FR_thigh(),
    fr_FR_calf_X_fr_FR_thigh(),
    fr_FR_thigh_X_fr_FR_calf(),
    fr_FL_thigh_X_fr_FL_hip(),
    fr_FL_hip_X_fr_FL_thigh(),
    fr_FL_calf_X_fr_FL_thigh(),
    fr_FL_thigh_X_fr_FL_calf(),
    fr_RR_thigh_X_fr_RR_hip(),
    fr_RR_hip_X_fr_RR_thigh(),
    fr_RR_calf_X_fr_RR_thigh(),
    fr_RR_thigh_X_fr_RR_calf(),
    fr_RL_thigh_X_fr_RL_hip(),
    fr_RL_hip_X_fr_RL_thigh(),
    fr_RL_calf_X_fr_RL_thigh(),
    fr_RL_thigh_X_fr_RL_calf()
{}
void MotionTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

ForceTransforms::ForceTransforms()
 :     fr_base_X_fr_FL_calf(),
    fr_FL_calf_X_fr_base(),
    fr_base_X_fr_FL_hip(),
    fr_FL_hip_X_fr_base(),
    fr_base_X_fr_FL_thigh(),
    fr_FL_thigh_X_fr_base(),
    fr_base_X_fr_FR_calf(),
    fr_FR_calf_X_fr_base(),
    fr_base_X_fr_FR_hip(),
    fr_FR_hip_X_fr_base(),
    fr_base_X_fr_FR_thigh(),
    fr_FR_thigh_X_fr_base(),
    fr_base_X_fr_RL_calf(),
    fr_RL_calf_X_fr_base(),
    fr_base_X_fr_RL_hip(),
    fr_RL_hip_X_fr_base(),
    fr_base_X_fr_RL_thigh(),
    fr_RL_thigh_X_fr_base(),
    fr_base_X_fr_RR_calf(),
    fr_RR_calf_X_fr_base(),
    fr_base_X_fr_RR_hip(),
    fr_RR_hip_X_fr_base(),
    fr_base_X_fr_RR_thigh(),
    fr_RR_thigh_X_fr_base(),
    fr_base_X_fr_FL_calf_COM(),
    fr_FL_calf_COM_X_fr_base(),
    fr_base_X_fr_FL_foot(),
    fr_FL_foot_X_fr_base(),
    fr_base_X_fr_FL_hip_COM(),
    fr_FL_hip_COM_X_fr_base(),
    fr_base_X_fr_FL_thigh_COM(),
    fr_FL_thigh_COM_X_fr_base(),
    fr_base_X_fr_FL_thigh_shoulder(),
    fr_FL_thigh_shoulder_X_fr_base(),
    fr_base_X_fr_FR_calf_COM(),
    fr_FR_calf_COM_X_fr_base(),
    fr_base_X_fr_FR_foot(),
    fr_FR_foot_X_fr_base(),
    fr_base_X_fr_FR_hip_COM(),
    fr_FR_hip_COM_X_fr_base(),
    fr_base_X_fr_FR_thigh_COM(),
    fr_FR_thigh_COM_X_fr_base(),
    fr_base_X_fr_FR_thigh_shoulder(),
    fr_FR_thigh_shoulder_X_fr_base(),
    fr_base_X_fr_RL_calf_COM(),
    fr_RL_calf_COM_X_fr_base(),
    fr_base_X_fr_RL_foot(),
    fr_RL_foot_X_fr_base(),
    fr_base_X_fr_RL_hip_COM(),
    fr_RL_hip_COM_X_fr_base(),
    fr_base_X_fr_RL_thigh_COM(),
    fr_RL_thigh_COM_X_fr_base(),
    fr_base_X_fr_RL_thigh_shoulder(),
    fr_RL_thigh_shoulder_X_fr_base(),
    fr_base_X_fr_RR_calf_COM(),
    fr_RR_calf_COM_X_fr_base(),
    fr_base_X_fr_RR_foot(),
    fr_RR_foot_X_fr_base(),
    fr_base_X_fr_RR_hip_COM(),
    fr_RR_hip_COM_X_fr_base(),
    fr_base_X_fr_RR_thigh_COM(),
    fr_RR_thigh_COM_X_fr_base(),
    fr_base_X_fr_RR_thigh_shoulder(),
    fr_RR_thigh_shoulder_X_fr_base(),
    fr_base_X_fr_base_COM(),
    fr_base_COM_X_fr_base(),
    fr_base_X_fr_imu_link(),
    fr_imu_link_X_fr_base(),
    fr_base_X_fr_trunk(),
    fr_trunk_X_fr_base(),
    fr_base_X_fr_FL_hip_joint(),
    fr_base_X_fr_FL_thigh_joint(),
    fr_base_X_fr_FL_calf_joint(),
    fr_base_X_fr_FR_hip_joint(),
    fr_base_X_fr_FR_thigh_joint(),
    fr_base_X_fr_FR_calf_joint(),
    fr_base_X_fr_RL_hip_joint(),
    fr_base_X_fr_RL_thigh_joint(),
    fr_base_X_fr_RL_calf_joint(),
    fr_base_X_fr_RR_hip_joint(),
    fr_base_X_fr_RR_thigh_joint(),
    fr_base_X_fr_RR_calf_joint(),
    fr_FR_thigh_X_fr_FR_hip(),
    fr_FR_hip_X_fr_FR_thigh(),
    fr_FR_calf_X_fr_FR_thigh(),
    fr_FR_thigh_X_fr_FR_calf(),
    fr_FL_thigh_X_fr_FL_hip(),
    fr_FL_hip_X_fr_FL_thigh(),
    fr_FL_calf_X_fr_FL_thigh(),
    fr_FL_thigh_X_fr_FL_calf(),
    fr_RR_thigh_X_fr_RR_hip(),
    fr_RR_hip_X_fr_RR_thigh(),
    fr_RR_calf_X_fr_RR_thigh(),
    fr_RR_thigh_X_fr_RR_calf(),
    fr_RL_thigh_X_fr_RL_hip(),
    fr_RL_hip_X_fr_RL_thigh(),
    fr_RL_calf_X_fr_RL_thigh(),
    fr_RL_thigh_X_fr_RL_calf()
{}
void ForceTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

HomogeneousTransforms::HomogeneousTransforms()
 :     fr_base_X_fr_FL_calf(),
    fr_FL_calf_X_fr_base(),
    fr_base_X_fr_FL_hip(),
    fr_FL_hip_X_fr_base(),
    fr_base_X_fr_FL_thigh(),
    fr_FL_thigh_X_fr_base(),
    fr_base_X_fr_FR_calf(),
    fr_FR_calf_X_fr_base(),
    fr_base_X_fr_FR_hip(),
    fr_FR_hip_X_fr_base(),
    fr_base_X_fr_FR_thigh(),
    fr_FR_thigh_X_fr_base(),
    fr_base_X_fr_RL_calf(),
    fr_RL_calf_X_fr_base(),
    fr_base_X_fr_RL_hip(),
    fr_RL_hip_X_fr_base(),
    fr_base_X_fr_RL_thigh(),
    fr_RL_thigh_X_fr_base(),
    fr_base_X_fr_RR_calf(),
    fr_RR_calf_X_fr_base(),
    fr_base_X_fr_RR_hip(),
    fr_RR_hip_X_fr_base(),
    fr_base_X_fr_RR_thigh(),
    fr_RR_thigh_X_fr_base(),
    fr_base_X_fr_FL_calf_COM(),
    fr_FL_calf_COM_X_fr_base(),
    fr_base_X_fr_FL_foot(),
    fr_FL_foot_X_fr_base(),
    fr_base_X_fr_FL_hip_COM(),
    fr_FL_hip_COM_X_fr_base(),
    fr_base_X_fr_FL_thigh_COM(),
    fr_FL_thigh_COM_X_fr_base(),
    fr_base_X_fr_FL_thigh_shoulder(),
    fr_FL_thigh_shoulder_X_fr_base(),
    fr_base_X_fr_FR_calf_COM(),
    fr_FR_calf_COM_X_fr_base(),
    fr_base_X_fr_FR_foot(),
    fr_FR_foot_X_fr_base(),
    fr_base_X_fr_FR_hip_COM(),
    fr_FR_hip_COM_X_fr_base(),
    fr_base_X_fr_FR_thigh_COM(),
    fr_FR_thigh_COM_X_fr_base(),
    fr_base_X_fr_FR_thigh_shoulder(),
    fr_FR_thigh_shoulder_X_fr_base(),
    fr_base_X_fr_RL_calf_COM(),
    fr_RL_calf_COM_X_fr_base(),
    fr_base_X_fr_RL_foot(),
    fr_RL_foot_X_fr_base(),
    fr_base_X_fr_RL_hip_COM(),
    fr_RL_hip_COM_X_fr_base(),
    fr_base_X_fr_RL_thigh_COM(),
    fr_RL_thigh_COM_X_fr_base(),
    fr_base_X_fr_RL_thigh_shoulder(),
    fr_RL_thigh_shoulder_X_fr_base(),
    fr_base_X_fr_RR_calf_COM(),
    fr_RR_calf_COM_X_fr_base(),
    fr_base_X_fr_RR_foot(),
    fr_RR_foot_X_fr_base(),
    fr_base_X_fr_RR_hip_COM(),
    fr_RR_hip_COM_X_fr_base(),
    fr_base_X_fr_RR_thigh_COM(),
    fr_RR_thigh_COM_X_fr_base(),
    fr_base_X_fr_RR_thigh_shoulder(),
    fr_RR_thigh_shoulder_X_fr_base(),
    fr_base_X_fr_base_COM(),
    fr_base_COM_X_fr_base(),
    fr_base_X_fr_imu_link(),
    fr_imu_link_X_fr_base(),
    fr_base_X_fr_trunk(),
    fr_trunk_X_fr_base(),
    fr_base_X_fr_FL_hip_joint(),
    fr_base_X_fr_FL_thigh_joint(),
    fr_base_X_fr_FL_calf_joint(),
    fr_base_X_fr_FR_hip_joint(),
    fr_base_X_fr_FR_thigh_joint(),
    fr_base_X_fr_FR_calf_joint(),
    fr_base_X_fr_RL_hip_joint(),
    fr_base_X_fr_RL_thigh_joint(),
    fr_base_X_fr_RL_calf_joint(),
    fr_base_X_fr_RR_hip_joint(),
    fr_base_X_fr_RR_thigh_joint(),
    fr_base_X_fr_RR_calf_joint(),
    fr_FR_thigh_X_fr_FR_hip(),
    fr_FR_hip_X_fr_FR_thigh(),
    fr_FR_calf_X_fr_FR_thigh(),
    fr_FR_thigh_X_fr_FR_calf(),
    fr_FL_thigh_X_fr_FL_hip(),
    fr_FL_hip_X_fr_FL_thigh(),
    fr_FL_calf_X_fr_FL_thigh(),
    fr_FL_thigh_X_fr_FL_calf(),
    fr_RR_thigh_X_fr_RR_hip(),
    fr_RR_hip_X_fr_RR_thigh(),
    fr_RR_calf_X_fr_RR_thigh(),
    fr_RR_thigh_X_fr_RR_calf(),
    fr_RL_thigh_X_fr_RL_hip(),
    fr_RL_hip_X_fr_RL_thigh(),
    fr_RL_calf_X_fr_RL_thigh(),
    fr_RL_thigh_X_fr_RL_calf()
{}
void HomogeneousTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

MotionTransforms::Type_fr_base_X_fr_FL_calf::Type_fr_base_X_fr_FL_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FL_calf& MotionTransforms::Type_fr_base_X_fr_FL_calf::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    (*this)(0,0) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(0,1) = (-cos_q_FL_calf_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(1,0) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,1) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(1,2) = cos_q_FL_hip_joint;
    (*this)(2,0) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,1) = (sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(3,0) = (((- ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(3,1) = ((( ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(3,2) = ( ty_FL_calf_joint * cos_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(3,3) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(3,4) = (-cos_q_FL_calf_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(4,0) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint);
    (*this)(4,1) = (((- ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)) * cos_q_FL_thigh_joint)+( ty_FL_calf_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint);
    (*this)(4,2) = ( ty_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(4,3) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(4,4) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(4,5) = cos_q_FL_hip_joint;
    (*this)(5,0) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint);
    (*this)(5,1) = (((- tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+( ty_FL_calf_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint);
    (*this)(5,2) = ( tx_FL_hip_joint * cos_q_FL_hip_joint)-( ty_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(5,3) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,4) = (sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,5) = sin_q_FL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_FL_calf_X_fr_base::Type_fr_FL_calf_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_FL_calf_X_fr_base& MotionTransforms::Type_fr_FL_calf_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,0) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(0,1) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(0,2) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,0) = (-cos_q_FL_calf_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(1,1) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(1,2) = (sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,1) = cos_q_FL_hip_joint;
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(3,0) = (((- ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(3,1) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint);
    (*this)(3,2) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint);
    (*this)(3,3) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(3,4) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(3,5) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(4,0) = ((( ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(4,1) = (((- ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)) * cos_q_FL_thigh_joint)+( ty_FL_calf_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint);
    (*this)(4,2) = (((- tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+( ty_FL_calf_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint);
    (*this)(4,3) = (-cos_q_FL_calf_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(4,4) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(4,5) = (sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,0) = ( ty_FL_calf_joint * cos_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(5,1) = ( ty_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(5,2) = ( tx_FL_hip_joint * cos_q_FL_hip_joint)-( ty_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(5,4) = cos_q_FL_hip_joint;
    (*this)(5,5) = sin_q_FL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FL_hip::Type_fr_base_X_fr_FL_hip()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = - ty_FL_hip_joint;    // Maxima DSL: -_k__ty_FL_hip_joint
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FL_hip& MotionTransforms::Type_fr_base_X_fr_FL_hip::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(1,0) = sin_q_FL_hip_joint;
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(2,0) = -cos_q_FL_hip_joint;
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(3,0) = - ty_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(3,1) =  ty_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(4,0) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(4,1) = - tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(4,3) = sin_q_FL_hip_joint;
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(5,0) =  tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(5,1) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(5,3) = -cos_q_FL_hip_joint;
    (*this)(5,4) = sin_q_FL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_FL_hip_X_fr_base::Type_fr_FL_hip_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = - ty_FL_hip_joint;    // Maxima DSL: -_k__ty_FL_hip_joint
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_FL_hip_X_fr_base& MotionTransforms::Type_fr_FL_hip_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,1) = sin_q_FL_hip_joint;
    (*this)(0,2) = -cos_q_FL_hip_joint;
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = sin_q_FL_hip_joint;
    (*this)(3,0) = - ty_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(3,1) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(3,2) =  tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(3,4) = sin_q_FL_hip_joint;
    (*this)(3,5) = -cos_q_FL_hip_joint;
    (*this)(4,0) =  ty_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(4,1) = - tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(4,2) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = sin_q_FL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FL_thigh::Type_fr_base_X_fr_FL_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FL_thigh& MotionTransforms::Type_fr_base_X_fr_FL_thigh::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    (*this)(0,0) = cos_q_FL_thigh_joint;
    (*this)(0,1) = -sin_q_FL_thigh_joint;
    (*this)(1,0) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(1,1) = sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(1,2) = cos_q_FL_hip_joint;
    (*this)(2,0) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(2,1) = -cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(3,0) = ((- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_FL_thigh_joint) * sin_q_FL_thigh_joint;
    (*this)(3,1) = ((- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_FL_thigh_joint) * cos_q_FL_thigh_joint;
    (*this)(3,2) =  ty_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(3,3) = cos_q_FL_thigh_joint;
    (*this)(3,4) = -sin_q_FL_thigh_joint;
    (*this)(4,0) = ( tx_FL_hip_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+( ty_FL_thigh_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(4,1) = ( tx_FL_hip_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-( ty_FL_thigh_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(4,2) = - tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(4,3) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(4,4) = sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(4,5) = cos_q_FL_hip_joint;
    (*this)(5,0) = ( tx_FL_hip_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(((- ty_FL_thigh_joint * cos_q_FL_hip_joint)- ty_FL_hip_joint) * cos_q_FL_thigh_joint);
    (*this)(5,1) = ((( ty_FL_thigh_joint * cos_q_FL_hip_joint)+ ty_FL_hip_joint) * sin_q_FL_thigh_joint)+( tx_FL_hip_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,2) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(5,3) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(5,4) = -cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(5,5) = sin_q_FL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_FL_thigh_X_fr_base::Type_fr_FL_thigh_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_FL_thigh_X_fr_base& MotionTransforms::Type_fr_FL_thigh_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,0) = cos_q_FL_thigh_joint;
    (*this)(0,1) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(0,2) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(1,0) = -sin_q_FL_thigh_joint;
    (*this)(1,1) = sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(1,2) = -cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(2,1) = cos_q_FL_hip_joint;
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(3,0) = ((- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_FL_thigh_joint) * sin_q_FL_thigh_joint;
    (*this)(3,1) = ( tx_FL_hip_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+( ty_FL_thigh_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(3,2) = ( tx_FL_hip_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(((- ty_FL_thigh_joint * cos_q_FL_hip_joint)- ty_FL_hip_joint) * cos_q_FL_thigh_joint);
    (*this)(3,3) = cos_q_FL_thigh_joint;
    (*this)(3,4) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(3,5) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(4,0) = ((- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_FL_thigh_joint) * cos_q_FL_thigh_joint;
    (*this)(4,1) = ( tx_FL_hip_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-( ty_FL_thigh_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(4,2) = ((( ty_FL_thigh_joint * cos_q_FL_hip_joint)+ ty_FL_hip_joint) * sin_q_FL_thigh_joint)+( tx_FL_hip_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(4,3) = -sin_q_FL_thigh_joint;
    (*this)(4,4) = sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(4,5) = -cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(5,0) =  ty_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(5,1) = - tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(5,2) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(5,4) = cos_q_FL_hip_joint;
    (*this)(5,5) = sin_q_FL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FR_calf::Type_fr_base_X_fr_FR_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FR_calf& MotionTransforms::Type_fr_base_X_fr_FR_calf::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    (*this)(0,0) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(0,1) = (-cos_q_FR_calf_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(1,0) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,1) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(1,2) = cos_q_FR_hip_joint;
    (*this)(2,0) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,1) = (sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(3,0) = (((- ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(3,1) = ((( ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(3,2) = ( ty_FR_calf_joint * cos_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(3,3) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(3,4) = (-cos_q_FR_calf_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(4,0) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint);
    (*this)(4,1) = (((- ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)) * cos_q_FR_thigh_joint)+( ty_FR_calf_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint);
    (*this)(4,2) = ( ty_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(4,3) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(4,4) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(4,5) = cos_q_FR_hip_joint;
    (*this)(5,0) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint);
    (*this)(5,1) = (((- tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+( ty_FR_calf_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint);
    (*this)(5,2) = ( tx_FR_hip_joint * cos_q_FR_hip_joint)-( ty_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(5,3) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,4) = (sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,5) = sin_q_FR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_FR_calf_X_fr_base::Type_fr_FR_calf_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_FR_calf_X_fr_base& MotionTransforms::Type_fr_FR_calf_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,0) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(0,1) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(0,2) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,0) = (-cos_q_FR_calf_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(1,1) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(1,2) = (sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,1) = cos_q_FR_hip_joint;
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(3,0) = (((- ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(3,1) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint);
    (*this)(3,2) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint);
    (*this)(3,3) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(3,4) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(3,5) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(4,0) = ((( ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(4,1) = (((- ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)) * cos_q_FR_thigh_joint)+( ty_FR_calf_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint);
    (*this)(4,2) = (((- tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+( ty_FR_calf_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint);
    (*this)(4,3) = (-cos_q_FR_calf_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(4,4) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(4,5) = (sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,0) = ( ty_FR_calf_joint * cos_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(5,1) = ( ty_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(5,2) = ( tx_FR_hip_joint * cos_q_FR_hip_joint)-( ty_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(5,4) = cos_q_FR_hip_joint;
    (*this)(5,5) = sin_q_FR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FR_hip::Type_fr_base_X_fr_FR_hip()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = - ty_FR_hip_joint;    // Maxima DSL: -_k__ty_FR_hip_joint
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FR_hip& MotionTransforms::Type_fr_base_X_fr_FR_hip::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(1,0) = sin_q_FR_hip_joint;
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(2,0) = -cos_q_FR_hip_joint;
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(3,0) = - ty_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(3,1) =  ty_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(4,0) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(4,1) = - tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(4,3) = sin_q_FR_hip_joint;
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(5,0) =  tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(5,1) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(5,3) = -cos_q_FR_hip_joint;
    (*this)(5,4) = sin_q_FR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_FR_hip_X_fr_base::Type_fr_FR_hip_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = - ty_FR_hip_joint;    // Maxima DSL: -_k__ty_FR_hip_joint
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_FR_hip_X_fr_base& MotionTransforms::Type_fr_FR_hip_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,1) = sin_q_FR_hip_joint;
    (*this)(0,2) = -cos_q_FR_hip_joint;
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = sin_q_FR_hip_joint;
    (*this)(3,0) = - ty_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(3,1) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(3,2) =  tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(3,4) = sin_q_FR_hip_joint;
    (*this)(3,5) = -cos_q_FR_hip_joint;
    (*this)(4,0) =  ty_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(4,1) = - tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(4,2) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = sin_q_FR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FR_thigh::Type_fr_base_X_fr_FR_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FR_thigh& MotionTransforms::Type_fr_base_X_fr_FR_thigh::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    (*this)(0,0) = cos_q_FR_thigh_joint;
    (*this)(0,1) = -sin_q_FR_thigh_joint;
    (*this)(1,0) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(1,1) = sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(1,2) = cos_q_FR_hip_joint;
    (*this)(2,0) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(2,1) = -cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(3,0) = ((- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_FR_thigh_joint) * sin_q_FR_thigh_joint;
    (*this)(3,1) = ((- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_FR_thigh_joint) * cos_q_FR_thigh_joint;
    (*this)(3,2) =  ty_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(3,3) = cos_q_FR_thigh_joint;
    (*this)(3,4) = -sin_q_FR_thigh_joint;
    (*this)(4,0) = ( tx_FR_hip_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+( ty_FR_thigh_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(4,1) = ( tx_FR_hip_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-( ty_FR_thigh_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(4,2) = - tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(4,3) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(4,4) = sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(4,5) = cos_q_FR_hip_joint;
    (*this)(5,0) = ( tx_FR_hip_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(((- ty_FR_thigh_joint * cos_q_FR_hip_joint)- ty_FR_hip_joint) * cos_q_FR_thigh_joint);
    (*this)(5,1) = ((( ty_FR_thigh_joint * cos_q_FR_hip_joint)+ ty_FR_hip_joint) * sin_q_FR_thigh_joint)+( tx_FR_hip_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,2) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(5,3) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(5,4) = -cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(5,5) = sin_q_FR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_FR_thigh_X_fr_base::Type_fr_FR_thigh_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_FR_thigh_X_fr_base& MotionTransforms::Type_fr_FR_thigh_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,0) = cos_q_FR_thigh_joint;
    (*this)(0,1) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(0,2) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(1,0) = -sin_q_FR_thigh_joint;
    (*this)(1,1) = sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(1,2) = -cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(2,1) = cos_q_FR_hip_joint;
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(3,0) = ((- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_FR_thigh_joint) * sin_q_FR_thigh_joint;
    (*this)(3,1) = ( tx_FR_hip_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+( ty_FR_thigh_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(3,2) = ( tx_FR_hip_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(((- ty_FR_thigh_joint * cos_q_FR_hip_joint)- ty_FR_hip_joint) * cos_q_FR_thigh_joint);
    (*this)(3,3) = cos_q_FR_thigh_joint;
    (*this)(3,4) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(3,5) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(4,0) = ((- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_FR_thigh_joint) * cos_q_FR_thigh_joint;
    (*this)(4,1) = ( tx_FR_hip_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-( ty_FR_thigh_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(4,2) = ((( ty_FR_thigh_joint * cos_q_FR_hip_joint)+ ty_FR_hip_joint) * sin_q_FR_thigh_joint)+( tx_FR_hip_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(4,3) = -sin_q_FR_thigh_joint;
    (*this)(4,4) = sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(4,5) = -cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(5,0) =  ty_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(5,1) = - tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(5,2) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(5,4) = cos_q_FR_hip_joint;
    (*this)(5,5) = sin_q_FR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RL_calf::Type_fr_base_X_fr_RL_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RL_calf& MotionTransforms::Type_fr_base_X_fr_RL_calf::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    (*this)(0,0) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(0,1) = (-cos_q_RL_calf_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(1,0) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,1) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(1,2) = cos_q_RL_hip_joint;
    (*this)(2,0) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,1) = (sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(3,0) = (((- ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(3,1) = ((( ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(3,2) = ( ty_RL_calf_joint * cos_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(3,3) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(3,4) = (-cos_q_RL_calf_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(4,0) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint);
    (*this)(4,1) = (((- ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)) * cos_q_RL_thigh_joint)+( ty_RL_calf_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint);
    (*this)(4,2) = ( ty_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(4,3) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(4,4) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(4,5) = cos_q_RL_hip_joint;
    (*this)(5,0) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint);
    (*this)(5,1) = (((- tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+( ty_RL_calf_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint);
    (*this)(5,2) = ( tx_RL_hip_joint * cos_q_RL_hip_joint)-( ty_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(5,3) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,4) = (sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,5) = sin_q_RL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_RL_calf_X_fr_base::Type_fr_RL_calf_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_RL_calf_X_fr_base& MotionTransforms::Type_fr_RL_calf_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,0) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(0,1) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(0,2) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,0) = (-cos_q_RL_calf_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(1,1) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(1,2) = (sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,1) = cos_q_RL_hip_joint;
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(3,0) = (((- ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(3,1) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint);
    (*this)(3,2) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint);
    (*this)(3,3) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(3,4) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(3,5) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(4,0) = ((( ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(4,1) = (((- ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)) * cos_q_RL_thigh_joint)+( ty_RL_calf_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint);
    (*this)(4,2) = (((- tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+( ty_RL_calf_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint);
    (*this)(4,3) = (-cos_q_RL_calf_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(4,4) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(4,5) = (sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,0) = ( ty_RL_calf_joint * cos_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(5,1) = ( ty_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(5,2) = ( tx_RL_hip_joint * cos_q_RL_hip_joint)-( ty_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(5,4) = cos_q_RL_hip_joint;
    (*this)(5,5) = sin_q_RL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RL_hip::Type_fr_base_X_fr_RL_hip()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = - ty_RL_hip_joint;    // Maxima DSL: -_k__ty_RL_hip_joint
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RL_hip& MotionTransforms::Type_fr_base_X_fr_RL_hip::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(1,0) = sin_q_RL_hip_joint;
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(2,0) = -cos_q_RL_hip_joint;
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(3,0) = - ty_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(3,1) =  ty_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(4,0) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(4,1) = - tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(4,3) = sin_q_RL_hip_joint;
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(5,0) =  tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(5,1) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(5,3) = -cos_q_RL_hip_joint;
    (*this)(5,4) = sin_q_RL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_RL_hip_X_fr_base::Type_fr_RL_hip_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = - ty_RL_hip_joint;    // Maxima DSL: -_k__ty_RL_hip_joint
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_RL_hip_X_fr_base& MotionTransforms::Type_fr_RL_hip_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,1) = sin_q_RL_hip_joint;
    (*this)(0,2) = -cos_q_RL_hip_joint;
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = sin_q_RL_hip_joint;
    (*this)(3,0) = - ty_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(3,1) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(3,2) =  tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(3,4) = sin_q_RL_hip_joint;
    (*this)(3,5) = -cos_q_RL_hip_joint;
    (*this)(4,0) =  ty_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(4,1) = - tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(4,2) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = sin_q_RL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RL_thigh::Type_fr_base_X_fr_RL_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RL_thigh& MotionTransforms::Type_fr_base_X_fr_RL_thigh::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    (*this)(0,0) = cos_q_RL_thigh_joint;
    (*this)(0,1) = -sin_q_RL_thigh_joint;
    (*this)(1,0) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(1,1) = sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(1,2) = cos_q_RL_hip_joint;
    (*this)(2,0) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(2,1) = -cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(3,0) = ((- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_RL_thigh_joint) * sin_q_RL_thigh_joint;
    (*this)(3,1) = ((- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_RL_thigh_joint) * cos_q_RL_thigh_joint;
    (*this)(3,2) =  ty_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(3,3) = cos_q_RL_thigh_joint;
    (*this)(3,4) = -sin_q_RL_thigh_joint;
    (*this)(4,0) = ( tx_RL_hip_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+( ty_RL_thigh_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(4,1) = ( tx_RL_hip_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-( ty_RL_thigh_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(4,2) = - tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(4,3) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(4,4) = sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(4,5) = cos_q_RL_hip_joint;
    (*this)(5,0) = ( tx_RL_hip_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(((- ty_RL_thigh_joint * cos_q_RL_hip_joint)- ty_RL_hip_joint) * cos_q_RL_thigh_joint);
    (*this)(5,1) = ((( ty_RL_thigh_joint * cos_q_RL_hip_joint)+ ty_RL_hip_joint) * sin_q_RL_thigh_joint)+( tx_RL_hip_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,2) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(5,3) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(5,4) = -cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(5,5) = sin_q_RL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_RL_thigh_X_fr_base::Type_fr_RL_thigh_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_RL_thigh_X_fr_base& MotionTransforms::Type_fr_RL_thigh_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,0) = cos_q_RL_thigh_joint;
    (*this)(0,1) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(0,2) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(1,0) = -sin_q_RL_thigh_joint;
    (*this)(1,1) = sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(1,2) = -cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(2,1) = cos_q_RL_hip_joint;
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(3,0) = ((- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_RL_thigh_joint) * sin_q_RL_thigh_joint;
    (*this)(3,1) = ( tx_RL_hip_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+( ty_RL_thigh_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(3,2) = ( tx_RL_hip_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(((- ty_RL_thigh_joint * cos_q_RL_hip_joint)- ty_RL_hip_joint) * cos_q_RL_thigh_joint);
    (*this)(3,3) = cos_q_RL_thigh_joint;
    (*this)(3,4) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(3,5) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(4,0) = ((- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_RL_thigh_joint) * cos_q_RL_thigh_joint;
    (*this)(4,1) = ( tx_RL_hip_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-( ty_RL_thigh_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(4,2) = ((( ty_RL_thigh_joint * cos_q_RL_hip_joint)+ ty_RL_hip_joint) * sin_q_RL_thigh_joint)+( tx_RL_hip_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(4,3) = -sin_q_RL_thigh_joint;
    (*this)(4,4) = sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(4,5) = -cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(5,0) =  ty_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(5,1) = - tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(5,2) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(5,4) = cos_q_RL_hip_joint;
    (*this)(5,5) = sin_q_RL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RR_calf::Type_fr_base_X_fr_RR_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RR_calf& MotionTransforms::Type_fr_base_X_fr_RR_calf::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    (*this)(0,0) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(0,1) = (-cos_q_RR_calf_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(1,0) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,1) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(1,2) = cos_q_RR_hip_joint;
    (*this)(2,0) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,1) = (sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(3,0) = (((- ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(3,1) = ((( ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(3,2) = ( ty_RR_calf_joint * cos_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(3,3) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(3,4) = (-cos_q_RR_calf_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(4,0) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint);
    (*this)(4,1) = (((- ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)) * cos_q_RR_thigh_joint)+( ty_RR_calf_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint);
    (*this)(4,2) = ( ty_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(4,3) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(4,4) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(4,5) = cos_q_RR_hip_joint;
    (*this)(5,0) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint);
    (*this)(5,1) = (((- tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+( ty_RR_calf_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint);
    (*this)(5,2) = ( tx_RR_hip_joint * cos_q_RR_hip_joint)-( ty_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(5,3) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,4) = (sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,5) = sin_q_RR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_RR_calf_X_fr_base::Type_fr_RR_calf_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_RR_calf_X_fr_base& MotionTransforms::Type_fr_RR_calf_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,0) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(0,1) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(0,2) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,0) = (-cos_q_RR_calf_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(1,1) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(1,2) = (sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,1) = cos_q_RR_hip_joint;
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(3,0) = (((- ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(3,1) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint);
    (*this)(3,2) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint);
    (*this)(3,3) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(3,4) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(3,5) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(4,0) = ((( ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(4,1) = (((- ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)) * cos_q_RR_thigh_joint)+( ty_RR_calf_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint);
    (*this)(4,2) = (((- tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+( ty_RR_calf_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint);
    (*this)(4,3) = (-cos_q_RR_calf_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(4,4) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(4,5) = (sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,0) = ( ty_RR_calf_joint * cos_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(5,1) = ( ty_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(5,2) = ( tx_RR_hip_joint * cos_q_RR_hip_joint)-( ty_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(5,4) = cos_q_RR_hip_joint;
    (*this)(5,5) = sin_q_RR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RR_hip::Type_fr_base_X_fr_RR_hip()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = - ty_RR_hip_joint;    // Maxima DSL: -_k__ty_RR_hip_joint
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RR_hip& MotionTransforms::Type_fr_base_X_fr_RR_hip::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(1,0) = sin_q_RR_hip_joint;
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(2,0) = -cos_q_RR_hip_joint;
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(3,0) = - ty_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(3,1) =  ty_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(4,0) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(4,1) = - tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(4,3) = sin_q_RR_hip_joint;
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(5,0) =  tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(5,1) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(5,3) = -cos_q_RR_hip_joint;
    (*this)(5,4) = sin_q_RR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_RR_hip_X_fr_base::Type_fr_RR_hip_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = - ty_RR_hip_joint;    // Maxima DSL: -_k__ty_RR_hip_joint
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_RR_hip_X_fr_base& MotionTransforms::Type_fr_RR_hip_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,1) = sin_q_RR_hip_joint;
    (*this)(0,2) = -cos_q_RR_hip_joint;
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = sin_q_RR_hip_joint;
    (*this)(3,0) = - ty_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(3,1) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(3,2) =  tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(3,4) = sin_q_RR_hip_joint;
    (*this)(3,5) = -cos_q_RR_hip_joint;
    (*this)(4,0) =  ty_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(4,1) = - tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(4,2) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = sin_q_RR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RR_thigh::Type_fr_base_X_fr_RR_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RR_thigh& MotionTransforms::Type_fr_base_X_fr_RR_thigh::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    (*this)(0,0) = cos_q_RR_thigh_joint;
    (*this)(0,1) = -sin_q_RR_thigh_joint;
    (*this)(1,0) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(1,1) = sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(1,2) = cos_q_RR_hip_joint;
    (*this)(2,0) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(2,1) = -cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(3,0) = ((- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_RR_thigh_joint) * sin_q_RR_thigh_joint;
    (*this)(3,1) = ((- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_RR_thigh_joint) * cos_q_RR_thigh_joint;
    (*this)(3,2) =  ty_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(3,3) = cos_q_RR_thigh_joint;
    (*this)(3,4) = -sin_q_RR_thigh_joint;
    (*this)(4,0) = ( tx_RR_hip_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+( ty_RR_thigh_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(4,1) = ( tx_RR_hip_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-( ty_RR_thigh_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(4,2) = - tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(4,3) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(4,4) = sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(4,5) = cos_q_RR_hip_joint;
    (*this)(5,0) = ( tx_RR_hip_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(((- ty_RR_thigh_joint * cos_q_RR_hip_joint)- ty_RR_hip_joint) * cos_q_RR_thigh_joint);
    (*this)(5,1) = ((( ty_RR_thigh_joint * cos_q_RR_hip_joint)+ ty_RR_hip_joint) * sin_q_RR_thigh_joint)+( tx_RR_hip_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,2) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(5,3) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(5,4) = -cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(5,5) = sin_q_RR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_RR_thigh_X_fr_base::Type_fr_RR_thigh_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_RR_thigh_X_fr_base& MotionTransforms::Type_fr_RR_thigh_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,0) = cos_q_RR_thigh_joint;
    (*this)(0,1) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(0,2) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(1,0) = -sin_q_RR_thigh_joint;
    (*this)(1,1) = sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(1,2) = -cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(2,1) = cos_q_RR_hip_joint;
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(3,0) = ((- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_RR_thigh_joint) * sin_q_RR_thigh_joint;
    (*this)(3,1) = ( tx_RR_hip_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+( ty_RR_thigh_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(3,2) = ( tx_RR_hip_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(((- ty_RR_thigh_joint * cos_q_RR_hip_joint)- ty_RR_hip_joint) * cos_q_RR_thigh_joint);
    (*this)(3,3) = cos_q_RR_thigh_joint;
    (*this)(3,4) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(3,5) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(4,0) = ((- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_RR_thigh_joint) * cos_q_RR_thigh_joint;
    (*this)(4,1) = ( tx_RR_hip_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-( ty_RR_thigh_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(4,2) = ((( ty_RR_thigh_joint * cos_q_RR_hip_joint)+ ty_RR_hip_joint) * sin_q_RR_thigh_joint)+( tx_RR_hip_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(4,3) = -sin_q_RR_thigh_joint;
    (*this)(4,4) = sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(4,5) = -cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(5,0) =  ty_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(5,1) = - tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(5,2) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(5,4) = cos_q_RR_hip_joint;
    (*this)(5,5) = sin_q_RR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FL_calf_COM::Type_fr_base_X_fr_FL_calf_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FL_calf_COM& MotionTransforms::Type_fr_base_X_fr_FL_calf_COM::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    (*this)(0,0) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(0,2) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(1,0) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,0) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,2) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(3,0) = (((- ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(3,1) = ((( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)-( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_fr_FL_calf_COM * sin_q_FL_calf_joint)+( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * cos_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(3,2) = (((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(3,3) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(3,5) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(4,0) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * cos_q_FL_calf_joint)- ty_fr_FL_calf_COM) * cos_q_FL_hip_joint);
    (*this)(4,1) = ((( tx_fr_FL_calf_COM * sin_q_FL_calf_joint)+( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+((( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)-( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)) * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(4,2) = ((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * sin_q_FL_calf_joint)- tx_fr_FL_calf_COM) * cos_q_FL_hip_joint);
    (*this)(4,3) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,0) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * cos_q_FL_calf_joint)- ty_fr_FL_calf_COM) * sin_q_FL_hip_joint);
    (*this)(5,1) = (((- tx_fr_FL_calf_COM * sin_q_FL_calf_joint)-( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)- ty_FL_calf_joint) * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+((( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)-( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)) * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)+( tx_FL_hip_joint * cos_q_FL_hip_joint);
    (*this)(5,2) = ((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * sin_q_FL_calf_joint)- tx_fr_FL_calf_COM) * sin_q_FL_hip_joint);
    (*this)(5,3) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,4) = sin_q_FL_hip_joint;
    (*this)(5,5) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    return *this;
}
MotionTransforms::Type_fr_FL_calf_COM_X_fr_base::Type_fr_FL_calf_COM_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(4,3) = 0.0;
}

const MotionTransforms::Type_fr_FL_calf_COM_X_fr_base& MotionTransforms::Type_fr_FL_calf_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,0) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(0,1) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(0,2) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = sin_q_FL_hip_joint;
    (*this)(2,0) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(2,1) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,2) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(3,0) = (((- ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(3,1) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * cos_q_FL_calf_joint)- ty_fr_FL_calf_COM) * cos_q_FL_hip_joint);
    (*this)(3,2) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * cos_q_FL_calf_joint)- ty_fr_FL_calf_COM) * sin_q_FL_hip_joint);
    (*this)(3,3) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(3,4) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(3,5) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(4,0) = ((( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)-( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_fr_FL_calf_COM * sin_q_FL_calf_joint)+( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * cos_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(4,1) = ((( tx_fr_FL_calf_COM * sin_q_FL_calf_joint)+( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+((( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)-( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)) * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(4,2) = (((- tx_fr_FL_calf_COM * sin_q_FL_calf_joint)-( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)- ty_FL_calf_joint) * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+((( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)-( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)) * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)+( tx_FL_hip_joint * cos_q_FL_hip_joint);
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = sin_q_FL_hip_joint;
    (*this)(5,0) = (((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(5,1) = ((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * sin_q_FL_calf_joint)- tx_fr_FL_calf_COM) * cos_q_FL_hip_joint);
    (*this)(5,2) = ((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * sin_q_FL_calf_joint)- tx_fr_FL_calf_COM) * sin_q_FL_hip_joint);
    (*this)(5,3) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(5,4) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,5) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FL_foot::Type_fr_base_X_fr_FL_foot()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FL_foot& MotionTransforms::Type_fr_base_X_fr_FL_foot::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    (*this)(0,0) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(0,2) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(1,0) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,0) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,2) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(3,0) = (((- ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(3,1) = (- ty_fr_FL_foot * sin_q_FL_calf_joint * sin_q_FL_thigh_joint)+((( ty_fr_FL_foot * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * cos_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(3,2) = (((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(3,3) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(3,5) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(4,0) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * cos_q_FL_calf_joint)- ty_fr_FL_foot) * cos_q_FL_hip_joint);
    (*this)(4,1) = ((( ty_fr_FL_foot * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+( ty_fr_FL_foot * sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(4,2) = ((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint);
    (*this)(4,3) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,0) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * cos_q_FL_calf_joint)- ty_fr_FL_foot) * sin_q_FL_hip_joint);
    (*this)(5,1) = (((- ty_fr_FL_foot * cos_q_FL_calf_joint)- ty_FL_calf_joint) * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-( ty_fr_FL_foot * sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)+( tx_FL_hip_joint * cos_q_FL_hip_joint);
    (*this)(5,2) = ((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint);
    (*this)(5,3) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,4) = sin_q_FL_hip_joint;
    (*this)(5,5) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    return *this;
}
MotionTransforms::Type_fr_FL_foot_X_fr_base::Type_fr_FL_foot_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(4,3) = 0.0;
}

const MotionTransforms::Type_fr_FL_foot_X_fr_base& MotionTransforms::Type_fr_FL_foot_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,0) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(0,1) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(0,2) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = sin_q_FL_hip_joint;
    (*this)(2,0) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(2,1) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,2) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(3,0) = (((- ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(3,1) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * cos_q_FL_calf_joint)- ty_fr_FL_foot) * cos_q_FL_hip_joint);
    (*this)(3,2) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * cos_q_FL_calf_joint)- ty_fr_FL_foot) * sin_q_FL_hip_joint);
    (*this)(3,3) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(3,4) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(3,5) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(4,0) = (- ty_fr_FL_foot * sin_q_FL_calf_joint * sin_q_FL_thigh_joint)+((( ty_fr_FL_foot * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * cos_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(4,1) = ((( ty_fr_FL_foot * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+( ty_fr_FL_foot * sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(4,2) = (((- ty_fr_FL_foot * cos_q_FL_calf_joint)- ty_FL_calf_joint) * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-( ty_fr_FL_foot * sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)+( tx_FL_hip_joint * cos_q_FL_hip_joint);
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = sin_q_FL_hip_joint;
    (*this)(5,0) = (((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(5,1) = ((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint);
    (*this)(5,2) = ((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint);
    (*this)(5,3) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(5,4) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,5) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FL_hip_COM::Type_fr_base_X_fr_FL_hip_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FL_hip_COM& MotionTransforms::Type_fr_base_X_fr_FL_hip_COM::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = -sin_q_FL_hip_joint;
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,2) = cos_q_FL_hip_joint;
    (*this)(3,1) = ( ty_FL_hip_joint * sin_q_FL_hip_joint)+ tx_fr_FL_hip_COM;
    (*this)(3,2) = ( ty_FL_hip_joint * cos_q_FL_hip_joint)+ ty_fr_FL_hip_COM;
    (*this)(4,0) = ( ty_fr_FL_hip_COM * sin_q_FL_hip_joint)-( tx_fr_FL_hip_COM * cos_q_FL_hip_joint);
    (*this)(4,1) = (- tz_fr_FL_hip_COM- tx_FL_hip_joint) * sin_q_FL_hip_joint;
    (*this)(4,2) = (- tz_fr_FL_hip_COM- tx_FL_hip_joint) * cos_q_FL_hip_joint;
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = -sin_q_FL_hip_joint;
    (*this)(5,0) = (- tx_fr_FL_hip_COM * sin_q_FL_hip_joint)-( ty_fr_FL_hip_COM * cos_q_FL_hip_joint)- ty_FL_hip_joint;
    (*this)(5,1) = ( tz_fr_FL_hip_COM+ tx_FL_hip_joint) * cos_q_FL_hip_joint;
    (*this)(5,2) = (- tz_fr_FL_hip_COM- tx_FL_hip_joint) * sin_q_FL_hip_joint;
    (*this)(5,4) = sin_q_FL_hip_joint;
    (*this)(5,5) = cos_q_FL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_FL_hip_COM_X_fr_base::Type_fr_FL_hip_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_FL_hip_COM_X_fr_base& MotionTransforms::Type_fr_FL_hip_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = sin_q_FL_hip_joint;
    (*this)(2,1) = -sin_q_FL_hip_joint;
    (*this)(2,2) = cos_q_FL_hip_joint;
    (*this)(3,1) = ( ty_fr_FL_hip_COM * sin_q_FL_hip_joint)-( tx_fr_FL_hip_COM * cos_q_FL_hip_joint);
    (*this)(3,2) = (- tx_fr_FL_hip_COM * sin_q_FL_hip_joint)-( ty_fr_FL_hip_COM * cos_q_FL_hip_joint)- ty_FL_hip_joint;
    (*this)(4,0) = ( ty_FL_hip_joint * sin_q_FL_hip_joint)+ tx_fr_FL_hip_COM;
    (*this)(4,1) = (- tz_fr_FL_hip_COM- tx_FL_hip_joint) * sin_q_FL_hip_joint;
    (*this)(4,2) = ( tz_fr_FL_hip_COM+ tx_FL_hip_joint) * cos_q_FL_hip_joint;
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = sin_q_FL_hip_joint;
    (*this)(5,0) = ( ty_FL_hip_joint * cos_q_FL_hip_joint)+ ty_fr_FL_hip_COM;
    (*this)(5,1) = (- tz_fr_FL_hip_COM- tx_FL_hip_joint) * cos_q_FL_hip_joint;
    (*this)(5,2) = (- tz_fr_FL_hip_COM- tx_FL_hip_joint) * sin_q_FL_hip_joint;
    (*this)(5,4) = -sin_q_FL_hip_joint;
    (*this)(5,5) = cos_q_FL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FL_thigh_COM::Type_fr_base_X_fr_FL_thigh_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FL_thigh_COM& MotionTransforms::Type_fr_base_X_fr_FL_thigh_COM::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    (*this)(0,0) = cos_q_FL_thigh_joint;
    (*this)(0,2) = sin_q_FL_thigh_joint;
    (*this)(1,0) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = -sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(2,0) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,2) = cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(3,0) = ((- ty_FL_hip_joint * cos_q_FL_hip_joint)- tz_fr_FL_thigh_COM- ty_FL_thigh_joint) * sin_q_FL_thigh_joint;
    (*this)(3,1) = ( tx_fr_FL_thigh_COM * sin_q_FL_thigh_joint)+( ty_fr_FL_thigh_COM * cos_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(3,2) = (( ty_FL_hip_joint * cos_q_FL_hip_joint)+ tz_fr_FL_thigh_COM+ ty_FL_thigh_joint) * cos_q_FL_thigh_joint;
    (*this)(3,3) = cos_q_FL_thigh_joint;
    (*this)(3,5) = sin_q_FL_thigh_joint;
    (*this)(4,0) = ( tx_FL_hip_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+(( tz_fr_FL_thigh_COM+ ty_FL_thigh_joint) * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( ty_fr_FL_thigh_COM * cos_q_FL_hip_joint);
    (*this)(4,1) = ( ty_fr_FL_thigh_COM * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-( tx_fr_FL_thigh_COM * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(4,2) = (( tz_fr_FL_thigh_COM+ ty_FL_thigh_joint) * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-( tx_FL_hip_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_fr_FL_thigh_COM * cos_q_FL_hip_joint);
    (*this)(4,3) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = -sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(5,0) = ( tx_FL_hip_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+((((- tz_fr_FL_thigh_COM- ty_FL_thigh_joint) * cos_q_FL_hip_joint)- ty_FL_hip_joint) * cos_q_FL_thigh_joint)-( ty_fr_FL_thigh_COM * sin_q_FL_hip_joint);
    (*this)(5,1) = (- ty_fr_FL_thigh_COM * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+( tx_fr_FL_thigh_COM * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)+( tx_FL_hip_joint * cos_q_FL_hip_joint);
    (*this)(5,2) = ((((- tz_fr_FL_thigh_COM- ty_FL_thigh_joint) * cos_q_FL_hip_joint)- ty_FL_hip_joint) * sin_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_fr_FL_thigh_COM * sin_q_FL_hip_joint);
    (*this)(5,3) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(5,4) = sin_q_FL_hip_joint;
    (*this)(5,5) = cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    return *this;
}
MotionTransforms::Type_fr_FL_thigh_COM_X_fr_base::Type_fr_FL_thigh_COM_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(4,3) = 0.0;
}

const MotionTransforms::Type_fr_FL_thigh_COM_X_fr_base& MotionTransforms::Type_fr_FL_thigh_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,0) = cos_q_FL_thigh_joint;
    (*this)(0,1) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(0,2) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = sin_q_FL_hip_joint;
    (*this)(2,0) = sin_q_FL_thigh_joint;
    (*this)(2,1) = -sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(2,2) = cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(3,0) = ((- ty_FL_hip_joint * cos_q_FL_hip_joint)- tz_fr_FL_thigh_COM- ty_FL_thigh_joint) * sin_q_FL_thigh_joint;
    (*this)(3,1) = ( tx_FL_hip_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+(( tz_fr_FL_thigh_COM+ ty_FL_thigh_joint) * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( ty_fr_FL_thigh_COM * cos_q_FL_hip_joint);
    (*this)(3,2) = ( tx_FL_hip_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+((((- tz_fr_FL_thigh_COM- ty_FL_thigh_joint) * cos_q_FL_hip_joint)- ty_FL_hip_joint) * cos_q_FL_thigh_joint)-( ty_fr_FL_thigh_COM * sin_q_FL_hip_joint);
    (*this)(3,3) = cos_q_FL_thigh_joint;
    (*this)(3,4) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(3,5) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(4,0) = ( tx_fr_FL_thigh_COM * sin_q_FL_thigh_joint)+( ty_fr_FL_thigh_COM * cos_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(4,1) = ( ty_fr_FL_thigh_COM * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-( tx_fr_FL_thigh_COM * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(4,2) = (- ty_fr_FL_thigh_COM * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+( tx_fr_FL_thigh_COM * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)+( tx_FL_hip_joint * cos_q_FL_hip_joint);
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = sin_q_FL_hip_joint;
    (*this)(5,0) = (( ty_FL_hip_joint * cos_q_FL_hip_joint)+ tz_fr_FL_thigh_COM+ ty_FL_thigh_joint) * cos_q_FL_thigh_joint;
    (*this)(5,1) = (( tz_fr_FL_thigh_COM+ ty_FL_thigh_joint) * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-( tx_FL_hip_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_fr_FL_thigh_COM * cos_q_FL_hip_joint);
    (*this)(5,2) = ((((- tz_fr_FL_thigh_COM- ty_FL_thigh_joint) * cos_q_FL_hip_joint)- ty_FL_hip_joint) * sin_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_fr_FL_thigh_COM * sin_q_FL_hip_joint);
    (*this)(5,3) = sin_q_FL_thigh_joint;
    (*this)(5,4) = -sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(5,5) = cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FL_thigh_shoulder::Type_fr_base_X_fr_FL_thigh_shoulder()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FL_thigh_shoulder& MotionTransforms::Type_fr_base_X_fr_FL_thigh_shoulder::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = -sin_q_FL_hip_joint;
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,2) = cos_q_FL_hip_joint;
    (*this)(3,1) =  ty_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(3,2) = ( ty_FL_hip_joint * cos_q_FL_hip_joint)+ ty_fr_FL_thigh_shoulder;
    (*this)(4,0) =  ty_fr_FL_thigh_shoulder * sin_q_FL_hip_joint;
    (*this)(4,1) = - tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(4,2) = - tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = -sin_q_FL_hip_joint;
    (*this)(5,0) = (- ty_fr_FL_thigh_shoulder * cos_q_FL_hip_joint)- ty_FL_hip_joint;
    (*this)(5,1) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(5,2) = - tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(5,4) = sin_q_FL_hip_joint;
    (*this)(5,5) = cos_q_FL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_FL_thigh_shoulder_X_fr_base::Type_fr_FL_thigh_shoulder_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_FL_thigh_shoulder_X_fr_base& MotionTransforms::Type_fr_FL_thigh_shoulder_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = sin_q_FL_hip_joint;
    (*this)(2,1) = -sin_q_FL_hip_joint;
    (*this)(2,2) = cos_q_FL_hip_joint;
    (*this)(3,1) =  ty_fr_FL_thigh_shoulder * sin_q_FL_hip_joint;
    (*this)(3,2) = (- ty_fr_FL_thigh_shoulder * cos_q_FL_hip_joint)- ty_FL_hip_joint;
    (*this)(4,0) =  ty_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(4,1) = - tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(4,2) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = sin_q_FL_hip_joint;
    (*this)(5,0) = ( ty_FL_hip_joint * cos_q_FL_hip_joint)+ ty_fr_FL_thigh_shoulder;
    (*this)(5,1) = - tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(5,2) = - tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(5,4) = -sin_q_FL_hip_joint;
    (*this)(5,5) = cos_q_FL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FR_calf_COM::Type_fr_base_X_fr_FR_calf_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FR_calf_COM& MotionTransforms::Type_fr_base_X_fr_FR_calf_COM::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    (*this)(0,0) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(0,2) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(1,0) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,0) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,2) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(3,0) = (((- ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(3,1) = ((( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)-( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_fr_FR_calf_COM * sin_q_FR_calf_joint)+( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * cos_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(3,2) = (((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(3,3) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(3,5) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(4,0) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * cos_q_FR_calf_joint)- ty_fr_FR_calf_COM) * cos_q_FR_hip_joint);
    (*this)(4,1) = ((( tx_fr_FR_calf_COM * sin_q_FR_calf_joint)+( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+((( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)-( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)) * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(4,2) = ((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * sin_q_FR_calf_joint)- tx_fr_FR_calf_COM) * cos_q_FR_hip_joint);
    (*this)(4,3) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,0) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * cos_q_FR_calf_joint)- ty_fr_FR_calf_COM) * sin_q_FR_hip_joint);
    (*this)(5,1) = (((- tx_fr_FR_calf_COM * sin_q_FR_calf_joint)-( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)- ty_FR_calf_joint) * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+((( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)-( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)) * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)+( tx_FR_hip_joint * cos_q_FR_hip_joint);
    (*this)(5,2) = ((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * sin_q_FR_calf_joint)- tx_fr_FR_calf_COM) * sin_q_FR_hip_joint);
    (*this)(5,3) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,4) = sin_q_FR_hip_joint;
    (*this)(5,5) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    return *this;
}
MotionTransforms::Type_fr_FR_calf_COM_X_fr_base::Type_fr_FR_calf_COM_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(4,3) = 0.0;
}

const MotionTransforms::Type_fr_FR_calf_COM_X_fr_base& MotionTransforms::Type_fr_FR_calf_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,0) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(0,1) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(0,2) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = sin_q_FR_hip_joint;
    (*this)(2,0) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(2,1) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,2) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(3,0) = (((- ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(3,1) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * cos_q_FR_calf_joint)- ty_fr_FR_calf_COM) * cos_q_FR_hip_joint);
    (*this)(3,2) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * cos_q_FR_calf_joint)- ty_fr_FR_calf_COM) * sin_q_FR_hip_joint);
    (*this)(3,3) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(3,4) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(3,5) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(4,0) = ((( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)-( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_fr_FR_calf_COM * sin_q_FR_calf_joint)+( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * cos_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(4,1) = ((( tx_fr_FR_calf_COM * sin_q_FR_calf_joint)+( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+((( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)-( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)) * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(4,2) = (((- tx_fr_FR_calf_COM * sin_q_FR_calf_joint)-( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)- ty_FR_calf_joint) * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+((( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)-( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)) * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)+( tx_FR_hip_joint * cos_q_FR_hip_joint);
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = sin_q_FR_hip_joint;
    (*this)(5,0) = (((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(5,1) = ((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * sin_q_FR_calf_joint)- tx_fr_FR_calf_COM) * cos_q_FR_hip_joint);
    (*this)(5,2) = ((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * sin_q_FR_calf_joint)- tx_fr_FR_calf_COM) * sin_q_FR_hip_joint);
    (*this)(5,3) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(5,4) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,5) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FR_foot::Type_fr_base_X_fr_FR_foot()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FR_foot& MotionTransforms::Type_fr_base_X_fr_FR_foot::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    (*this)(0,0) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(0,2) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(1,0) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,0) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,2) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(3,0) = (((- ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(3,1) = (- ty_fr_FR_foot * sin_q_FR_calf_joint * sin_q_FR_thigh_joint)+((( ty_fr_FR_foot * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * cos_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(3,2) = (((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(3,3) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(3,5) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(4,0) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * cos_q_FR_calf_joint)- ty_fr_FR_foot) * cos_q_FR_hip_joint);
    (*this)(4,1) = ((( ty_fr_FR_foot * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+( ty_fr_FR_foot * sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(4,2) = ((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint);
    (*this)(4,3) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,0) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * cos_q_FR_calf_joint)- ty_fr_FR_foot) * sin_q_FR_hip_joint);
    (*this)(5,1) = (((- ty_fr_FR_foot * cos_q_FR_calf_joint)- ty_FR_calf_joint) * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-( ty_fr_FR_foot * sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)+( tx_FR_hip_joint * cos_q_FR_hip_joint);
    (*this)(5,2) = ((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint);
    (*this)(5,3) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,4) = sin_q_FR_hip_joint;
    (*this)(5,5) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    return *this;
}
MotionTransforms::Type_fr_FR_foot_X_fr_base::Type_fr_FR_foot_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(4,3) = 0.0;
}

const MotionTransforms::Type_fr_FR_foot_X_fr_base& MotionTransforms::Type_fr_FR_foot_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,0) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(0,1) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(0,2) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = sin_q_FR_hip_joint;
    (*this)(2,0) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(2,1) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,2) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(3,0) = (((- ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(3,1) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * cos_q_FR_calf_joint)- ty_fr_FR_foot) * cos_q_FR_hip_joint);
    (*this)(3,2) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * cos_q_FR_calf_joint)- ty_fr_FR_foot) * sin_q_FR_hip_joint);
    (*this)(3,3) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(3,4) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(3,5) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(4,0) = (- ty_fr_FR_foot * sin_q_FR_calf_joint * sin_q_FR_thigh_joint)+((( ty_fr_FR_foot * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * cos_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(4,1) = ((( ty_fr_FR_foot * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+( ty_fr_FR_foot * sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(4,2) = (((- ty_fr_FR_foot * cos_q_FR_calf_joint)- ty_FR_calf_joint) * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-( ty_fr_FR_foot * sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)+( tx_FR_hip_joint * cos_q_FR_hip_joint);
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = sin_q_FR_hip_joint;
    (*this)(5,0) = (((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(5,1) = ((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint);
    (*this)(5,2) = ((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint);
    (*this)(5,3) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(5,4) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,5) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FR_hip_COM::Type_fr_base_X_fr_FR_hip_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FR_hip_COM& MotionTransforms::Type_fr_base_X_fr_FR_hip_COM::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = -sin_q_FR_hip_joint;
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,2) = cos_q_FR_hip_joint;
    (*this)(3,1) = ( ty_FR_hip_joint * sin_q_FR_hip_joint)+ tx_fr_FR_hip_COM;
    (*this)(3,2) = ( ty_FR_hip_joint * cos_q_FR_hip_joint)+ ty_fr_FR_hip_COM;
    (*this)(4,0) = ( ty_fr_FR_hip_COM * sin_q_FR_hip_joint)-( tx_fr_FR_hip_COM * cos_q_FR_hip_joint);
    (*this)(4,1) = (- tz_fr_FR_hip_COM- tx_FR_hip_joint) * sin_q_FR_hip_joint;
    (*this)(4,2) = (- tz_fr_FR_hip_COM- tx_FR_hip_joint) * cos_q_FR_hip_joint;
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = -sin_q_FR_hip_joint;
    (*this)(5,0) = (- tx_fr_FR_hip_COM * sin_q_FR_hip_joint)-( ty_fr_FR_hip_COM * cos_q_FR_hip_joint)- ty_FR_hip_joint;
    (*this)(5,1) = ( tz_fr_FR_hip_COM+ tx_FR_hip_joint) * cos_q_FR_hip_joint;
    (*this)(5,2) = (- tz_fr_FR_hip_COM- tx_FR_hip_joint) * sin_q_FR_hip_joint;
    (*this)(5,4) = sin_q_FR_hip_joint;
    (*this)(5,5) = cos_q_FR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_FR_hip_COM_X_fr_base::Type_fr_FR_hip_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_FR_hip_COM_X_fr_base& MotionTransforms::Type_fr_FR_hip_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = sin_q_FR_hip_joint;
    (*this)(2,1) = -sin_q_FR_hip_joint;
    (*this)(2,2) = cos_q_FR_hip_joint;
    (*this)(3,1) = ( ty_fr_FR_hip_COM * sin_q_FR_hip_joint)-( tx_fr_FR_hip_COM * cos_q_FR_hip_joint);
    (*this)(3,2) = (- tx_fr_FR_hip_COM * sin_q_FR_hip_joint)-( ty_fr_FR_hip_COM * cos_q_FR_hip_joint)- ty_FR_hip_joint;
    (*this)(4,0) = ( ty_FR_hip_joint * sin_q_FR_hip_joint)+ tx_fr_FR_hip_COM;
    (*this)(4,1) = (- tz_fr_FR_hip_COM- tx_FR_hip_joint) * sin_q_FR_hip_joint;
    (*this)(4,2) = ( tz_fr_FR_hip_COM+ tx_FR_hip_joint) * cos_q_FR_hip_joint;
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = sin_q_FR_hip_joint;
    (*this)(5,0) = ( ty_FR_hip_joint * cos_q_FR_hip_joint)+ ty_fr_FR_hip_COM;
    (*this)(5,1) = (- tz_fr_FR_hip_COM- tx_FR_hip_joint) * cos_q_FR_hip_joint;
    (*this)(5,2) = (- tz_fr_FR_hip_COM- tx_FR_hip_joint) * sin_q_FR_hip_joint;
    (*this)(5,4) = -sin_q_FR_hip_joint;
    (*this)(5,5) = cos_q_FR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FR_thigh_COM::Type_fr_base_X_fr_FR_thigh_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FR_thigh_COM& MotionTransforms::Type_fr_base_X_fr_FR_thigh_COM::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    (*this)(0,0) = cos_q_FR_thigh_joint;
    (*this)(0,2) = sin_q_FR_thigh_joint;
    (*this)(1,0) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = -sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(2,0) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,2) = cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(3,0) = ((- ty_FR_hip_joint * cos_q_FR_hip_joint)- tz_fr_FR_thigh_COM- ty_FR_thigh_joint) * sin_q_FR_thigh_joint;
    (*this)(3,1) = ( tx_fr_FR_thigh_COM * sin_q_FR_thigh_joint)+( ty_fr_FR_thigh_COM * cos_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(3,2) = (( ty_FR_hip_joint * cos_q_FR_hip_joint)+ tz_fr_FR_thigh_COM+ ty_FR_thigh_joint) * cos_q_FR_thigh_joint;
    (*this)(3,3) = cos_q_FR_thigh_joint;
    (*this)(3,5) = sin_q_FR_thigh_joint;
    (*this)(4,0) = ( tx_FR_hip_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+(( tz_fr_FR_thigh_COM+ ty_FR_thigh_joint) * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( ty_fr_FR_thigh_COM * cos_q_FR_hip_joint);
    (*this)(4,1) = ( ty_fr_FR_thigh_COM * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-( tx_fr_FR_thigh_COM * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(4,2) = (( tz_fr_FR_thigh_COM+ ty_FR_thigh_joint) * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-( tx_FR_hip_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_fr_FR_thigh_COM * cos_q_FR_hip_joint);
    (*this)(4,3) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = -sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(5,0) = ( tx_FR_hip_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+((((- tz_fr_FR_thigh_COM- ty_FR_thigh_joint) * cos_q_FR_hip_joint)- ty_FR_hip_joint) * cos_q_FR_thigh_joint)-( ty_fr_FR_thigh_COM * sin_q_FR_hip_joint);
    (*this)(5,1) = (- ty_fr_FR_thigh_COM * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+( tx_fr_FR_thigh_COM * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)+( tx_FR_hip_joint * cos_q_FR_hip_joint);
    (*this)(5,2) = ((((- tz_fr_FR_thigh_COM- ty_FR_thigh_joint) * cos_q_FR_hip_joint)- ty_FR_hip_joint) * sin_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_fr_FR_thigh_COM * sin_q_FR_hip_joint);
    (*this)(5,3) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(5,4) = sin_q_FR_hip_joint;
    (*this)(5,5) = cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    return *this;
}
MotionTransforms::Type_fr_FR_thigh_COM_X_fr_base::Type_fr_FR_thigh_COM_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(4,3) = 0.0;
}

const MotionTransforms::Type_fr_FR_thigh_COM_X_fr_base& MotionTransforms::Type_fr_FR_thigh_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,0) = cos_q_FR_thigh_joint;
    (*this)(0,1) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(0,2) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = sin_q_FR_hip_joint;
    (*this)(2,0) = sin_q_FR_thigh_joint;
    (*this)(2,1) = -sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(2,2) = cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(3,0) = ((- ty_FR_hip_joint * cos_q_FR_hip_joint)- tz_fr_FR_thigh_COM- ty_FR_thigh_joint) * sin_q_FR_thigh_joint;
    (*this)(3,1) = ( tx_FR_hip_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+(( tz_fr_FR_thigh_COM+ ty_FR_thigh_joint) * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( ty_fr_FR_thigh_COM * cos_q_FR_hip_joint);
    (*this)(3,2) = ( tx_FR_hip_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+((((- tz_fr_FR_thigh_COM- ty_FR_thigh_joint) * cos_q_FR_hip_joint)- ty_FR_hip_joint) * cos_q_FR_thigh_joint)-( ty_fr_FR_thigh_COM * sin_q_FR_hip_joint);
    (*this)(3,3) = cos_q_FR_thigh_joint;
    (*this)(3,4) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(3,5) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(4,0) = ( tx_fr_FR_thigh_COM * sin_q_FR_thigh_joint)+( ty_fr_FR_thigh_COM * cos_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(4,1) = ( ty_fr_FR_thigh_COM * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-( tx_fr_FR_thigh_COM * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(4,2) = (- ty_fr_FR_thigh_COM * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+( tx_fr_FR_thigh_COM * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)+( tx_FR_hip_joint * cos_q_FR_hip_joint);
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = sin_q_FR_hip_joint;
    (*this)(5,0) = (( ty_FR_hip_joint * cos_q_FR_hip_joint)+ tz_fr_FR_thigh_COM+ ty_FR_thigh_joint) * cos_q_FR_thigh_joint;
    (*this)(5,1) = (( tz_fr_FR_thigh_COM+ ty_FR_thigh_joint) * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-( tx_FR_hip_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_fr_FR_thigh_COM * cos_q_FR_hip_joint);
    (*this)(5,2) = ((((- tz_fr_FR_thigh_COM- ty_FR_thigh_joint) * cos_q_FR_hip_joint)- ty_FR_hip_joint) * sin_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_fr_FR_thigh_COM * sin_q_FR_hip_joint);
    (*this)(5,3) = sin_q_FR_thigh_joint;
    (*this)(5,4) = -sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(5,5) = cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FR_thigh_shoulder::Type_fr_base_X_fr_FR_thigh_shoulder()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FR_thigh_shoulder& MotionTransforms::Type_fr_base_X_fr_FR_thigh_shoulder::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = -sin_q_FR_hip_joint;
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,2) = cos_q_FR_hip_joint;
    (*this)(3,1) =  ty_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(3,2) = ( ty_FR_hip_joint * cos_q_FR_hip_joint)+ ty_fr_FR_thigh_shoulder;
    (*this)(4,0) =  ty_fr_FR_thigh_shoulder * sin_q_FR_hip_joint;
    (*this)(4,1) = - tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(4,2) = - tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = -sin_q_FR_hip_joint;
    (*this)(5,0) = (- ty_fr_FR_thigh_shoulder * cos_q_FR_hip_joint)- ty_FR_hip_joint;
    (*this)(5,1) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(5,2) = - tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(5,4) = sin_q_FR_hip_joint;
    (*this)(5,5) = cos_q_FR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_FR_thigh_shoulder_X_fr_base::Type_fr_FR_thigh_shoulder_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_FR_thigh_shoulder_X_fr_base& MotionTransforms::Type_fr_FR_thigh_shoulder_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = sin_q_FR_hip_joint;
    (*this)(2,1) = -sin_q_FR_hip_joint;
    (*this)(2,2) = cos_q_FR_hip_joint;
    (*this)(3,1) =  ty_fr_FR_thigh_shoulder * sin_q_FR_hip_joint;
    (*this)(3,2) = (- ty_fr_FR_thigh_shoulder * cos_q_FR_hip_joint)- ty_FR_hip_joint;
    (*this)(4,0) =  ty_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(4,1) = - tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(4,2) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = sin_q_FR_hip_joint;
    (*this)(5,0) = ( ty_FR_hip_joint * cos_q_FR_hip_joint)+ ty_fr_FR_thigh_shoulder;
    (*this)(5,1) = - tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(5,2) = - tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(5,4) = -sin_q_FR_hip_joint;
    (*this)(5,5) = cos_q_FR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RL_calf_COM::Type_fr_base_X_fr_RL_calf_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RL_calf_COM& MotionTransforms::Type_fr_base_X_fr_RL_calf_COM::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    (*this)(0,0) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(0,2) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(1,0) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,0) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,2) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(3,0) = (((- ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(3,1) = ((( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)-( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_fr_RL_calf_COM * sin_q_RL_calf_joint)+( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * cos_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(3,2) = (((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(3,3) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(3,5) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(4,0) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * cos_q_RL_calf_joint)- ty_fr_RL_calf_COM) * cos_q_RL_hip_joint);
    (*this)(4,1) = ((( tx_fr_RL_calf_COM * sin_q_RL_calf_joint)+( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+((( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)-( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)) * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(4,2) = ((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * sin_q_RL_calf_joint)- tx_fr_RL_calf_COM) * cos_q_RL_hip_joint);
    (*this)(4,3) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,0) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * cos_q_RL_calf_joint)- ty_fr_RL_calf_COM) * sin_q_RL_hip_joint);
    (*this)(5,1) = (((- tx_fr_RL_calf_COM * sin_q_RL_calf_joint)-( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)- ty_RL_calf_joint) * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+((( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)-( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)) * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)+( tx_RL_hip_joint * cos_q_RL_hip_joint);
    (*this)(5,2) = ((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * sin_q_RL_calf_joint)- tx_fr_RL_calf_COM) * sin_q_RL_hip_joint);
    (*this)(5,3) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,4) = sin_q_RL_hip_joint;
    (*this)(5,5) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    return *this;
}
MotionTransforms::Type_fr_RL_calf_COM_X_fr_base::Type_fr_RL_calf_COM_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(4,3) = 0.0;
}

const MotionTransforms::Type_fr_RL_calf_COM_X_fr_base& MotionTransforms::Type_fr_RL_calf_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,0) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(0,1) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(0,2) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = sin_q_RL_hip_joint;
    (*this)(2,0) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(2,1) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,2) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(3,0) = (((- ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(3,1) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * cos_q_RL_calf_joint)- ty_fr_RL_calf_COM) * cos_q_RL_hip_joint);
    (*this)(3,2) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * cos_q_RL_calf_joint)- ty_fr_RL_calf_COM) * sin_q_RL_hip_joint);
    (*this)(3,3) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(3,4) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(3,5) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(4,0) = ((( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)-( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_fr_RL_calf_COM * sin_q_RL_calf_joint)+( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * cos_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(4,1) = ((( tx_fr_RL_calf_COM * sin_q_RL_calf_joint)+( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+((( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)-( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)) * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(4,2) = (((- tx_fr_RL_calf_COM * sin_q_RL_calf_joint)-( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)- ty_RL_calf_joint) * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+((( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)-( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)) * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)+( tx_RL_hip_joint * cos_q_RL_hip_joint);
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = sin_q_RL_hip_joint;
    (*this)(5,0) = (((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(5,1) = ((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * sin_q_RL_calf_joint)- tx_fr_RL_calf_COM) * cos_q_RL_hip_joint);
    (*this)(5,2) = ((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * sin_q_RL_calf_joint)- tx_fr_RL_calf_COM) * sin_q_RL_hip_joint);
    (*this)(5,3) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(5,4) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,5) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RL_foot::Type_fr_base_X_fr_RL_foot()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RL_foot& MotionTransforms::Type_fr_base_X_fr_RL_foot::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    (*this)(0,0) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(0,2) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(1,0) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,0) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,2) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(3,0) = (((- ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(3,1) = (- ty_fr_RL_foot * sin_q_RL_calf_joint * sin_q_RL_thigh_joint)+((( ty_fr_RL_foot * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * cos_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(3,2) = (((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(3,3) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(3,5) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(4,0) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * cos_q_RL_calf_joint)- ty_fr_RL_foot) * cos_q_RL_hip_joint);
    (*this)(4,1) = ((( ty_fr_RL_foot * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+( ty_fr_RL_foot * sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(4,2) = ((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint);
    (*this)(4,3) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,0) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * cos_q_RL_calf_joint)- ty_fr_RL_foot) * sin_q_RL_hip_joint);
    (*this)(5,1) = (((- ty_fr_RL_foot * cos_q_RL_calf_joint)- ty_RL_calf_joint) * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-( ty_fr_RL_foot * sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)+( tx_RL_hip_joint * cos_q_RL_hip_joint);
    (*this)(5,2) = ((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint);
    (*this)(5,3) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,4) = sin_q_RL_hip_joint;
    (*this)(5,5) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    return *this;
}
MotionTransforms::Type_fr_RL_foot_X_fr_base::Type_fr_RL_foot_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(4,3) = 0.0;
}

const MotionTransforms::Type_fr_RL_foot_X_fr_base& MotionTransforms::Type_fr_RL_foot_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,0) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(0,1) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(0,2) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = sin_q_RL_hip_joint;
    (*this)(2,0) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(2,1) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,2) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(3,0) = (((- ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(3,1) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * cos_q_RL_calf_joint)- ty_fr_RL_foot) * cos_q_RL_hip_joint);
    (*this)(3,2) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * cos_q_RL_calf_joint)- ty_fr_RL_foot) * sin_q_RL_hip_joint);
    (*this)(3,3) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(3,4) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(3,5) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(4,0) = (- ty_fr_RL_foot * sin_q_RL_calf_joint * sin_q_RL_thigh_joint)+((( ty_fr_RL_foot * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * cos_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(4,1) = ((( ty_fr_RL_foot * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+( ty_fr_RL_foot * sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(4,2) = (((- ty_fr_RL_foot * cos_q_RL_calf_joint)- ty_RL_calf_joint) * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-( ty_fr_RL_foot * sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)+( tx_RL_hip_joint * cos_q_RL_hip_joint);
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = sin_q_RL_hip_joint;
    (*this)(5,0) = (((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(5,1) = ((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint);
    (*this)(5,2) = ((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint);
    (*this)(5,3) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(5,4) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,5) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RL_hip_COM::Type_fr_base_X_fr_RL_hip_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RL_hip_COM& MotionTransforms::Type_fr_base_X_fr_RL_hip_COM::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = -sin_q_RL_hip_joint;
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,2) = cos_q_RL_hip_joint;
    (*this)(3,1) = ( ty_RL_hip_joint * sin_q_RL_hip_joint)+ tx_fr_RL_hip_COM;
    (*this)(3,2) = ( ty_RL_hip_joint * cos_q_RL_hip_joint)+ ty_fr_RL_hip_COM;
    (*this)(4,0) = ( ty_fr_RL_hip_COM * sin_q_RL_hip_joint)-( tx_fr_RL_hip_COM * cos_q_RL_hip_joint);
    (*this)(4,1) = (- tz_fr_RL_hip_COM- tx_RL_hip_joint) * sin_q_RL_hip_joint;
    (*this)(4,2) = (- tz_fr_RL_hip_COM- tx_RL_hip_joint) * cos_q_RL_hip_joint;
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = -sin_q_RL_hip_joint;
    (*this)(5,0) = (- tx_fr_RL_hip_COM * sin_q_RL_hip_joint)-( ty_fr_RL_hip_COM * cos_q_RL_hip_joint)- ty_RL_hip_joint;
    (*this)(5,1) = ( tz_fr_RL_hip_COM+ tx_RL_hip_joint) * cos_q_RL_hip_joint;
    (*this)(5,2) = (- tz_fr_RL_hip_COM- tx_RL_hip_joint) * sin_q_RL_hip_joint;
    (*this)(5,4) = sin_q_RL_hip_joint;
    (*this)(5,5) = cos_q_RL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_RL_hip_COM_X_fr_base::Type_fr_RL_hip_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_RL_hip_COM_X_fr_base& MotionTransforms::Type_fr_RL_hip_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = sin_q_RL_hip_joint;
    (*this)(2,1) = -sin_q_RL_hip_joint;
    (*this)(2,2) = cos_q_RL_hip_joint;
    (*this)(3,1) = ( ty_fr_RL_hip_COM * sin_q_RL_hip_joint)-( tx_fr_RL_hip_COM * cos_q_RL_hip_joint);
    (*this)(3,2) = (- tx_fr_RL_hip_COM * sin_q_RL_hip_joint)-( ty_fr_RL_hip_COM * cos_q_RL_hip_joint)- ty_RL_hip_joint;
    (*this)(4,0) = ( ty_RL_hip_joint * sin_q_RL_hip_joint)+ tx_fr_RL_hip_COM;
    (*this)(4,1) = (- tz_fr_RL_hip_COM- tx_RL_hip_joint) * sin_q_RL_hip_joint;
    (*this)(4,2) = ( tz_fr_RL_hip_COM+ tx_RL_hip_joint) * cos_q_RL_hip_joint;
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = sin_q_RL_hip_joint;
    (*this)(5,0) = ( ty_RL_hip_joint * cos_q_RL_hip_joint)+ ty_fr_RL_hip_COM;
    (*this)(5,1) = (- tz_fr_RL_hip_COM- tx_RL_hip_joint) * cos_q_RL_hip_joint;
    (*this)(5,2) = (- tz_fr_RL_hip_COM- tx_RL_hip_joint) * sin_q_RL_hip_joint;
    (*this)(5,4) = -sin_q_RL_hip_joint;
    (*this)(5,5) = cos_q_RL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RL_thigh_COM::Type_fr_base_X_fr_RL_thigh_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RL_thigh_COM& MotionTransforms::Type_fr_base_X_fr_RL_thigh_COM::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    (*this)(0,0) = cos_q_RL_thigh_joint;
    (*this)(0,2) = sin_q_RL_thigh_joint;
    (*this)(1,0) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = -sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(2,0) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,2) = cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(3,0) = ((- ty_RL_hip_joint * cos_q_RL_hip_joint)- tz_fr_RL_thigh_COM- ty_RL_thigh_joint) * sin_q_RL_thigh_joint;
    (*this)(3,1) = ( tx_fr_RL_thigh_COM * sin_q_RL_thigh_joint)+( ty_fr_RL_thigh_COM * cos_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(3,2) = (( ty_RL_hip_joint * cos_q_RL_hip_joint)+ tz_fr_RL_thigh_COM+ ty_RL_thigh_joint) * cos_q_RL_thigh_joint;
    (*this)(3,3) = cos_q_RL_thigh_joint;
    (*this)(3,5) = sin_q_RL_thigh_joint;
    (*this)(4,0) = ( tx_RL_hip_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+(( tz_fr_RL_thigh_COM+ ty_RL_thigh_joint) * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( ty_fr_RL_thigh_COM * cos_q_RL_hip_joint);
    (*this)(4,1) = ( ty_fr_RL_thigh_COM * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-( tx_fr_RL_thigh_COM * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(4,2) = (( tz_fr_RL_thigh_COM+ ty_RL_thigh_joint) * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-( tx_RL_hip_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_fr_RL_thigh_COM * cos_q_RL_hip_joint);
    (*this)(4,3) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = -sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(5,0) = ( tx_RL_hip_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+((((- tz_fr_RL_thigh_COM- ty_RL_thigh_joint) * cos_q_RL_hip_joint)- ty_RL_hip_joint) * cos_q_RL_thigh_joint)-( ty_fr_RL_thigh_COM * sin_q_RL_hip_joint);
    (*this)(5,1) = (- ty_fr_RL_thigh_COM * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+( tx_fr_RL_thigh_COM * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)+( tx_RL_hip_joint * cos_q_RL_hip_joint);
    (*this)(5,2) = ((((- tz_fr_RL_thigh_COM- ty_RL_thigh_joint) * cos_q_RL_hip_joint)- ty_RL_hip_joint) * sin_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_fr_RL_thigh_COM * sin_q_RL_hip_joint);
    (*this)(5,3) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(5,4) = sin_q_RL_hip_joint;
    (*this)(5,5) = cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    return *this;
}
MotionTransforms::Type_fr_RL_thigh_COM_X_fr_base::Type_fr_RL_thigh_COM_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(4,3) = 0.0;
}

const MotionTransforms::Type_fr_RL_thigh_COM_X_fr_base& MotionTransforms::Type_fr_RL_thigh_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,0) = cos_q_RL_thigh_joint;
    (*this)(0,1) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(0,2) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = sin_q_RL_hip_joint;
    (*this)(2,0) = sin_q_RL_thigh_joint;
    (*this)(2,1) = -sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(2,2) = cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(3,0) = ((- ty_RL_hip_joint * cos_q_RL_hip_joint)- tz_fr_RL_thigh_COM- ty_RL_thigh_joint) * sin_q_RL_thigh_joint;
    (*this)(3,1) = ( tx_RL_hip_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+(( tz_fr_RL_thigh_COM+ ty_RL_thigh_joint) * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( ty_fr_RL_thigh_COM * cos_q_RL_hip_joint);
    (*this)(3,2) = ( tx_RL_hip_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+((((- tz_fr_RL_thigh_COM- ty_RL_thigh_joint) * cos_q_RL_hip_joint)- ty_RL_hip_joint) * cos_q_RL_thigh_joint)-( ty_fr_RL_thigh_COM * sin_q_RL_hip_joint);
    (*this)(3,3) = cos_q_RL_thigh_joint;
    (*this)(3,4) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(3,5) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(4,0) = ( tx_fr_RL_thigh_COM * sin_q_RL_thigh_joint)+( ty_fr_RL_thigh_COM * cos_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(4,1) = ( ty_fr_RL_thigh_COM * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-( tx_fr_RL_thigh_COM * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(4,2) = (- ty_fr_RL_thigh_COM * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+( tx_fr_RL_thigh_COM * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)+( tx_RL_hip_joint * cos_q_RL_hip_joint);
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = sin_q_RL_hip_joint;
    (*this)(5,0) = (( ty_RL_hip_joint * cos_q_RL_hip_joint)+ tz_fr_RL_thigh_COM+ ty_RL_thigh_joint) * cos_q_RL_thigh_joint;
    (*this)(5,1) = (( tz_fr_RL_thigh_COM+ ty_RL_thigh_joint) * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-( tx_RL_hip_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_fr_RL_thigh_COM * cos_q_RL_hip_joint);
    (*this)(5,2) = ((((- tz_fr_RL_thigh_COM- ty_RL_thigh_joint) * cos_q_RL_hip_joint)- ty_RL_hip_joint) * sin_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_fr_RL_thigh_COM * sin_q_RL_hip_joint);
    (*this)(5,3) = sin_q_RL_thigh_joint;
    (*this)(5,4) = -sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(5,5) = cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RL_thigh_shoulder::Type_fr_base_X_fr_RL_thigh_shoulder()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RL_thigh_shoulder& MotionTransforms::Type_fr_base_X_fr_RL_thigh_shoulder::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = -sin_q_RL_hip_joint;
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,2) = cos_q_RL_hip_joint;
    (*this)(3,1) =  ty_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(3,2) = ( ty_RL_hip_joint * cos_q_RL_hip_joint)+ ty_fr_RL_thigh_shoulder;
    (*this)(4,0) =  ty_fr_RL_thigh_shoulder * sin_q_RL_hip_joint;
    (*this)(4,1) = - tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(4,2) = - tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = -sin_q_RL_hip_joint;
    (*this)(5,0) = (- ty_fr_RL_thigh_shoulder * cos_q_RL_hip_joint)- ty_RL_hip_joint;
    (*this)(5,1) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(5,2) = - tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(5,4) = sin_q_RL_hip_joint;
    (*this)(5,5) = cos_q_RL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_RL_thigh_shoulder_X_fr_base::Type_fr_RL_thigh_shoulder_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_RL_thigh_shoulder_X_fr_base& MotionTransforms::Type_fr_RL_thigh_shoulder_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = sin_q_RL_hip_joint;
    (*this)(2,1) = -sin_q_RL_hip_joint;
    (*this)(2,2) = cos_q_RL_hip_joint;
    (*this)(3,1) =  ty_fr_RL_thigh_shoulder * sin_q_RL_hip_joint;
    (*this)(3,2) = (- ty_fr_RL_thigh_shoulder * cos_q_RL_hip_joint)- ty_RL_hip_joint;
    (*this)(4,0) =  ty_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(4,1) = - tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(4,2) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = sin_q_RL_hip_joint;
    (*this)(5,0) = ( ty_RL_hip_joint * cos_q_RL_hip_joint)+ ty_fr_RL_thigh_shoulder;
    (*this)(5,1) = - tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(5,2) = - tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(5,4) = -sin_q_RL_hip_joint;
    (*this)(5,5) = cos_q_RL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RR_calf_COM::Type_fr_base_X_fr_RR_calf_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RR_calf_COM& MotionTransforms::Type_fr_base_X_fr_RR_calf_COM::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    (*this)(0,0) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(0,2) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(1,0) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,0) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,2) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(3,0) = (((- ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(3,1) = ((( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)-( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_fr_RR_calf_COM * sin_q_RR_calf_joint)+( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * cos_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(3,2) = (((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(3,3) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(3,5) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(4,0) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * cos_q_RR_calf_joint)- ty_fr_RR_calf_COM) * cos_q_RR_hip_joint);
    (*this)(4,1) = ((( tx_fr_RR_calf_COM * sin_q_RR_calf_joint)+( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+((( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)-( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)) * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(4,2) = ((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * sin_q_RR_calf_joint)- tx_fr_RR_calf_COM) * cos_q_RR_hip_joint);
    (*this)(4,3) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,0) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * cos_q_RR_calf_joint)- ty_fr_RR_calf_COM) * sin_q_RR_hip_joint);
    (*this)(5,1) = (((- tx_fr_RR_calf_COM * sin_q_RR_calf_joint)-( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)- ty_RR_calf_joint) * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+((( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)-( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)) * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)+( tx_RR_hip_joint * cos_q_RR_hip_joint);
    (*this)(5,2) = ((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * sin_q_RR_calf_joint)- tx_fr_RR_calf_COM) * sin_q_RR_hip_joint);
    (*this)(5,3) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,4) = sin_q_RR_hip_joint;
    (*this)(5,5) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    return *this;
}
MotionTransforms::Type_fr_RR_calf_COM_X_fr_base::Type_fr_RR_calf_COM_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(4,3) = 0.0;
}

const MotionTransforms::Type_fr_RR_calf_COM_X_fr_base& MotionTransforms::Type_fr_RR_calf_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,0) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(0,1) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(0,2) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = sin_q_RR_hip_joint;
    (*this)(2,0) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(2,1) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,2) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(3,0) = (((- ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(3,1) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * cos_q_RR_calf_joint)- ty_fr_RR_calf_COM) * cos_q_RR_hip_joint);
    (*this)(3,2) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * cos_q_RR_calf_joint)- ty_fr_RR_calf_COM) * sin_q_RR_hip_joint);
    (*this)(3,3) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(3,4) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(3,5) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(4,0) = ((( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)-( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_fr_RR_calf_COM * sin_q_RR_calf_joint)+( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * cos_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(4,1) = ((( tx_fr_RR_calf_COM * sin_q_RR_calf_joint)+( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+((( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)-( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)) * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(4,2) = (((- tx_fr_RR_calf_COM * sin_q_RR_calf_joint)-( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)- ty_RR_calf_joint) * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+((( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)-( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)) * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)+( tx_RR_hip_joint * cos_q_RR_hip_joint);
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = sin_q_RR_hip_joint;
    (*this)(5,0) = (((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(5,1) = ((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * sin_q_RR_calf_joint)- tx_fr_RR_calf_COM) * cos_q_RR_hip_joint);
    (*this)(5,2) = ((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * sin_q_RR_calf_joint)- tx_fr_RR_calf_COM) * sin_q_RR_hip_joint);
    (*this)(5,3) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(5,4) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,5) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RR_foot::Type_fr_base_X_fr_RR_foot()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RR_foot& MotionTransforms::Type_fr_base_X_fr_RR_foot::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    (*this)(0,0) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(0,2) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(1,0) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,0) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,2) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(3,0) = (((- ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(3,1) = (- ty_fr_RR_foot * sin_q_RR_calf_joint * sin_q_RR_thigh_joint)+((( ty_fr_RR_foot * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * cos_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(3,2) = (((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(3,3) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(3,5) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(4,0) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * cos_q_RR_calf_joint)- ty_fr_RR_foot) * cos_q_RR_hip_joint);
    (*this)(4,1) = ((( ty_fr_RR_foot * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+( ty_fr_RR_foot * sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(4,2) = ((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint);
    (*this)(4,3) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,0) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * cos_q_RR_calf_joint)- ty_fr_RR_foot) * sin_q_RR_hip_joint);
    (*this)(5,1) = (((- ty_fr_RR_foot * cos_q_RR_calf_joint)- ty_RR_calf_joint) * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-( ty_fr_RR_foot * sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)+( tx_RR_hip_joint * cos_q_RR_hip_joint);
    (*this)(5,2) = ((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint);
    (*this)(5,3) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,4) = sin_q_RR_hip_joint;
    (*this)(5,5) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    return *this;
}
MotionTransforms::Type_fr_RR_foot_X_fr_base::Type_fr_RR_foot_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(4,3) = 0.0;
}

const MotionTransforms::Type_fr_RR_foot_X_fr_base& MotionTransforms::Type_fr_RR_foot_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,0) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(0,1) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(0,2) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = sin_q_RR_hip_joint;
    (*this)(2,0) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(2,1) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,2) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(3,0) = (((- ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(3,1) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * cos_q_RR_calf_joint)- ty_fr_RR_foot) * cos_q_RR_hip_joint);
    (*this)(3,2) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * cos_q_RR_calf_joint)- ty_fr_RR_foot) * sin_q_RR_hip_joint);
    (*this)(3,3) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(3,4) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(3,5) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(4,0) = (- ty_fr_RR_foot * sin_q_RR_calf_joint * sin_q_RR_thigh_joint)+((( ty_fr_RR_foot * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * cos_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(4,1) = ((( ty_fr_RR_foot * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+( ty_fr_RR_foot * sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(4,2) = (((- ty_fr_RR_foot * cos_q_RR_calf_joint)- ty_RR_calf_joint) * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-( ty_fr_RR_foot * sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)+( tx_RR_hip_joint * cos_q_RR_hip_joint);
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = sin_q_RR_hip_joint;
    (*this)(5,0) = (((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(5,1) = ((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint);
    (*this)(5,2) = ((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint);
    (*this)(5,3) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(5,4) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,5) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RR_hip_COM::Type_fr_base_X_fr_RR_hip_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RR_hip_COM& MotionTransforms::Type_fr_base_X_fr_RR_hip_COM::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = -sin_q_RR_hip_joint;
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,2) = cos_q_RR_hip_joint;
    (*this)(3,1) = ( ty_RR_hip_joint * sin_q_RR_hip_joint)+ tx_fr_RR_hip_COM;
    (*this)(3,2) = ( ty_RR_hip_joint * cos_q_RR_hip_joint)+ ty_fr_RR_hip_COM;
    (*this)(4,0) = ( ty_fr_RR_hip_COM * sin_q_RR_hip_joint)-( tx_fr_RR_hip_COM * cos_q_RR_hip_joint);
    (*this)(4,1) = (- tz_fr_RR_hip_COM- tx_RR_hip_joint) * sin_q_RR_hip_joint;
    (*this)(4,2) = (- tz_fr_RR_hip_COM- tx_RR_hip_joint) * cos_q_RR_hip_joint;
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = -sin_q_RR_hip_joint;
    (*this)(5,0) = (- tx_fr_RR_hip_COM * sin_q_RR_hip_joint)-( ty_fr_RR_hip_COM * cos_q_RR_hip_joint)- ty_RR_hip_joint;
    (*this)(5,1) = ( tz_fr_RR_hip_COM+ tx_RR_hip_joint) * cos_q_RR_hip_joint;
    (*this)(5,2) = (- tz_fr_RR_hip_COM- tx_RR_hip_joint) * sin_q_RR_hip_joint;
    (*this)(5,4) = sin_q_RR_hip_joint;
    (*this)(5,5) = cos_q_RR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_RR_hip_COM_X_fr_base::Type_fr_RR_hip_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_RR_hip_COM_X_fr_base& MotionTransforms::Type_fr_RR_hip_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = sin_q_RR_hip_joint;
    (*this)(2,1) = -sin_q_RR_hip_joint;
    (*this)(2,2) = cos_q_RR_hip_joint;
    (*this)(3,1) = ( ty_fr_RR_hip_COM * sin_q_RR_hip_joint)-( tx_fr_RR_hip_COM * cos_q_RR_hip_joint);
    (*this)(3,2) = (- tx_fr_RR_hip_COM * sin_q_RR_hip_joint)-( ty_fr_RR_hip_COM * cos_q_RR_hip_joint)- ty_RR_hip_joint;
    (*this)(4,0) = ( ty_RR_hip_joint * sin_q_RR_hip_joint)+ tx_fr_RR_hip_COM;
    (*this)(4,1) = (- tz_fr_RR_hip_COM- tx_RR_hip_joint) * sin_q_RR_hip_joint;
    (*this)(4,2) = ( tz_fr_RR_hip_COM+ tx_RR_hip_joint) * cos_q_RR_hip_joint;
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = sin_q_RR_hip_joint;
    (*this)(5,0) = ( ty_RR_hip_joint * cos_q_RR_hip_joint)+ ty_fr_RR_hip_COM;
    (*this)(5,1) = (- tz_fr_RR_hip_COM- tx_RR_hip_joint) * cos_q_RR_hip_joint;
    (*this)(5,2) = (- tz_fr_RR_hip_COM- tx_RR_hip_joint) * sin_q_RR_hip_joint;
    (*this)(5,4) = -sin_q_RR_hip_joint;
    (*this)(5,5) = cos_q_RR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RR_thigh_COM::Type_fr_base_X_fr_RR_thigh_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RR_thigh_COM& MotionTransforms::Type_fr_base_X_fr_RR_thigh_COM::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    (*this)(0,0) = cos_q_RR_thigh_joint;
    (*this)(0,2) = sin_q_RR_thigh_joint;
    (*this)(1,0) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = -sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(2,0) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,2) = cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(3,0) = ((- ty_RR_hip_joint * cos_q_RR_hip_joint)- tz_fr_RR_thigh_COM- ty_RR_thigh_joint) * sin_q_RR_thigh_joint;
    (*this)(3,1) = ( tx_fr_RR_thigh_COM * sin_q_RR_thigh_joint)+( ty_fr_RR_thigh_COM * cos_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(3,2) = (( ty_RR_hip_joint * cos_q_RR_hip_joint)+ tz_fr_RR_thigh_COM+ ty_RR_thigh_joint) * cos_q_RR_thigh_joint;
    (*this)(3,3) = cos_q_RR_thigh_joint;
    (*this)(3,5) = sin_q_RR_thigh_joint;
    (*this)(4,0) = ( tx_RR_hip_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+(( tz_fr_RR_thigh_COM+ ty_RR_thigh_joint) * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( ty_fr_RR_thigh_COM * cos_q_RR_hip_joint);
    (*this)(4,1) = ( ty_fr_RR_thigh_COM * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-( tx_fr_RR_thigh_COM * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(4,2) = (( tz_fr_RR_thigh_COM+ ty_RR_thigh_joint) * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-( tx_RR_hip_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_fr_RR_thigh_COM * cos_q_RR_hip_joint);
    (*this)(4,3) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = -sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(5,0) = ( tx_RR_hip_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+((((- tz_fr_RR_thigh_COM- ty_RR_thigh_joint) * cos_q_RR_hip_joint)- ty_RR_hip_joint) * cos_q_RR_thigh_joint)-( ty_fr_RR_thigh_COM * sin_q_RR_hip_joint);
    (*this)(5,1) = (- ty_fr_RR_thigh_COM * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+( tx_fr_RR_thigh_COM * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)+( tx_RR_hip_joint * cos_q_RR_hip_joint);
    (*this)(5,2) = ((((- tz_fr_RR_thigh_COM- ty_RR_thigh_joint) * cos_q_RR_hip_joint)- ty_RR_hip_joint) * sin_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_fr_RR_thigh_COM * sin_q_RR_hip_joint);
    (*this)(5,3) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(5,4) = sin_q_RR_hip_joint;
    (*this)(5,5) = cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    return *this;
}
MotionTransforms::Type_fr_RR_thigh_COM_X_fr_base::Type_fr_RR_thigh_COM_X_fr_base()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(4,3) = 0.0;
}

const MotionTransforms::Type_fr_RR_thigh_COM_X_fr_base& MotionTransforms::Type_fr_RR_thigh_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,0) = cos_q_RR_thigh_joint;
    (*this)(0,1) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(0,2) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = sin_q_RR_hip_joint;
    (*this)(2,0) = sin_q_RR_thigh_joint;
    (*this)(2,1) = -sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(2,2) = cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(3,0) = ((- ty_RR_hip_joint * cos_q_RR_hip_joint)- tz_fr_RR_thigh_COM- ty_RR_thigh_joint) * sin_q_RR_thigh_joint;
    (*this)(3,1) = ( tx_RR_hip_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+(( tz_fr_RR_thigh_COM+ ty_RR_thigh_joint) * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( ty_fr_RR_thigh_COM * cos_q_RR_hip_joint);
    (*this)(3,2) = ( tx_RR_hip_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+((((- tz_fr_RR_thigh_COM- ty_RR_thigh_joint) * cos_q_RR_hip_joint)- ty_RR_hip_joint) * cos_q_RR_thigh_joint)-( ty_fr_RR_thigh_COM * sin_q_RR_hip_joint);
    (*this)(3,3) = cos_q_RR_thigh_joint;
    (*this)(3,4) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(3,5) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(4,0) = ( tx_fr_RR_thigh_COM * sin_q_RR_thigh_joint)+( ty_fr_RR_thigh_COM * cos_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(4,1) = ( ty_fr_RR_thigh_COM * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-( tx_fr_RR_thigh_COM * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(4,2) = (- ty_fr_RR_thigh_COM * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+( tx_fr_RR_thigh_COM * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)+( tx_RR_hip_joint * cos_q_RR_hip_joint);
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = sin_q_RR_hip_joint;
    (*this)(5,0) = (( ty_RR_hip_joint * cos_q_RR_hip_joint)+ tz_fr_RR_thigh_COM+ ty_RR_thigh_joint) * cos_q_RR_thigh_joint;
    (*this)(5,1) = (( tz_fr_RR_thigh_COM+ ty_RR_thigh_joint) * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-( tx_RR_hip_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_fr_RR_thigh_COM * cos_q_RR_hip_joint);
    (*this)(5,2) = ((((- tz_fr_RR_thigh_COM- ty_RR_thigh_joint) * cos_q_RR_hip_joint)- ty_RR_hip_joint) * sin_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_fr_RR_thigh_COM * sin_q_RR_hip_joint);
    (*this)(5,3) = sin_q_RR_thigh_joint;
    (*this)(5,4) = -sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(5,5) = cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RR_thigh_shoulder::Type_fr_base_X_fr_RR_thigh_shoulder()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RR_thigh_shoulder& MotionTransforms::Type_fr_base_X_fr_RR_thigh_shoulder::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = -sin_q_RR_hip_joint;
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,2) = cos_q_RR_hip_joint;
    (*this)(3,1) =  ty_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(3,2) = ( ty_RR_hip_joint * cos_q_RR_hip_joint)+ ty_fr_RR_thigh_shoulder;
    (*this)(4,0) =  ty_fr_RR_thigh_shoulder * sin_q_RR_hip_joint;
    (*this)(4,1) = - tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(4,2) = - tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = -sin_q_RR_hip_joint;
    (*this)(5,0) = (- ty_fr_RR_thigh_shoulder * cos_q_RR_hip_joint)- ty_RR_hip_joint;
    (*this)(5,1) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(5,2) = - tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(5,4) = sin_q_RR_hip_joint;
    (*this)(5,5) = cos_q_RR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_RR_thigh_shoulder_X_fr_base::Type_fr_RR_thigh_shoulder_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_RR_thigh_shoulder_X_fr_base& MotionTransforms::Type_fr_RR_thigh_shoulder_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = sin_q_RR_hip_joint;
    (*this)(2,1) = -sin_q_RR_hip_joint;
    (*this)(2,2) = cos_q_RR_hip_joint;
    (*this)(3,1) =  ty_fr_RR_thigh_shoulder * sin_q_RR_hip_joint;
    (*this)(3,2) = (- ty_fr_RR_thigh_shoulder * cos_q_RR_hip_joint)- ty_RR_hip_joint;
    (*this)(4,0) =  ty_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(4,1) = - tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(4,2) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = sin_q_RR_hip_joint;
    (*this)(5,0) = ( ty_RR_hip_joint * cos_q_RR_hip_joint)+ ty_fr_RR_thigh_shoulder;
    (*this)(5,1) = - tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(5,2) = - tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(5,4) = -sin_q_RR_hip_joint;
    (*this)(5,5) = cos_q_RR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_base_COM::Type_fr_base_X_fr_base_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = - tz_fr_base_COM;    // Maxima DSL: -_k__tz_fr_base_COM
    (*this)(3,2) =  ty_fr_base_COM;    // Maxima DSL: _k__ty_fr_base_COM
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) =  tz_fr_base_COM;    // Maxima DSL: _k__tz_fr_base_COM
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = - ty_fr_base_COM;    // Maxima DSL: -_k__ty_fr_base_COM
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_X_fr_base_COM& MotionTransforms::Type_fr_base_X_fr_base_COM::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_COM_X_fr_base::Type_fr_base_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) =  tz_fr_base_COM;    // Maxima DSL: _k__tz_fr_base_COM
    (*this)(3,2) = - ty_fr_base_COM;    // Maxima DSL: -_k__ty_fr_base_COM
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = - tz_fr_base_COM;    // Maxima DSL: -_k__tz_fr_base_COM
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) =  ty_fr_base_COM;    // Maxima DSL: _k__ty_fr_base_COM
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_COM_X_fr_base& MotionTransforms::Type_fr_base_COM_X_fr_base::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_imu_link::Type_fr_base_X_fr_imu_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_X_fr_imu_link& MotionTransforms::Type_fr_base_X_fr_imu_link::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_imu_link_X_fr_base::Type_fr_imu_link_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_imu_link_X_fr_base& MotionTransforms::Type_fr_imu_link_X_fr_base::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_trunk::Type_fr_base_X_fr_trunk()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_X_fr_trunk& MotionTransforms::Type_fr_base_X_fr_trunk::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_trunk_X_fr_base::Type_fr_trunk_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_trunk_X_fr_base& MotionTransforms::Type_fr_trunk_X_fr_base::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FL_hip_joint::Type_fr_base_X_fr_FL_hip_joint()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = - ty_FL_hip_joint;    // Maxima DSL: -_k__ty_FL_hip_joint
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) =  tx_FL_hip_joint;    // Maxima DSL: _k__tx_FL_hip_joint
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) =  tx_FL_hip_joint;    // Maxima DSL: _k__tx_FL_hip_joint
    (*this)(5,2) = - ty_FL_hip_joint;    // Maxima DSL: -_k__ty_FL_hip_joint
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FL_hip_joint& MotionTransforms::Type_fr_base_X_fr_FL_hip_joint::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FL_thigh_joint::Type_fr_base_X_fr_FL_thigh_joint()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FL_thigh_joint& MotionTransforms::Type_fr_base_X_fr_FL_thigh_joint::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(1,1) = sin_q_FL_hip_joint;
    (*this)(1,2) = cos_q_FL_hip_joint;
    (*this)(2,1) = -cos_q_FL_hip_joint;
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(3,1) = (- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_FL_thigh_joint;
    (*this)(3,2) =  ty_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(4,0) =  ty_FL_thigh_joint * sin_q_FL_hip_joint;
    (*this)(4,1) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(4,2) = - tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(4,4) = sin_q_FL_hip_joint;
    (*this)(4,5) = cos_q_FL_hip_joint;
    (*this)(5,0) = (- ty_FL_thigh_joint * cos_q_FL_hip_joint)- ty_FL_hip_joint;
    (*this)(5,1) =  tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(5,2) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(5,4) = -cos_q_FL_hip_joint;
    (*this)(5,5) = sin_q_FL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FL_calf_joint::Type_fr_base_X_fr_FL_calf_joint()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FL_calf_joint& MotionTransforms::Type_fr_base_X_fr_FL_calf_joint::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    (*this)(0,0) = cos_q_FL_thigh_joint;
    (*this)(0,1) = -sin_q_FL_thigh_joint;
    (*this)(1,0) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(1,1) = sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(1,2) = cos_q_FL_hip_joint;
    (*this)(2,0) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(2,1) = -cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(3,0) = ((- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_FL_thigh_joint) * sin_q_FL_thigh_joint;
    (*this)(3,1) = ((- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_FL_thigh_joint) * cos_q_FL_thigh_joint;
    (*this)(3,2) = ( ty_FL_calf_joint * cos_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(3,3) = cos_q_FL_thigh_joint;
    (*this)(3,4) = -sin_q_FL_thigh_joint;
    (*this)(4,0) = ( tx_FL_hip_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+( ty_FL_thigh_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * cos_q_FL_hip_joint);
    (*this)(4,1) = ( tx_FL_hip_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-( ty_FL_thigh_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(4,2) = ( ty_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(4,3) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(4,4) = sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(4,5) = cos_q_FL_hip_joint;
    (*this)(5,0) = ( tx_FL_hip_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(((- ty_FL_thigh_joint * cos_q_FL_hip_joint)- ty_FL_hip_joint) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * sin_q_FL_hip_joint);
    (*this)(5,1) = ((( ty_FL_thigh_joint * cos_q_FL_hip_joint)+ ty_FL_hip_joint) * sin_q_FL_thigh_joint)+( tx_FL_hip_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,2) = ( tx_FL_hip_joint * cos_q_FL_hip_joint)-( ty_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(5,3) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(5,4) = -cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(5,5) = sin_q_FL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FR_hip_joint::Type_fr_base_X_fr_FR_hip_joint()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = - ty_FR_hip_joint;    // Maxima DSL: -_k__ty_FR_hip_joint
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) =  tx_FR_hip_joint;    // Maxima DSL: _k__tx_FR_hip_joint
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) =  tx_FR_hip_joint;    // Maxima DSL: _k__tx_FR_hip_joint
    (*this)(5,2) = - ty_FR_hip_joint;    // Maxima DSL: -_k__ty_FR_hip_joint
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FR_hip_joint& MotionTransforms::Type_fr_base_X_fr_FR_hip_joint::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FR_thigh_joint::Type_fr_base_X_fr_FR_thigh_joint()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FR_thigh_joint& MotionTransforms::Type_fr_base_X_fr_FR_thigh_joint::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(1,1) = sin_q_FR_hip_joint;
    (*this)(1,2) = cos_q_FR_hip_joint;
    (*this)(2,1) = -cos_q_FR_hip_joint;
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(3,1) = (- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_FR_thigh_joint;
    (*this)(3,2) =  ty_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(4,0) =  ty_FR_thigh_joint * sin_q_FR_hip_joint;
    (*this)(4,1) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(4,2) = - tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(4,4) = sin_q_FR_hip_joint;
    (*this)(4,5) = cos_q_FR_hip_joint;
    (*this)(5,0) = (- ty_FR_thigh_joint * cos_q_FR_hip_joint)- ty_FR_hip_joint;
    (*this)(5,1) =  tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(5,2) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(5,4) = -cos_q_FR_hip_joint;
    (*this)(5,5) = sin_q_FR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_FR_calf_joint::Type_fr_base_X_fr_FR_calf_joint()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_FR_calf_joint& MotionTransforms::Type_fr_base_X_fr_FR_calf_joint::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    (*this)(0,0) = cos_q_FR_thigh_joint;
    (*this)(0,1) = -sin_q_FR_thigh_joint;
    (*this)(1,0) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(1,1) = sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(1,2) = cos_q_FR_hip_joint;
    (*this)(2,0) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(2,1) = -cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(3,0) = ((- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_FR_thigh_joint) * sin_q_FR_thigh_joint;
    (*this)(3,1) = ((- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_FR_thigh_joint) * cos_q_FR_thigh_joint;
    (*this)(3,2) = ( ty_FR_calf_joint * cos_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(3,3) = cos_q_FR_thigh_joint;
    (*this)(3,4) = -sin_q_FR_thigh_joint;
    (*this)(4,0) = ( tx_FR_hip_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+( ty_FR_thigh_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * cos_q_FR_hip_joint);
    (*this)(4,1) = ( tx_FR_hip_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-( ty_FR_thigh_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(4,2) = ( ty_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(4,3) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(4,4) = sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(4,5) = cos_q_FR_hip_joint;
    (*this)(5,0) = ( tx_FR_hip_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(((- ty_FR_thigh_joint * cos_q_FR_hip_joint)- ty_FR_hip_joint) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * sin_q_FR_hip_joint);
    (*this)(5,1) = ((( ty_FR_thigh_joint * cos_q_FR_hip_joint)+ ty_FR_hip_joint) * sin_q_FR_thigh_joint)+( tx_FR_hip_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,2) = ( tx_FR_hip_joint * cos_q_FR_hip_joint)-( ty_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(5,3) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(5,4) = -cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(5,5) = sin_q_FR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RL_hip_joint::Type_fr_base_X_fr_RL_hip_joint()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = - ty_RL_hip_joint;    // Maxima DSL: -_k__ty_RL_hip_joint
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) =  tx_RL_hip_joint;    // Maxima DSL: _k__tx_RL_hip_joint
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) =  tx_RL_hip_joint;    // Maxima DSL: _k__tx_RL_hip_joint
    (*this)(5,2) = - ty_RL_hip_joint;    // Maxima DSL: -_k__ty_RL_hip_joint
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RL_hip_joint& MotionTransforms::Type_fr_base_X_fr_RL_hip_joint::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RL_thigh_joint::Type_fr_base_X_fr_RL_thigh_joint()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RL_thigh_joint& MotionTransforms::Type_fr_base_X_fr_RL_thigh_joint::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(1,1) = sin_q_RL_hip_joint;
    (*this)(1,2) = cos_q_RL_hip_joint;
    (*this)(2,1) = -cos_q_RL_hip_joint;
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(3,1) = (- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_RL_thigh_joint;
    (*this)(3,2) =  ty_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(4,0) =  ty_RL_thigh_joint * sin_q_RL_hip_joint;
    (*this)(4,1) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(4,2) = - tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(4,4) = sin_q_RL_hip_joint;
    (*this)(4,5) = cos_q_RL_hip_joint;
    (*this)(5,0) = (- ty_RL_thigh_joint * cos_q_RL_hip_joint)- ty_RL_hip_joint;
    (*this)(5,1) =  tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(5,2) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(5,4) = -cos_q_RL_hip_joint;
    (*this)(5,5) = sin_q_RL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RL_calf_joint::Type_fr_base_X_fr_RL_calf_joint()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RL_calf_joint& MotionTransforms::Type_fr_base_X_fr_RL_calf_joint::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    (*this)(0,0) = cos_q_RL_thigh_joint;
    (*this)(0,1) = -sin_q_RL_thigh_joint;
    (*this)(1,0) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(1,1) = sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(1,2) = cos_q_RL_hip_joint;
    (*this)(2,0) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(2,1) = -cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(3,0) = ((- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_RL_thigh_joint) * sin_q_RL_thigh_joint;
    (*this)(3,1) = ((- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_RL_thigh_joint) * cos_q_RL_thigh_joint;
    (*this)(3,2) = ( ty_RL_calf_joint * cos_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(3,3) = cos_q_RL_thigh_joint;
    (*this)(3,4) = -sin_q_RL_thigh_joint;
    (*this)(4,0) = ( tx_RL_hip_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+( ty_RL_thigh_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * cos_q_RL_hip_joint);
    (*this)(4,1) = ( tx_RL_hip_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-( ty_RL_thigh_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(4,2) = ( ty_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(4,3) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(4,4) = sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(4,5) = cos_q_RL_hip_joint;
    (*this)(5,0) = ( tx_RL_hip_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(((- ty_RL_thigh_joint * cos_q_RL_hip_joint)- ty_RL_hip_joint) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * sin_q_RL_hip_joint);
    (*this)(5,1) = ((( ty_RL_thigh_joint * cos_q_RL_hip_joint)+ ty_RL_hip_joint) * sin_q_RL_thigh_joint)+( tx_RL_hip_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,2) = ( tx_RL_hip_joint * cos_q_RL_hip_joint)-( ty_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(5,3) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(5,4) = -cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(5,5) = sin_q_RL_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RR_hip_joint::Type_fr_base_X_fr_RR_hip_joint()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = - ty_RR_hip_joint;    // Maxima DSL: -_k__ty_RR_hip_joint
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) =  tx_RR_hip_joint;    // Maxima DSL: _k__tx_RR_hip_joint
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) =  tx_RR_hip_joint;    // Maxima DSL: _k__tx_RR_hip_joint
    (*this)(5,2) = - ty_RR_hip_joint;    // Maxima DSL: -_k__ty_RR_hip_joint
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RR_hip_joint& MotionTransforms::Type_fr_base_X_fr_RR_hip_joint::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RR_thigh_joint::Type_fr_base_X_fr_RR_thigh_joint()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,3) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RR_thigh_joint& MotionTransforms::Type_fr_base_X_fr_RR_thigh_joint::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(1,1) = sin_q_RR_hip_joint;
    (*this)(1,2) = cos_q_RR_hip_joint;
    (*this)(2,1) = -cos_q_RR_hip_joint;
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(3,1) = (- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_RR_thigh_joint;
    (*this)(3,2) =  ty_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(4,0) =  ty_RR_thigh_joint * sin_q_RR_hip_joint;
    (*this)(4,1) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(4,2) = - tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(4,4) = sin_q_RR_hip_joint;
    (*this)(4,5) = cos_q_RR_hip_joint;
    (*this)(5,0) = (- ty_RR_thigh_joint * cos_q_RR_hip_joint)- ty_RR_hip_joint;
    (*this)(5,1) =  tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(5,2) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(5,4) = -cos_q_RR_hip_joint;
    (*this)(5,5) = sin_q_RR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_base_X_fr_RR_calf_joint::Type_fr_base_X_fr_RR_calf_joint()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
}

const MotionTransforms::Type_fr_base_X_fr_RR_calf_joint& MotionTransforms::Type_fr_base_X_fr_RR_calf_joint::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    (*this)(0,0) = cos_q_RR_thigh_joint;
    (*this)(0,1) = -sin_q_RR_thigh_joint;
    (*this)(1,0) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(1,1) = sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(1,2) = cos_q_RR_hip_joint;
    (*this)(2,0) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(2,1) = -cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(3,0) = ((- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_RR_thigh_joint) * sin_q_RR_thigh_joint;
    (*this)(3,1) = ((- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_RR_thigh_joint) * cos_q_RR_thigh_joint;
    (*this)(3,2) = ( ty_RR_calf_joint * cos_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(3,3) = cos_q_RR_thigh_joint;
    (*this)(3,4) = -sin_q_RR_thigh_joint;
    (*this)(4,0) = ( tx_RR_hip_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+( ty_RR_thigh_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * cos_q_RR_hip_joint);
    (*this)(4,1) = ( tx_RR_hip_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-( ty_RR_thigh_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(4,2) = ( ty_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(4,3) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(4,4) = sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(4,5) = cos_q_RR_hip_joint;
    (*this)(5,0) = ( tx_RR_hip_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(((- ty_RR_thigh_joint * cos_q_RR_hip_joint)- ty_RR_hip_joint) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * sin_q_RR_hip_joint);
    (*this)(5,1) = ((( ty_RR_thigh_joint * cos_q_RR_hip_joint)+ ty_RR_hip_joint) * sin_q_RR_thigh_joint)+( tx_RR_hip_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,2) = ( tx_RR_hip_joint * cos_q_RR_hip_joint)-( ty_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(5,3) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(5,4) = -cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(5,5) = sin_q_RR_hip_joint;
    return *this;
}
MotionTransforms::Type_fr_FR_thigh_X_fr_FR_hip::Type_fr_FR_thigh_X_fr_FR_hip()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_FR_thigh_X_fr_FR_hip& MotionTransforms::Type_fr_FR_thigh_X_fr_FR_hip::update(const state_t& q)
{
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    (*this)(0,0) = sin_q_FR_thigh_joint;
    (*this)(0,2) = cos_q_FR_thigh_joint;
    (*this)(1,0) = cos_q_FR_thigh_joint;
    (*this)(1,2) = -sin_q_FR_thigh_joint;
    (*this)(3,0) =  ty_FR_thigh_joint * cos_q_FR_thigh_joint;
    (*this)(3,2) = - ty_FR_thigh_joint * sin_q_FR_thigh_joint;
    (*this)(3,3) = sin_q_FR_thigh_joint;
    (*this)(3,5) = cos_q_FR_thigh_joint;
    (*this)(4,0) = - ty_FR_thigh_joint * sin_q_FR_thigh_joint;
    (*this)(4,2) = - ty_FR_thigh_joint * cos_q_FR_thigh_joint;
    (*this)(4,3) = cos_q_FR_thigh_joint;
    (*this)(4,5) = -sin_q_FR_thigh_joint;
    return *this;
}
MotionTransforms::Type_fr_FR_hip_X_fr_FR_thigh::Type_fr_FR_hip_X_fr_FR_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_FR_hip_X_fr_FR_thigh& MotionTransforms::Type_fr_FR_hip_X_fr_FR_thigh::update(const state_t& q)
{
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    (*this)(0,0) = sin_q_FR_thigh_joint;
    (*this)(0,1) = cos_q_FR_thigh_joint;
    (*this)(2,0) = cos_q_FR_thigh_joint;
    (*this)(2,1) = -sin_q_FR_thigh_joint;
    (*this)(3,0) =  ty_FR_thigh_joint * cos_q_FR_thigh_joint;
    (*this)(3,1) = - ty_FR_thigh_joint * sin_q_FR_thigh_joint;
    (*this)(3,3) = sin_q_FR_thigh_joint;
    (*this)(3,4) = cos_q_FR_thigh_joint;
    (*this)(5,0) = - ty_FR_thigh_joint * sin_q_FR_thigh_joint;
    (*this)(5,1) = - ty_FR_thigh_joint * cos_q_FR_thigh_joint;
    (*this)(5,3) = cos_q_FR_thigh_joint;
    (*this)(5,4) = -sin_q_FR_thigh_joint;
    return *this;
}
MotionTransforms::Type_fr_FR_calf_X_fr_FR_thigh::Type_fr_FR_calf_X_fr_FR_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) =  ty_FR_calf_joint;    // Maxima DSL: _k__ty_FR_calf_joint
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_FR_calf_X_fr_FR_thigh& MotionTransforms::Type_fr_FR_calf_X_fr_FR_thigh::update(const state_t& q)
{
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    (*this)(0,0) = cos_q_FR_calf_joint;
    (*this)(0,1) = sin_q_FR_calf_joint;
    (*this)(1,0) = -sin_q_FR_calf_joint;
    (*this)(1,1) = cos_q_FR_calf_joint;
    (*this)(3,2) = - ty_FR_calf_joint * cos_q_FR_calf_joint;
    (*this)(3,3) = cos_q_FR_calf_joint;
    (*this)(3,4) = sin_q_FR_calf_joint;
    (*this)(4,2) =  ty_FR_calf_joint * sin_q_FR_calf_joint;
    (*this)(4,3) = -sin_q_FR_calf_joint;
    (*this)(4,4) = cos_q_FR_calf_joint;
    return *this;
}
MotionTransforms::Type_fr_FR_thigh_X_fr_FR_calf::Type_fr_FR_thigh_X_fr_FR_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) =  ty_FR_calf_joint;    // Maxima DSL: _k__ty_FR_calf_joint
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_FR_thigh_X_fr_FR_calf& MotionTransforms::Type_fr_FR_thigh_X_fr_FR_calf::update(const state_t& q)
{
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    (*this)(0,0) = cos_q_FR_calf_joint;
    (*this)(0,1) = -sin_q_FR_calf_joint;
    (*this)(1,0) = sin_q_FR_calf_joint;
    (*this)(1,1) = cos_q_FR_calf_joint;
    (*this)(3,3) = cos_q_FR_calf_joint;
    (*this)(3,4) = -sin_q_FR_calf_joint;
    (*this)(4,3) = sin_q_FR_calf_joint;
    (*this)(4,4) = cos_q_FR_calf_joint;
    (*this)(5,0) = - ty_FR_calf_joint * cos_q_FR_calf_joint;
    (*this)(5,1) =  ty_FR_calf_joint * sin_q_FR_calf_joint;
    return *this;
}
MotionTransforms::Type_fr_FL_thigh_X_fr_FL_hip::Type_fr_FL_thigh_X_fr_FL_hip()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_FL_thigh_X_fr_FL_hip& MotionTransforms::Type_fr_FL_thigh_X_fr_FL_hip::update(const state_t& q)
{
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    (*this)(0,0) = sin_q_FL_thigh_joint;
    (*this)(0,2) = cos_q_FL_thigh_joint;
    (*this)(1,0) = cos_q_FL_thigh_joint;
    (*this)(1,2) = -sin_q_FL_thigh_joint;
    (*this)(3,0) =  ty_FL_thigh_joint * cos_q_FL_thigh_joint;
    (*this)(3,2) = - ty_FL_thigh_joint * sin_q_FL_thigh_joint;
    (*this)(3,3) = sin_q_FL_thigh_joint;
    (*this)(3,5) = cos_q_FL_thigh_joint;
    (*this)(4,0) = - ty_FL_thigh_joint * sin_q_FL_thigh_joint;
    (*this)(4,2) = - ty_FL_thigh_joint * cos_q_FL_thigh_joint;
    (*this)(4,3) = cos_q_FL_thigh_joint;
    (*this)(4,5) = -sin_q_FL_thigh_joint;
    return *this;
}
MotionTransforms::Type_fr_FL_hip_X_fr_FL_thigh::Type_fr_FL_hip_X_fr_FL_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_FL_hip_X_fr_FL_thigh& MotionTransforms::Type_fr_FL_hip_X_fr_FL_thigh::update(const state_t& q)
{
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    (*this)(0,0) = sin_q_FL_thigh_joint;
    (*this)(0,1) = cos_q_FL_thigh_joint;
    (*this)(2,0) = cos_q_FL_thigh_joint;
    (*this)(2,1) = -sin_q_FL_thigh_joint;
    (*this)(3,0) =  ty_FL_thigh_joint * cos_q_FL_thigh_joint;
    (*this)(3,1) = - ty_FL_thigh_joint * sin_q_FL_thigh_joint;
    (*this)(3,3) = sin_q_FL_thigh_joint;
    (*this)(3,4) = cos_q_FL_thigh_joint;
    (*this)(5,0) = - ty_FL_thigh_joint * sin_q_FL_thigh_joint;
    (*this)(5,1) = - ty_FL_thigh_joint * cos_q_FL_thigh_joint;
    (*this)(5,3) = cos_q_FL_thigh_joint;
    (*this)(5,4) = -sin_q_FL_thigh_joint;
    return *this;
}
MotionTransforms::Type_fr_FL_calf_X_fr_FL_thigh::Type_fr_FL_calf_X_fr_FL_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) =  ty_FL_calf_joint;    // Maxima DSL: _k__ty_FL_calf_joint
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_FL_calf_X_fr_FL_thigh& MotionTransforms::Type_fr_FL_calf_X_fr_FL_thigh::update(const state_t& q)
{
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    (*this)(0,0) = cos_q_FL_calf_joint;
    (*this)(0,1) = sin_q_FL_calf_joint;
    (*this)(1,0) = -sin_q_FL_calf_joint;
    (*this)(1,1) = cos_q_FL_calf_joint;
    (*this)(3,2) = - ty_FL_calf_joint * cos_q_FL_calf_joint;
    (*this)(3,3) = cos_q_FL_calf_joint;
    (*this)(3,4) = sin_q_FL_calf_joint;
    (*this)(4,2) =  ty_FL_calf_joint * sin_q_FL_calf_joint;
    (*this)(4,3) = -sin_q_FL_calf_joint;
    (*this)(4,4) = cos_q_FL_calf_joint;
    return *this;
}
MotionTransforms::Type_fr_FL_thigh_X_fr_FL_calf::Type_fr_FL_thigh_X_fr_FL_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) =  ty_FL_calf_joint;    // Maxima DSL: _k__ty_FL_calf_joint
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_FL_thigh_X_fr_FL_calf& MotionTransforms::Type_fr_FL_thigh_X_fr_FL_calf::update(const state_t& q)
{
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    (*this)(0,0) = cos_q_FL_calf_joint;
    (*this)(0,1) = -sin_q_FL_calf_joint;
    (*this)(1,0) = sin_q_FL_calf_joint;
    (*this)(1,1) = cos_q_FL_calf_joint;
    (*this)(3,3) = cos_q_FL_calf_joint;
    (*this)(3,4) = -sin_q_FL_calf_joint;
    (*this)(4,3) = sin_q_FL_calf_joint;
    (*this)(4,4) = cos_q_FL_calf_joint;
    (*this)(5,0) = - ty_FL_calf_joint * cos_q_FL_calf_joint;
    (*this)(5,1) =  ty_FL_calf_joint * sin_q_FL_calf_joint;
    return *this;
}
MotionTransforms::Type_fr_RR_thigh_X_fr_RR_hip::Type_fr_RR_thigh_X_fr_RR_hip()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_RR_thigh_X_fr_RR_hip& MotionTransforms::Type_fr_RR_thigh_X_fr_RR_hip::update(const state_t& q)
{
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    (*this)(0,0) = sin_q_RR_thigh_joint;
    (*this)(0,2) = cos_q_RR_thigh_joint;
    (*this)(1,0) = cos_q_RR_thigh_joint;
    (*this)(1,2) = -sin_q_RR_thigh_joint;
    (*this)(3,0) =  ty_RR_thigh_joint * cos_q_RR_thigh_joint;
    (*this)(3,2) = - ty_RR_thigh_joint * sin_q_RR_thigh_joint;
    (*this)(3,3) = sin_q_RR_thigh_joint;
    (*this)(3,5) = cos_q_RR_thigh_joint;
    (*this)(4,0) = - ty_RR_thigh_joint * sin_q_RR_thigh_joint;
    (*this)(4,2) = - ty_RR_thigh_joint * cos_q_RR_thigh_joint;
    (*this)(4,3) = cos_q_RR_thigh_joint;
    (*this)(4,5) = -sin_q_RR_thigh_joint;
    return *this;
}
MotionTransforms::Type_fr_RR_hip_X_fr_RR_thigh::Type_fr_RR_hip_X_fr_RR_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_RR_hip_X_fr_RR_thigh& MotionTransforms::Type_fr_RR_hip_X_fr_RR_thigh::update(const state_t& q)
{
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    (*this)(0,0) = sin_q_RR_thigh_joint;
    (*this)(0,1) = cos_q_RR_thigh_joint;
    (*this)(2,0) = cos_q_RR_thigh_joint;
    (*this)(2,1) = -sin_q_RR_thigh_joint;
    (*this)(3,0) =  ty_RR_thigh_joint * cos_q_RR_thigh_joint;
    (*this)(3,1) = - ty_RR_thigh_joint * sin_q_RR_thigh_joint;
    (*this)(3,3) = sin_q_RR_thigh_joint;
    (*this)(3,4) = cos_q_RR_thigh_joint;
    (*this)(5,0) = - ty_RR_thigh_joint * sin_q_RR_thigh_joint;
    (*this)(5,1) = - ty_RR_thigh_joint * cos_q_RR_thigh_joint;
    (*this)(5,3) = cos_q_RR_thigh_joint;
    (*this)(5,4) = -sin_q_RR_thigh_joint;
    return *this;
}
MotionTransforms::Type_fr_RR_calf_X_fr_RR_thigh::Type_fr_RR_calf_X_fr_RR_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) =  ty_RR_calf_joint;    // Maxima DSL: _k__ty_RR_calf_joint
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_RR_calf_X_fr_RR_thigh& MotionTransforms::Type_fr_RR_calf_X_fr_RR_thigh::update(const state_t& q)
{
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    (*this)(0,0) = cos_q_RR_calf_joint;
    (*this)(0,1) = sin_q_RR_calf_joint;
    (*this)(1,0) = -sin_q_RR_calf_joint;
    (*this)(1,1) = cos_q_RR_calf_joint;
    (*this)(3,2) = - ty_RR_calf_joint * cos_q_RR_calf_joint;
    (*this)(3,3) = cos_q_RR_calf_joint;
    (*this)(3,4) = sin_q_RR_calf_joint;
    (*this)(4,2) =  ty_RR_calf_joint * sin_q_RR_calf_joint;
    (*this)(4,3) = -sin_q_RR_calf_joint;
    (*this)(4,4) = cos_q_RR_calf_joint;
    return *this;
}
MotionTransforms::Type_fr_RR_thigh_X_fr_RR_calf::Type_fr_RR_thigh_X_fr_RR_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) =  ty_RR_calf_joint;    // Maxima DSL: _k__ty_RR_calf_joint
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_RR_thigh_X_fr_RR_calf& MotionTransforms::Type_fr_RR_thigh_X_fr_RR_calf::update(const state_t& q)
{
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    (*this)(0,0) = cos_q_RR_calf_joint;
    (*this)(0,1) = -sin_q_RR_calf_joint;
    (*this)(1,0) = sin_q_RR_calf_joint;
    (*this)(1,1) = cos_q_RR_calf_joint;
    (*this)(3,3) = cos_q_RR_calf_joint;
    (*this)(3,4) = -sin_q_RR_calf_joint;
    (*this)(4,3) = sin_q_RR_calf_joint;
    (*this)(4,4) = cos_q_RR_calf_joint;
    (*this)(5,0) = - ty_RR_calf_joint * cos_q_RR_calf_joint;
    (*this)(5,1) =  ty_RR_calf_joint * sin_q_RR_calf_joint;
    return *this;
}
MotionTransforms::Type_fr_RL_thigh_X_fr_RL_hip::Type_fr_RL_thigh_X_fr_RL_hip()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_RL_thigh_X_fr_RL_hip& MotionTransforms::Type_fr_RL_thigh_X_fr_RL_hip::update(const state_t& q)
{
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    (*this)(0,0) = sin_q_RL_thigh_joint;
    (*this)(0,2) = cos_q_RL_thigh_joint;
    (*this)(1,0) = cos_q_RL_thigh_joint;
    (*this)(1,2) = -sin_q_RL_thigh_joint;
    (*this)(3,0) =  ty_RL_thigh_joint * cos_q_RL_thigh_joint;
    (*this)(3,2) = - ty_RL_thigh_joint * sin_q_RL_thigh_joint;
    (*this)(3,3) = sin_q_RL_thigh_joint;
    (*this)(3,5) = cos_q_RL_thigh_joint;
    (*this)(4,0) = - ty_RL_thigh_joint * sin_q_RL_thigh_joint;
    (*this)(4,2) = - ty_RL_thigh_joint * cos_q_RL_thigh_joint;
    (*this)(4,3) = cos_q_RL_thigh_joint;
    (*this)(4,5) = -sin_q_RL_thigh_joint;
    return *this;
}
MotionTransforms::Type_fr_RL_hip_X_fr_RL_thigh::Type_fr_RL_hip_X_fr_RL_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_RL_hip_X_fr_RL_thigh& MotionTransforms::Type_fr_RL_hip_X_fr_RL_thigh::update(const state_t& q)
{
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    (*this)(0,0) = sin_q_RL_thigh_joint;
    (*this)(0,1) = cos_q_RL_thigh_joint;
    (*this)(2,0) = cos_q_RL_thigh_joint;
    (*this)(2,1) = -sin_q_RL_thigh_joint;
    (*this)(3,0) =  ty_RL_thigh_joint * cos_q_RL_thigh_joint;
    (*this)(3,1) = - ty_RL_thigh_joint * sin_q_RL_thigh_joint;
    (*this)(3,3) = sin_q_RL_thigh_joint;
    (*this)(3,4) = cos_q_RL_thigh_joint;
    (*this)(5,0) = - ty_RL_thigh_joint * sin_q_RL_thigh_joint;
    (*this)(5,1) = - ty_RL_thigh_joint * cos_q_RL_thigh_joint;
    (*this)(5,3) = cos_q_RL_thigh_joint;
    (*this)(5,4) = -sin_q_RL_thigh_joint;
    return *this;
}
MotionTransforms::Type_fr_RL_calf_X_fr_RL_thigh::Type_fr_RL_calf_X_fr_RL_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) =  ty_RL_calf_joint;    // Maxima DSL: _k__ty_RL_calf_joint
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_RL_calf_X_fr_RL_thigh& MotionTransforms::Type_fr_RL_calf_X_fr_RL_thigh::update(const state_t& q)
{
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    (*this)(0,0) = cos_q_RL_calf_joint;
    (*this)(0,1) = sin_q_RL_calf_joint;
    (*this)(1,0) = -sin_q_RL_calf_joint;
    (*this)(1,1) = cos_q_RL_calf_joint;
    (*this)(3,2) = - ty_RL_calf_joint * cos_q_RL_calf_joint;
    (*this)(3,3) = cos_q_RL_calf_joint;
    (*this)(3,4) = sin_q_RL_calf_joint;
    (*this)(4,2) =  ty_RL_calf_joint * sin_q_RL_calf_joint;
    (*this)(4,3) = -sin_q_RL_calf_joint;
    (*this)(4,4) = cos_q_RL_calf_joint;
    return *this;
}
MotionTransforms::Type_fr_RL_thigh_X_fr_RL_calf::Type_fr_RL_thigh_X_fr_RL_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) =  ty_RL_calf_joint;    // Maxima DSL: _k__ty_RL_calf_joint
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_RL_thigh_X_fr_RL_calf& MotionTransforms::Type_fr_RL_thigh_X_fr_RL_calf::update(const state_t& q)
{
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    (*this)(0,0) = cos_q_RL_calf_joint;
    (*this)(0,1) = -sin_q_RL_calf_joint;
    (*this)(1,0) = sin_q_RL_calf_joint;
    (*this)(1,1) = cos_q_RL_calf_joint;
    (*this)(3,3) = cos_q_RL_calf_joint;
    (*this)(3,4) = -sin_q_RL_calf_joint;
    (*this)(4,3) = sin_q_RL_calf_joint;
    (*this)(4,4) = cos_q_RL_calf_joint;
    (*this)(5,0) = - ty_RL_calf_joint * cos_q_RL_calf_joint;
    (*this)(5,1) =  ty_RL_calf_joint * sin_q_RL_calf_joint;
    return *this;
}

ForceTransforms::Type_fr_base_X_fr_FL_calf::Type_fr_base_X_fr_FL_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FL_calf& ForceTransforms::Type_fr_base_X_fr_FL_calf::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    (*this)(0,0) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(0,1) = (-cos_q_FL_calf_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(0,3) = (((- ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(0,4) = ((( ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(0,5) = ( ty_FL_calf_joint * cos_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(1,0) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,1) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(1,2) = cos_q_FL_hip_joint;
    (*this)(1,3) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint);
    (*this)(1,4) = (((- ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)) * cos_q_FL_thigh_joint)+( ty_FL_calf_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint);
    (*this)(1,5) = ( ty_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(2,0) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,1) = (sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(2,3) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint);
    (*this)(2,4) = (((- tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+( ty_FL_calf_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint);
    (*this)(2,5) = ( tx_FL_hip_joint * cos_q_FL_hip_joint)-( ty_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(3,3) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(3,4) = (-cos_q_FL_calf_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(4,3) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(4,4) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(4,5) = cos_q_FL_hip_joint;
    (*this)(5,3) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,4) = (sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,5) = sin_q_FL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_FL_calf_X_fr_base::Type_fr_FL_calf_X_fr_base()
{
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_FL_calf_X_fr_base& ForceTransforms::Type_fr_FL_calf_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,0) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(0,1) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(0,2) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(0,3) = (((- ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(0,4) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint);
    (*this)(0,5) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint);
    (*this)(1,0) = (-cos_q_FL_calf_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(1,1) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(1,2) = (sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,3) = ((( ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(1,4) = (((- ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)) * cos_q_FL_thigh_joint)+( ty_FL_calf_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint);
    (*this)(1,5) = (((- tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+( ty_FL_calf_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint);
    (*this)(2,1) = cos_q_FL_hip_joint;
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(2,3) = ( ty_FL_calf_joint * cos_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(2,4) = ( ty_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(2,5) = ( tx_FL_hip_joint * cos_q_FL_hip_joint)-( ty_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(3,3) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(3,4) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(3,5) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(4,3) = (-cos_q_FL_calf_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(4,4) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(4,5) = (sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,4) = cos_q_FL_hip_joint;
    (*this)(5,5) = sin_q_FL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FL_hip::Type_fr_base_X_fr_FL_hip()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = - ty_FL_hip_joint;    // Maxima DSL: -_k__ty_FL_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FL_hip& ForceTransforms::Type_fr_base_X_fr_FL_hip::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,3) = - ty_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(0,4) =  ty_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(1,0) = sin_q_FL_hip_joint;
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,3) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(1,4) = - tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(2,0) = -cos_q_FL_hip_joint;
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,3) =  tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(2,4) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(4,3) = sin_q_FL_hip_joint;
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(5,3) = -cos_q_FL_hip_joint;
    (*this)(5,4) = sin_q_FL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_FL_hip_X_fr_base::Type_fr_FL_hip_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = - ty_FL_hip_joint;    // Maxima DSL: -_k__ty_FL_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_FL_hip_X_fr_base& ForceTransforms::Type_fr_FL_hip_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,1) = sin_q_FL_hip_joint;
    (*this)(0,2) = -cos_q_FL_hip_joint;
    (*this)(0,3) = - ty_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(0,4) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(0,5) =  tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = sin_q_FL_hip_joint;
    (*this)(1,3) =  ty_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(1,4) = - tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(1,5) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(3,4) = sin_q_FL_hip_joint;
    (*this)(3,5) = -cos_q_FL_hip_joint;
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = sin_q_FL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FL_thigh::Type_fr_base_X_fr_FL_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FL_thigh& ForceTransforms::Type_fr_base_X_fr_FL_thigh::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    (*this)(0,0) = cos_q_FL_thigh_joint;
    (*this)(0,1) = -sin_q_FL_thigh_joint;
    (*this)(0,3) = ((- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_FL_thigh_joint) * sin_q_FL_thigh_joint;
    (*this)(0,4) = ((- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_FL_thigh_joint) * cos_q_FL_thigh_joint;
    (*this)(0,5) =  ty_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(1,0) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(1,1) = sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(1,2) = cos_q_FL_hip_joint;
    (*this)(1,3) = ( tx_FL_hip_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+( ty_FL_thigh_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,4) = ( tx_FL_hip_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-( ty_FL_thigh_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(1,5) = - tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(2,0) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(2,1) = -cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(2,3) = ( tx_FL_hip_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(((- ty_FL_thigh_joint * cos_q_FL_hip_joint)- ty_FL_hip_joint) * cos_q_FL_thigh_joint);
    (*this)(2,4) = ((( ty_FL_thigh_joint * cos_q_FL_hip_joint)+ ty_FL_hip_joint) * sin_q_FL_thigh_joint)+( tx_FL_hip_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,5) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(3,3) = cos_q_FL_thigh_joint;
    (*this)(3,4) = -sin_q_FL_thigh_joint;
    (*this)(4,3) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(4,4) = sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(4,5) = cos_q_FL_hip_joint;
    (*this)(5,3) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(5,4) = -cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(5,5) = sin_q_FL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_FL_thigh_X_fr_base::Type_fr_FL_thigh_X_fr_base()
{
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_FL_thigh_X_fr_base& ForceTransforms::Type_fr_FL_thigh_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,0) = cos_q_FL_thigh_joint;
    (*this)(0,1) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(0,2) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(0,3) = ((- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_FL_thigh_joint) * sin_q_FL_thigh_joint;
    (*this)(0,4) = ( tx_FL_hip_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+( ty_FL_thigh_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(0,5) = ( tx_FL_hip_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(((- ty_FL_thigh_joint * cos_q_FL_hip_joint)- ty_FL_hip_joint) * cos_q_FL_thigh_joint);
    (*this)(1,0) = -sin_q_FL_thigh_joint;
    (*this)(1,1) = sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(1,2) = -cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(1,3) = ((- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_FL_thigh_joint) * cos_q_FL_thigh_joint;
    (*this)(1,4) = ( tx_FL_hip_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-( ty_FL_thigh_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(1,5) = ((( ty_FL_thigh_joint * cos_q_FL_hip_joint)+ ty_FL_hip_joint) * sin_q_FL_thigh_joint)+( tx_FL_hip_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,1) = cos_q_FL_hip_joint;
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(2,3) =  ty_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(2,4) = - tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(2,5) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(3,3) = cos_q_FL_thigh_joint;
    (*this)(3,4) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(3,5) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(4,3) = -sin_q_FL_thigh_joint;
    (*this)(4,4) = sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(4,5) = -cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(5,4) = cos_q_FL_hip_joint;
    (*this)(5,5) = sin_q_FL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FR_calf::Type_fr_base_X_fr_FR_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FR_calf& ForceTransforms::Type_fr_base_X_fr_FR_calf::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    (*this)(0,0) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(0,1) = (-cos_q_FR_calf_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(0,3) = (((- ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(0,4) = ((( ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(0,5) = ( ty_FR_calf_joint * cos_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(1,0) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,1) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(1,2) = cos_q_FR_hip_joint;
    (*this)(1,3) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint);
    (*this)(1,4) = (((- ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)) * cos_q_FR_thigh_joint)+( ty_FR_calf_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint);
    (*this)(1,5) = ( ty_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(2,0) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,1) = (sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(2,3) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint);
    (*this)(2,4) = (((- tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+( ty_FR_calf_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint);
    (*this)(2,5) = ( tx_FR_hip_joint * cos_q_FR_hip_joint)-( ty_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(3,3) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(3,4) = (-cos_q_FR_calf_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(4,3) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(4,4) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(4,5) = cos_q_FR_hip_joint;
    (*this)(5,3) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,4) = (sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,5) = sin_q_FR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_FR_calf_X_fr_base::Type_fr_FR_calf_X_fr_base()
{
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_FR_calf_X_fr_base& ForceTransforms::Type_fr_FR_calf_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,0) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(0,1) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(0,2) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(0,3) = (((- ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(0,4) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint);
    (*this)(0,5) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint);
    (*this)(1,0) = (-cos_q_FR_calf_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(1,1) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(1,2) = (sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,3) = ((( ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(1,4) = (((- ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)) * cos_q_FR_thigh_joint)+( ty_FR_calf_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint);
    (*this)(1,5) = (((- tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+( ty_FR_calf_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint);
    (*this)(2,1) = cos_q_FR_hip_joint;
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(2,3) = ( ty_FR_calf_joint * cos_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(2,4) = ( ty_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(2,5) = ( tx_FR_hip_joint * cos_q_FR_hip_joint)-( ty_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(3,3) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(3,4) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(3,5) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(4,3) = (-cos_q_FR_calf_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(4,4) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(4,5) = (sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,4) = cos_q_FR_hip_joint;
    (*this)(5,5) = sin_q_FR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FR_hip::Type_fr_base_X_fr_FR_hip()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = - ty_FR_hip_joint;    // Maxima DSL: -_k__ty_FR_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FR_hip& ForceTransforms::Type_fr_base_X_fr_FR_hip::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,3) = - ty_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(0,4) =  ty_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(1,0) = sin_q_FR_hip_joint;
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,3) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(1,4) = - tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(2,0) = -cos_q_FR_hip_joint;
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,3) =  tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(2,4) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(4,3) = sin_q_FR_hip_joint;
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(5,3) = -cos_q_FR_hip_joint;
    (*this)(5,4) = sin_q_FR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_FR_hip_X_fr_base::Type_fr_FR_hip_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = - ty_FR_hip_joint;    // Maxima DSL: -_k__ty_FR_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_FR_hip_X_fr_base& ForceTransforms::Type_fr_FR_hip_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,1) = sin_q_FR_hip_joint;
    (*this)(0,2) = -cos_q_FR_hip_joint;
    (*this)(0,3) = - ty_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(0,4) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(0,5) =  tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = sin_q_FR_hip_joint;
    (*this)(1,3) =  ty_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(1,4) = - tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(1,5) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(3,4) = sin_q_FR_hip_joint;
    (*this)(3,5) = -cos_q_FR_hip_joint;
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = sin_q_FR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FR_thigh::Type_fr_base_X_fr_FR_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FR_thigh& ForceTransforms::Type_fr_base_X_fr_FR_thigh::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    (*this)(0,0) = cos_q_FR_thigh_joint;
    (*this)(0,1) = -sin_q_FR_thigh_joint;
    (*this)(0,3) = ((- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_FR_thigh_joint) * sin_q_FR_thigh_joint;
    (*this)(0,4) = ((- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_FR_thigh_joint) * cos_q_FR_thigh_joint;
    (*this)(0,5) =  ty_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(1,0) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(1,1) = sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(1,2) = cos_q_FR_hip_joint;
    (*this)(1,3) = ( tx_FR_hip_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+( ty_FR_thigh_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,4) = ( tx_FR_hip_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-( ty_FR_thigh_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(1,5) = - tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(2,0) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(2,1) = -cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(2,3) = ( tx_FR_hip_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(((- ty_FR_thigh_joint * cos_q_FR_hip_joint)- ty_FR_hip_joint) * cos_q_FR_thigh_joint);
    (*this)(2,4) = ((( ty_FR_thigh_joint * cos_q_FR_hip_joint)+ ty_FR_hip_joint) * sin_q_FR_thigh_joint)+( tx_FR_hip_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,5) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(3,3) = cos_q_FR_thigh_joint;
    (*this)(3,4) = -sin_q_FR_thigh_joint;
    (*this)(4,3) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(4,4) = sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(4,5) = cos_q_FR_hip_joint;
    (*this)(5,3) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(5,4) = -cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(5,5) = sin_q_FR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_FR_thigh_X_fr_base::Type_fr_FR_thigh_X_fr_base()
{
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_FR_thigh_X_fr_base& ForceTransforms::Type_fr_FR_thigh_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,0) = cos_q_FR_thigh_joint;
    (*this)(0,1) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(0,2) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(0,3) = ((- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_FR_thigh_joint) * sin_q_FR_thigh_joint;
    (*this)(0,4) = ( tx_FR_hip_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+( ty_FR_thigh_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(0,5) = ( tx_FR_hip_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(((- ty_FR_thigh_joint * cos_q_FR_hip_joint)- ty_FR_hip_joint) * cos_q_FR_thigh_joint);
    (*this)(1,0) = -sin_q_FR_thigh_joint;
    (*this)(1,1) = sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(1,2) = -cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(1,3) = ((- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_FR_thigh_joint) * cos_q_FR_thigh_joint;
    (*this)(1,4) = ( tx_FR_hip_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-( ty_FR_thigh_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(1,5) = ((( ty_FR_thigh_joint * cos_q_FR_hip_joint)+ ty_FR_hip_joint) * sin_q_FR_thigh_joint)+( tx_FR_hip_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,1) = cos_q_FR_hip_joint;
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(2,3) =  ty_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(2,4) = - tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(2,5) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(3,3) = cos_q_FR_thigh_joint;
    (*this)(3,4) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(3,5) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(4,3) = -sin_q_FR_thigh_joint;
    (*this)(4,4) = sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(4,5) = -cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(5,4) = cos_q_FR_hip_joint;
    (*this)(5,5) = sin_q_FR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RL_calf::Type_fr_base_X_fr_RL_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RL_calf& ForceTransforms::Type_fr_base_X_fr_RL_calf::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    (*this)(0,0) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(0,1) = (-cos_q_RL_calf_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(0,3) = (((- ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(0,4) = ((( ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(0,5) = ( ty_RL_calf_joint * cos_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(1,0) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,1) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(1,2) = cos_q_RL_hip_joint;
    (*this)(1,3) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint);
    (*this)(1,4) = (((- ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)) * cos_q_RL_thigh_joint)+( ty_RL_calf_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint);
    (*this)(1,5) = ( ty_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(2,0) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,1) = (sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(2,3) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint);
    (*this)(2,4) = (((- tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+( ty_RL_calf_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint);
    (*this)(2,5) = ( tx_RL_hip_joint * cos_q_RL_hip_joint)-( ty_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(3,3) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(3,4) = (-cos_q_RL_calf_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(4,3) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(4,4) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(4,5) = cos_q_RL_hip_joint;
    (*this)(5,3) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,4) = (sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,5) = sin_q_RL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_RL_calf_X_fr_base::Type_fr_RL_calf_X_fr_base()
{
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_RL_calf_X_fr_base& ForceTransforms::Type_fr_RL_calf_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,0) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(0,1) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(0,2) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(0,3) = (((- ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(0,4) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint);
    (*this)(0,5) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint);
    (*this)(1,0) = (-cos_q_RL_calf_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(1,1) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(1,2) = (sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,3) = ((( ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(1,4) = (((- ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)) * cos_q_RL_thigh_joint)+( ty_RL_calf_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint);
    (*this)(1,5) = (((- tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+( ty_RL_calf_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint);
    (*this)(2,1) = cos_q_RL_hip_joint;
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(2,3) = ( ty_RL_calf_joint * cos_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(2,4) = ( ty_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(2,5) = ( tx_RL_hip_joint * cos_q_RL_hip_joint)-( ty_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(3,3) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(3,4) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(3,5) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(4,3) = (-cos_q_RL_calf_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(4,4) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(4,5) = (sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,4) = cos_q_RL_hip_joint;
    (*this)(5,5) = sin_q_RL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RL_hip::Type_fr_base_X_fr_RL_hip()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = - ty_RL_hip_joint;    // Maxima DSL: -_k__ty_RL_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RL_hip& ForceTransforms::Type_fr_base_X_fr_RL_hip::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,3) = - ty_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(0,4) =  ty_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(1,0) = sin_q_RL_hip_joint;
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,3) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(1,4) = - tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(2,0) = -cos_q_RL_hip_joint;
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,3) =  tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(2,4) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(4,3) = sin_q_RL_hip_joint;
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(5,3) = -cos_q_RL_hip_joint;
    (*this)(5,4) = sin_q_RL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_RL_hip_X_fr_base::Type_fr_RL_hip_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = - ty_RL_hip_joint;    // Maxima DSL: -_k__ty_RL_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_RL_hip_X_fr_base& ForceTransforms::Type_fr_RL_hip_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,1) = sin_q_RL_hip_joint;
    (*this)(0,2) = -cos_q_RL_hip_joint;
    (*this)(0,3) = - ty_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(0,4) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(0,5) =  tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = sin_q_RL_hip_joint;
    (*this)(1,3) =  ty_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(1,4) = - tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(1,5) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(3,4) = sin_q_RL_hip_joint;
    (*this)(3,5) = -cos_q_RL_hip_joint;
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = sin_q_RL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RL_thigh::Type_fr_base_X_fr_RL_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RL_thigh& ForceTransforms::Type_fr_base_X_fr_RL_thigh::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    (*this)(0,0) = cos_q_RL_thigh_joint;
    (*this)(0,1) = -sin_q_RL_thigh_joint;
    (*this)(0,3) = ((- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_RL_thigh_joint) * sin_q_RL_thigh_joint;
    (*this)(0,4) = ((- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_RL_thigh_joint) * cos_q_RL_thigh_joint;
    (*this)(0,5) =  ty_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(1,0) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(1,1) = sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(1,2) = cos_q_RL_hip_joint;
    (*this)(1,3) = ( tx_RL_hip_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+( ty_RL_thigh_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,4) = ( tx_RL_hip_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-( ty_RL_thigh_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(1,5) = - tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(2,0) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(2,1) = -cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(2,3) = ( tx_RL_hip_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(((- ty_RL_thigh_joint * cos_q_RL_hip_joint)- ty_RL_hip_joint) * cos_q_RL_thigh_joint);
    (*this)(2,4) = ((( ty_RL_thigh_joint * cos_q_RL_hip_joint)+ ty_RL_hip_joint) * sin_q_RL_thigh_joint)+( tx_RL_hip_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,5) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(3,3) = cos_q_RL_thigh_joint;
    (*this)(3,4) = -sin_q_RL_thigh_joint;
    (*this)(4,3) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(4,4) = sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(4,5) = cos_q_RL_hip_joint;
    (*this)(5,3) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(5,4) = -cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(5,5) = sin_q_RL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_RL_thigh_X_fr_base::Type_fr_RL_thigh_X_fr_base()
{
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_RL_thigh_X_fr_base& ForceTransforms::Type_fr_RL_thigh_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,0) = cos_q_RL_thigh_joint;
    (*this)(0,1) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(0,2) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(0,3) = ((- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_RL_thigh_joint) * sin_q_RL_thigh_joint;
    (*this)(0,4) = ( tx_RL_hip_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+( ty_RL_thigh_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(0,5) = ( tx_RL_hip_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(((- ty_RL_thigh_joint * cos_q_RL_hip_joint)- ty_RL_hip_joint) * cos_q_RL_thigh_joint);
    (*this)(1,0) = -sin_q_RL_thigh_joint;
    (*this)(1,1) = sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(1,2) = -cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(1,3) = ((- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_RL_thigh_joint) * cos_q_RL_thigh_joint;
    (*this)(1,4) = ( tx_RL_hip_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-( ty_RL_thigh_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(1,5) = ((( ty_RL_thigh_joint * cos_q_RL_hip_joint)+ ty_RL_hip_joint) * sin_q_RL_thigh_joint)+( tx_RL_hip_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,1) = cos_q_RL_hip_joint;
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(2,3) =  ty_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(2,4) = - tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(2,5) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(3,3) = cos_q_RL_thigh_joint;
    (*this)(3,4) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(3,5) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(4,3) = -sin_q_RL_thigh_joint;
    (*this)(4,4) = sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(4,5) = -cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(5,4) = cos_q_RL_hip_joint;
    (*this)(5,5) = sin_q_RL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RR_calf::Type_fr_base_X_fr_RR_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RR_calf& ForceTransforms::Type_fr_base_X_fr_RR_calf::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    (*this)(0,0) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(0,1) = (-cos_q_RR_calf_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(0,3) = (((- ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(0,4) = ((( ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(0,5) = ( ty_RR_calf_joint * cos_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(1,0) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,1) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(1,2) = cos_q_RR_hip_joint;
    (*this)(1,3) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint);
    (*this)(1,4) = (((- ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)) * cos_q_RR_thigh_joint)+( ty_RR_calf_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint);
    (*this)(1,5) = ( ty_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(2,0) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,1) = (sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(2,3) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint);
    (*this)(2,4) = (((- tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+( ty_RR_calf_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint);
    (*this)(2,5) = ( tx_RR_hip_joint * cos_q_RR_hip_joint)-( ty_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(3,3) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(3,4) = (-cos_q_RR_calf_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(4,3) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(4,4) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(4,5) = cos_q_RR_hip_joint;
    (*this)(5,3) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,4) = (sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,5) = sin_q_RR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_RR_calf_X_fr_base::Type_fr_RR_calf_X_fr_base()
{
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_RR_calf_X_fr_base& ForceTransforms::Type_fr_RR_calf_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,0) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(0,1) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(0,2) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(0,3) = (((- ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(0,4) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint);
    (*this)(0,5) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint);
    (*this)(1,0) = (-cos_q_RR_calf_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(1,1) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(1,2) = (sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,3) = ((( ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(1,4) = (((- ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)) * cos_q_RR_thigh_joint)+( ty_RR_calf_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint);
    (*this)(1,5) = (((- tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+( ty_RR_calf_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint);
    (*this)(2,1) = cos_q_RR_hip_joint;
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(2,3) = ( ty_RR_calf_joint * cos_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(2,4) = ( ty_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(2,5) = ( tx_RR_hip_joint * cos_q_RR_hip_joint)-( ty_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(3,3) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(3,4) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(3,5) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(4,3) = (-cos_q_RR_calf_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(4,4) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(4,5) = (sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,4) = cos_q_RR_hip_joint;
    (*this)(5,5) = sin_q_RR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RR_hip::Type_fr_base_X_fr_RR_hip()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = - ty_RR_hip_joint;    // Maxima DSL: -_k__ty_RR_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RR_hip& ForceTransforms::Type_fr_base_X_fr_RR_hip::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,3) = - ty_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(0,4) =  ty_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(1,0) = sin_q_RR_hip_joint;
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,3) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(1,4) = - tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(2,0) = -cos_q_RR_hip_joint;
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,3) =  tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(2,4) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(4,3) = sin_q_RR_hip_joint;
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(5,3) = -cos_q_RR_hip_joint;
    (*this)(5,4) = sin_q_RR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_RR_hip_X_fr_base::Type_fr_RR_hip_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = - ty_RR_hip_joint;    // Maxima DSL: -_k__ty_RR_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_RR_hip_X_fr_base& ForceTransforms::Type_fr_RR_hip_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,1) = sin_q_RR_hip_joint;
    (*this)(0,2) = -cos_q_RR_hip_joint;
    (*this)(0,3) = - ty_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(0,4) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(0,5) =  tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = sin_q_RR_hip_joint;
    (*this)(1,3) =  ty_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(1,4) = - tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(1,5) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(3,4) = sin_q_RR_hip_joint;
    (*this)(3,5) = -cos_q_RR_hip_joint;
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = sin_q_RR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RR_thigh::Type_fr_base_X_fr_RR_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RR_thigh& ForceTransforms::Type_fr_base_X_fr_RR_thigh::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    (*this)(0,0) = cos_q_RR_thigh_joint;
    (*this)(0,1) = -sin_q_RR_thigh_joint;
    (*this)(0,3) = ((- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_RR_thigh_joint) * sin_q_RR_thigh_joint;
    (*this)(0,4) = ((- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_RR_thigh_joint) * cos_q_RR_thigh_joint;
    (*this)(0,5) =  ty_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(1,0) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(1,1) = sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(1,2) = cos_q_RR_hip_joint;
    (*this)(1,3) = ( tx_RR_hip_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+( ty_RR_thigh_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,4) = ( tx_RR_hip_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-( ty_RR_thigh_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(1,5) = - tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(2,0) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(2,1) = -cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(2,3) = ( tx_RR_hip_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(((- ty_RR_thigh_joint * cos_q_RR_hip_joint)- ty_RR_hip_joint) * cos_q_RR_thigh_joint);
    (*this)(2,4) = ((( ty_RR_thigh_joint * cos_q_RR_hip_joint)+ ty_RR_hip_joint) * sin_q_RR_thigh_joint)+( tx_RR_hip_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,5) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(3,3) = cos_q_RR_thigh_joint;
    (*this)(3,4) = -sin_q_RR_thigh_joint;
    (*this)(4,3) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(4,4) = sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(4,5) = cos_q_RR_hip_joint;
    (*this)(5,3) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(5,4) = -cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(5,5) = sin_q_RR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_RR_thigh_X_fr_base::Type_fr_RR_thigh_X_fr_base()
{
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_RR_thigh_X_fr_base& ForceTransforms::Type_fr_RR_thigh_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,0) = cos_q_RR_thigh_joint;
    (*this)(0,1) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(0,2) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(0,3) = ((- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_RR_thigh_joint) * sin_q_RR_thigh_joint;
    (*this)(0,4) = ( tx_RR_hip_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+( ty_RR_thigh_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(0,5) = ( tx_RR_hip_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(((- ty_RR_thigh_joint * cos_q_RR_hip_joint)- ty_RR_hip_joint) * cos_q_RR_thigh_joint);
    (*this)(1,0) = -sin_q_RR_thigh_joint;
    (*this)(1,1) = sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(1,2) = -cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(1,3) = ((- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_RR_thigh_joint) * cos_q_RR_thigh_joint;
    (*this)(1,4) = ( tx_RR_hip_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-( ty_RR_thigh_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(1,5) = ((( ty_RR_thigh_joint * cos_q_RR_hip_joint)+ ty_RR_hip_joint) * sin_q_RR_thigh_joint)+( tx_RR_hip_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,1) = cos_q_RR_hip_joint;
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(2,3) =  ty_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(2,4) = - tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(2,5) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(3,3) = cos_q_RR_thigh_joint;
    (*this)(3,4) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(3,5) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(4,3) = -sin_q_RR_thigh_joint;
    (*this)(4,4) = sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(4,5) = -cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(5,4) = cos_q_RR_hip_joint;
    (*this)(5,5) = sin_q_RR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FL_calf_COM::Type_fr_base_X_fr_FL_calf_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FL_calf_COM& ForceTransforms::Type_fr_base_X_fr_FL_calf_COM::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    (*this)(0,0) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(0,2) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(0,3) = (((- ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(0,4) = ((( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)-( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_fr_FL_calf_COM * sin_q_FL_calf_joint)+( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * cos_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(0,5) = (((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(1,0) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,3) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * cos_q_FL_calf_joint)- ty_fr_FL_calf_COM) * cos_q_FL_hip_joint);
    (*this)(1,4) = ((( tx_fr_FL_calf_COM * sin_q_FL_calf_joint)+( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+((( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)-( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)) * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(1,5) = ((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * sin_q_FL_calf_joint)- tx_fr_FL_calf_COM) * cos_q_FL_hip_joint);
    (*this)(2,0) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,2) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(2,3) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * cos_q_FL_calf_joint)- ty_fr_FL_calf_COM) * sin_q_FL_hip_joint);
    (*this)(2,4) = (((- tx_fr_FL_calf_COM * sin_q_FL_calf_joint)-( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)- ty_FL_calf_joint) * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+((( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)-( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)) * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)+( tx_FL_hip_joint * cos_q_FL_hip_joint);
    (*this)(2,5) = ((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * sin_q_FL_calf_joint)- tx_fr_FL_calf_COM) * sin_q_FL_hip_joint);
    (*this)(3,3) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(3,5) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(4,3) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,3) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,4) = sin_q_FL_hip_joint;
    (*this)(5,5) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    return *this;
}
ForceTransforms::Type_fr_FL_calf_COM_X_fr_base::Type_fr_FL_calf_COM_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_FL_calf_COM_X_fr_base& ForceTransforms::Type_fr_FL_calf_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,0) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(0,1) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(0,2) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(0,3) = (((- ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(0,4) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * cos_q_FL_calf_joint)- ty_fr_FL_calf_COM) * cos_q_FL_hip_joint);
    (*this)(0,5) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * cos_q_FL_calf_joint)- ty_fr_FL_calf_COM) * sin_q_FL_hip_joint);
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = sin_q_FL_hip_joint;
    (*this)(1,3) = ((( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)-( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_fr_FL_calf_COM * sin_q_FL_calf_joint)+( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * cos_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(1,4) = ((( tx_fr_FL_calf_COM * sin_q_FL_calf_joint)+( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+((( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)-( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)) * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(1,5) = (((- tx_fr_FL_calf_COM * sin_q_FL_calf_joint)-( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)- ty_FL_calf_joint) * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+((( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)-( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)) * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)+( tx_FL_hip_joint * cos_q_FL_hip_joint);
    (*this)(2,0) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(2,1) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,2) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(2,3) = (((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(2,4) = ((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * sin_q_FL_calf_joint)- tx_fr_FL_calf_COM) * cos_q_FL_hip_joint);
    (*this)(2,5) = ((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * sin_q_FL_calf_joint)- tx_fr_FL_calf_COM) * sin_q_FL_hip_joint);
    (*this)(3,3) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(3,4) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(3,5) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = sin_q_FL_hip_joint;
    (*this)(5,3) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(5,4) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,5) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FL_foot::Type_fr_base_X_fr_FL_foot()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FL_foot& ForceTransforms::Type_fr_base_X_fr_FL_foot::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    (*this)(0,0) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(0,2) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(0,3) = (((- ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(0,4) = (- ty_fr_FL_foot * sin_q_FL_calf_joint * sin_q_FL_thigh_joint)+((( ty_fr_FL_foot * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * cos_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(0,5) = (((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(1,0) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,3) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * cos_q_FL_calf_joint)- ty_fr_FL_foot) * cos_q_FL_hip_joint);
    (*this)(1,4) = ((( ty_fr_FL_foot * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+( ty_fr_FL_foot * sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(1,5) = ((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint);
    (*this)(2,0) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,2) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(2,3) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * cos_q_FL_calf_joint)- ty_fr_FL_foot) * sin_q_FL_hip_joint);
    (*this)(2,4) = (((- ty_fr_FL_foot * cos_q_FL_calf_joint)- ty_FL_calf_joint) * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-( ty_fr_FL_foot * sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)+( tx_FL_hip_joint * cos_q_FL_hip_joint);
    (*this)(2,5) = ((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint);
    (*this)(3,3) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(3,5) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(4,3) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,3) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,4) = sin_q_FL_hip_joint;
    (*this)(5,5) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    return *this;
}
ForceTransforms::Type_fr_FL_foot_X_fr_base::Type_fr_FL_foot_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_FL_foot_X_fr_base& ForceTransforms::Type_fr_FL_foot_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,0) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(0,1) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(0,2) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(0,3) = (((- ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(0,4) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * cos_q_FL_calf_joint)- ty_fr_FL_foot) * cos_q_FL_hip_joint);
    (*this)(0,5) = ((( tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_hip_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+(((- ty_FL_calf_joint * cos_q_FL_calf_joint)- ty_fr_FL_foot) * sin_q_FL_hip_joint);
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = sin_q_FL_hip_joint;
    (*this)(1,3) = (- ty_fr_FL_foot * sin_q_FL_calf_joint * sin_q_FL_thigh_joint)+((( ty_fr_FL_foot * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * cos_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(1,4) = ((( ty_fr_FL_foot * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+( ty_fr_FL_foot * sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(1,5) = (((- ty_fr_FL_foot * cos_q_FL_calf_joint)- ty_FL_calf_joint) * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-( ty_fr_FL_foot * sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)+( tx_FL_hip_joint * cos_q_FL_hip_joint);
    (*this)(2,0) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(2,1) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,2) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(2,3) = (((- ty_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)+( ty_FL_thigh_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(2,4) = ((( ty_FL_thigh_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_thigh_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint);
    (*this)(2,5) = ((( tx_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * cos_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- tx_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)-( ty_FL_thigh_joint * sin_q_FL_calf_joint * cos_q_FL_hip_joint)-( ty_FL_hip_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint);
    (*this)(3,3) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(3,4) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(3,5) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = sin_q_FL_hip_joint;
    (*this)(5,3) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(5,4) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,5) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FL_hip_COM::Type_fr_base_X_fr_FL_hip_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FL_hip_COM& ForceTransforms::Type_fr_base_X_fr_FL_hip_COM::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,4) = ( ty_FL_hip_joint * sin_q_FL_hip_joint)+ tx_fr_FL_hip_COM;
    (*this)(0,5) = ( ty_FL_hip_joint * cos_q_FL_hip_joint)+ ty_fr_FL_hip_COM;
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = -sin_q_FL_hip_joint;
    (*this)(1,3) = ( ty_fr_FL_hip_COM * sin_q_FL_hip_joint)-( tx_fr_FL_hip_COM * cos_q_FL_hip_joint);
    (*this)(1,4) = (- tz_fr_FL_hip_COM- tx_FL_hip_joint) * sin_q_FL_hip_joint;
    (*this)(1,5) = (- tz_fr_FL_hip_COM- tx_FL_hip_joint) * cos_q_FL_hip_joint;
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,2) = cos_q_FL_hip_joint;
    (*this)(2,3) = (- tx_fr_FL_hip_COM * sin_q_FL_hip_joint)-( ty_fr_FL_hip_COM * cos_q_FL_hip_joint)- ty_FL_hip_joint;
    (*this)(2,4) = ( tz_fr_FL_hip_COM+ tx_FL_hip_joint) * cos_q_FL_hip_joint;
    (*this)(2,5) = (- tz_fr_FL_hip_COM- tx_FL_hip_joint) * sin_q_FL_hip_joint;
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = -sin_q_FL_hip_joint;
    (*this)(5,4) = sin_q_FL_hip_joint;
    (*this)(5,5) = cos_q_FL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_FL_hip_COM_X_fr_base::Type_fr_FL_hip_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_FL_hip_COM_X_fr_base& ForceTransforms::Type_fr_FL_hip_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,4) = ( ty_fr_FL_hip_COM * sin_q_FL_hip_joint)-( tx_fr_FL_hip_COM * cos_q_FL_hip_joint);
    (*this)(0,5) = (- tx_fr_FL_hip_COM * sin_q_FL_hip_joint)-( ty_fr_FL_hip_COM * cos_q_FL_hip_joint)- ty_FL_hip_joint;
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = sin_q_FL_hip_joint;
    (*this)(1,3) = ( ty_FL_hip_joint * sin_q_FL_hip_joint)+ tx_fr_FL_hip_COM;
    (*this)(1,4) = (- tz_fr_FL_hip_COM- tx_FL_hip_joint) * sin_q_FL_hip_joint;
    (*this)(1,5) = ( tz_fr_FL_hip_COM+ tx_FL_hip_joint) * cos_q_FL_hip_joint;
    (*this)(2,1) = -sin_q_FL_hip_joint;
    (*this)(2,2) = cos_q_FL_hip_joint;
    (*this)(2,3) = ( ty_FL_hip_joint * cos_q_FL_hip_joint)+ ty_fr_FL_hip_COM;
    (*this)(2,4) = (- tz_fr_FL_hip_COM- tx_FL_hip_joint) * cos_q_FL_hip_joint;
    (*this)(2,5) = (- tz_fr_FL_hip_COM- tx_FL_hip_joint) * sin_q_FL_hip_joint;
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = sin_q_FL_hip_joint;
    (*this)(5,4) = -sin_q_FL_hip_joint;
    (*this)(5,5) = cos_q_FL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FL_thigh_COM::Type_fr_base_X_fr_FL_thigh_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FL_thigh_COM& ForceTransforms::Type_fr_base_X_fr_FL_thigh_COM::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    (*this)(0,0) = cos_q_FL_thigh_joint;
    (*this)(0,2) = sin_q_FL_thigh_joint;
    (*this)(0,3) = ((- ty_FL_hip_joint * cos_q_FL_hip_joint)- tz_fr_FL_thigh_COM- ty_FL_thigh_joint) * sin_q_FL_thigh_joint;
    (*this)(0,4) = ( tx_fr_FL_thigh_COM * sin_q_FL_thigh_joint)+( ty_fr_FL_thigh_COM * cos_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(0,5) = (( ty_FL_hip_joint * cos_q_FL_hip_joint)+ tz_fr_FL_thigh_COM+ ty_FL_thigh_joint) * cos_q_FL_thigh_joint;
    (*this)(1,0) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = -sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(1,3) = ( tx_FL_hip_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+(( tz_fr_FL_thigh_COM+ ty_FL_thigh_joint) * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( ty_fr_FL_thigh_COM * cos_q_FL_hip_joint);
    (*this)(1,4) = ( ty_fr_FL_thigh_COM * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-( tx_fr_FL_thigh_COM * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(1,5) = (( tz_fr_FL_thigh_COM+ ty_FL_thigh_joint) * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-( tx_FL_hip_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_fr_FL_thigh_COM * cos_q_FL_hip_joint);
    (*this)(2,0) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,2) = cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(2,3) = ( tx_FL_hip_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+((((- tz_fr_FL_thigh_COM- ty_FL_thigh_joint) * cos_q_FL_hip_joint)- ty_FL_hip_joint) * cos_q_FL_thigh_joint)-( ty_fr_FL_thigh_COM * sin_q_FL_hip_joint);
    (*this)(2,4) = (- ty_fr_FL_thigh_COM * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+( tx_fr_FL_thigh_COM * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)+( tx_FL_hip_joint * cos_q_FL_hip_joint);
    (*this)(2,5) = ((((- tz_fr_FL_thigh_COM- ty_FL_thigh_joint) * cos_q_FL_hip_joint)- ty_FL_hip_joint) * sin_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_fr_FL_thigh_COM * sin_q_FL_hip_joint);
    (*this)(3,3) = cos_q_FL_thigh_joint;
    (*this)(3,5) = sin_q_FL_thigh_joint;
    (*this)(4,3) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = -sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(5,3) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(5,4) = sin_q_FL_hip_joint;
    (*this)(5,5) = cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    return *this;
}
ForceTransforms::Type_fr_FL_thigh_COM_X_fr_base::Type_fr_FL_thigh_COM_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_FL_thigh_COM_X_fr_base& ForceTransforms::Type_fr_FL_thigh_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,0) = cos_q_FL_thigh_joint;
    (*this)(0,1) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(0,2) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(0,3) = ((- ty_FL_hip_joint * cos_q_FL_hip_joint)- tz_fr_FL_thigh_COM- ty_FL_thigh_joint) * sin_q_FL_thigh_joint;
    (*this)(0,4) = ( tx_FL_hip_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+(( tz_fr_FL_thigh_COM+ ty_FL_thigh_joint) * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( ty_fr_FL_thigh_COM * cos_q_FL_hip_joint);
    (*this)(0,5) = ( tx_FL_hip_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+((((- tz_fr_FL_thigh_COM- ty_FL_thigh_joint) * cos_q_FL_hip_joint)- ty_FL_hip_joint) * cos_q_FL_thigh_joint)-( ty_fr_FL_thigh_COM * sin_q_FL_hip_joint);
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = sin_q_FL_hip_joint;
    (*this)(1,3) = ( tx_fr_FL_thigh_COM * sin_q_FL_thigh_joint)+( ty_fr_FL_thigh_COM * cos_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(1,4) = ( ty_fr_FL_thigh_COM * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-( tx_fr_FL_thigh_COM * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(1,5) = (- ty_fr_FL_thigh_COM * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+( tx_fr_FL_thigh_COM * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)+( tx_FL_hip_joint * cos_q_FL_hip_joint);
    (*this)(2,0) = sin_q_FL_thigh_joint;
    (*this)(2,1) = -sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(2,2) = cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(2,3) = (( ty_FL_hip_joint * cos_q_FL_hip_joint)+ tz_fr_FL_thigh_COM+ ty_FL_thigh_joint) * cos_q_FL_thigh_joint;
    (*this)(2,4) = (( tz_fr_FL_thigh_COM+ ty_FL_thigh_joint) * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-( tx_FL_hip_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_fr_FL_thigh_COM * cos_q_FL_hip_joint);
    (*this)(2,5) = ((((- tz_fr_FL_thigh_COM- ty_FL_thigh_joint) * cos_q_FL_hip_joint)- ty_FL_hip_joint) * sin_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( tx_fr_FL_thigh_COM * sin_q_FL_hip_joint);
    (*this)(3,3) = cos_q_FL_thigh_joint;
    (*this)(3,4) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(3,5) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = sin_q_FL_hip_joint;
    (*this)(5,3) = sin_q_FL_thigh_joint;
    (*this)(5,4) = -sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(5,5) = cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FL_thigh_shoulder::Type_fr_base_X_fr_FL_thigh_shoulder()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FL_thigh_shoulder& ForceTransforms::Type_fr_base_X_fr_FL_thigh_shoulder::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,4) =  ty_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(0,5) = ( ty_FL_hip_joint * cos_q_FL_hip_joint)+ ty_fr_FL_thigh_shoulder;
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = -sin_q_FL_hip_joint;
    (*this)(1,3) =  ty_fr_FL_thigh_shoulder * sin_q_FL_hip_joint;
    (*this)(1,4) = - tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(1,5) = - tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,2) = cos_q_FL_hip_joint;
    (*this)(2,3) = (- ty_fr_FL_thigh_shoulder * cos_q_FL_hip_joint)- ty_FL_hip_joint;
    (*this)(2,4) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(2,5) = - tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = -sin_q_FL_hip_joint;
    (*this)(5,4) = sin_q_FL_hip_joint;
    (*this)(5,5) = cos_q_FL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_FL_thigh_shoulder_X_fr_base::Type_fr_FL_thigh_shoulder_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_FL_thigh_shoulder_X_fr_base& ForceTransforms::Type_fr_FL_thigh_shoulder_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,4) =  ty_fr_FL_thigh_shoulder * sin_q_FL_hip_joint;
    (*this)(0,5) = (- ty_fr_FL_thigh_shoulder * cos_q_FL_hip_joint)- ty_FL_hip_joint;
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = sin_q_FL_hip_joint;
    (*this)(1,3) =  ty_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(1,4) = - tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(1,5) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(2,1) = -sin_q_FL_hip_joint;
    (*this)(2,2) = cos_q_FL_hip_joint;
    (*this)(2,3) = ( ty_FL_hip_joint * cos_q_FL_hip_joint)+ ty_fr_FL_thigh_shoulder;
    (*this)(2,4) = - tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(2,5) = - tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(4,4) = cos_q_FL_hip_joint;
    (*this)(4,5) = sin_q_FL_hip_joint;
    (*this)(5,4) = -sin_q_FL_hip_joint;
    (*this)(5,5) = cos_q_FL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FR_calf_COM::Type_fr_base_X_fr_FR_calf_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FR_calf_COM& ForceTransforms::Type_fr_base_X_fr_FR_calf_COM::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    (*this)(0,0) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(0,2) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(0,3) = (((- ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(0,4) = ((( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)-( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_fr_FR_calf_COM * sin_q_FR_calf_joint)+( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * cos_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(0,5) = (((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(1,0) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,3) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * cos_q_FR_calf_joint)- ty_fr_FR_calf_COM) * cos_q_FR_hip_joint);
    (*this)(1,4) = ((( tx_fr_FR_calf_COM * sin_q_FR_calf_joint)+( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+((( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)-( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)) * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(1,5) = ((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * sin_q_FR_calf_joint)- tx_fr_FR_calf_COM) * cos_q_FR_hip_joint);
    (*this)(2,0) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,2) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(2,3) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * cos_q_FR_calf_joint)- ty_fr_FR_calf_COM) * sin_q_FR_hip_joint);
    (*this)(2,4) = (((- tx_fr_FR_calf_COM * sin_q_FR_calf_joint)-( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)- ty_FR_calf_joint) * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+((( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)-( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)) * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)+( tx_FR_hip_joint * cos_q_FR_hip_joint);
    (*this)(2,5) = ((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * sin_q_FR_calf_joint)- tx_fr_FR_calf_COM) * sin_q_FR_hip_joint);
    (*this)(3,3) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(3,5) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(4,3) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,3) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,4) = sin_q_FR_hip_joint;
    (*this)(5,5) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    return *this;
}
ForceTransforms::Type_fr_FR_calf_COM_X_fr_base::Type_fr_FR_calf_COM_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_FR_calf_COM_X_fr_base& ForceTransforms::Type_fr_FR_calf_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,0) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(0,1) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(0,2) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(0,3) = (((- ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(0,4) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * cos_q_FR_calf_joint)- ty_fr_FR_calf_COM) * cos_q_FR_hip_joint);
    (*this)(0,5) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * cos_q_FR_calf_joint)- ty_fr_FR_calf_COM) * sin_q_FR_hip_joint);
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = sin_q_FR_hip_joint;
    (*this)(1,3) = ((( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)-( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_fr_FR_calf_COM * sin_q_FR_calf_joint)+( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * cos_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(1,4) = ((( tx_fr_FR_calf_COM * sin_q_FR_calf_joint)+( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+((( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)-( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)) * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(1,5) = (((- tx_fr_FR_calf_COM * sin_q_FR_calf_joint)-( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)- ty_FR_calf_joint) * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+((( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)-( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)) * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)+( tx_FR_hip_joint * cos_q_FR_hip_joint);
    (*this)(2,0) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(2,1) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,2) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(2,3) = (((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(2,4) = ((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * sin_q_FR_calf_joint)- tx_fr_FR_calf_COM) * cos_q_FR_hip_joint);
    (*this)(2,5) = ((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * sin_q_FR_calf_joint)- tx_fr_FR_calf_COM) * sin_q_FR_hip_joint);
    (*this)(3,3) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(3,4) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(3,5) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = sin_q_FR_hip_joint;
    (*this)(5,3) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(5,4) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,5) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FR_foot::Type_fr_base_X_fr_FR_foot()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FR_foot& ForceTransforms::Type_fr_base_X_fr_FR_foot::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    (*this)(0,0) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(0,2) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(0,3) = (((- ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(0,4) = (- ty_fr_FR_foot * sin_q_FR_calf_joint * sin_q_FR_thigh_joint)+((( ty_fr_FR_foot * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * cos_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(0,5) = (((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(1,0) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,3) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * cos_q_FR_calf_joint)- ty_fr_FR_foot) * cos_q_FR_hip_joint);
    (*this)(1,4) = ((( ty_fr_FR_foot * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+( ty_fr_FR_foot * sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(1,5) = ((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint);
    (*this)(2,0) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,2) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(2,3) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * cos_q_FR_calf_joint)- ty_fr_FR_foot) * sin_q_FR_hip_joint);
    (*this)(2,4) = (((- ty_fr_FR_foot * cos_q_FR_calf_joint)- ty_FR_calf_joint) * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-( ty_fr_FR_foot * sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)+( tx_FR_hip_joint * cos_q_FR_hip_joint);
    (*this)(2,5) = ((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint);
    (*this)(3,3) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(3,5) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(4,3) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,3) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,4) = sin_q_FR_hip_joint;
    (*this)(5,5) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    return *this;
}
ForceTransforms::Type_fr_FR_foot_X_fr_base::Type_fr_FR_foot_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_FR_foot_X_fr_base& ForceTransforms::Type_fr_FR_foot_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,0) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(0,1) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(0,2) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(0,3) = (((- ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(0,4) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * cos_q_FR_calf_joint)- ty_fr_FR_foot) * cos_q_FR_hip_joint);
    (*this)(0,5) = ((( tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_hip_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+(((- ty_FR_calf_joint * cos_q_FR_calf_joint)- ty_fr_FR_foot) * sin_q_FR_hip_joint);
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = sin_q_FR_hip_joint;
    (*this)(1,3) = (- ty_fr_FR_foot * sin_q_FR_calf_joint * sin_q_FR_thigh_joint)+((( ty_fr_FR_foot * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * cos_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(1,4) = ((( ty_fr_FR_foot * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+( ty_fr_FR_foot * sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(1,5) = (((- ty_fr_FR_foot * cos_q_FR_calf_joint)- ty_FR_calf_joint) * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-( ty_fr_FR_foot * sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)+( tx_FR_hip_joint * cos_q_FR_hip_joint);
    (*this)(2,0) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(2,1) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,2) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(2,3) = (((- ty_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)+( ty_FR_thigh_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(2,4) = ((( ty_FR_thigh_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_thigh_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint);
    (*this)(2,5) = ((( tx_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * cos_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- tx_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)-( ty_FR_thigh_joint * sin_q_FR_calf_joint * cos_q_FR_hip_joint)-( ty_FR_hip_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint);
    (*this)(3,3) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(3,4) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(3,5) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = sin_q_FR_hip_joint;
    (*this)(5,3) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(5,4) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,5) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FR_hip_COM::Type_fr_base_X_fr_FR_hip_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FR_hip_COM& ForceTransforms::Type_fr_base_X_fr_FR_hip_COM::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,4) = ( ty_FR_hip_joint * sin_q_FR_hip_joint)+ tx_fr_FR_hip_COM;
    (*this)(0,5) = ( ty_FR_hip_joint * cos_q_FR_hip_joint)+ ty_fr_FR_hip_COM;
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = -sin_q_FR_hip_joint;
    (*this)(1,3) = ( ty_fr_FR_hip_COM * sin_q_FR_hip_joint)-( tx_fr_FR_hip_COM * cos_q_FR_hip_joint);
    (*this)(1,4) = (- tz_fr_FR_hip_COM- tx_FR_hip_joint) * sin_q_FR_hip_joint;
    (*this)(1,5) = (- tz_fr_FR_hip_COM- tx_FR_hip_joint) * cos_q_FR_hip_joint;
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,2) = cos_q_FR_hip_joint;
    (*this)(2,3) = (- tx_fr_FR_hip_COM * sin_q_FR_hip_joint)-( ty_fr_FR_hip_COM * cos_q_FR_hip_joint)- ty_FR_hip_joint;
    (*this)(2,4) = ( tz_fr_FR_hip_COM+ tx_FR_hip_joint) * cos_q_FR_hip_joint;
    (*this)(2,5) = (- tz_fr_FR_hip_COM- tx_FR_hip_joint) * sin_q_FR_hip_joint;
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = -sin_q_FR_hip_joint;
    (*this)(5,4) = sin_q_FR_hip_joint;
    (*this)(5,5) = cos_q_FR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_FR_hip_COM_X_fr_base::Type_fr_FR_hip_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_FR_hip_COM_X_fr_base& ForceTransforms::Type_fr_FR_hip_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,4) = ( ty_fr_FR_hip_COM * sin_q_FR_hip_joint)-( tx_fr_FR_hip_COM * cos_q_FR_hip_joint);
    (*this)(0,5) = (- tx_fr_FR_hip_COM * sin_q_FR_hip_joint)-( ty_fr_FR_hip_COM * cos_q_FR_hip_joint)- ty_FR_hip_joint;
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = sin_q_FR_hip_joint;
    (*this)(1,3) = ( ty_FR_hip_joint * sin_q_FR_hip_joint)+ tx_fr_FR_hip_COM;
    (*this)(1,4) = (- tz_fr_FR_hip_COM- tx_FR_hip_joint) * sin_q_FR_hip_joint;
    (*this)(1,5) = ( tz_fr_FR_hip_COM+ tx_FR_hip_joint) * cos_q_FR_hip_joint;
    (*this)(2,1) = -sin_q_FR_hip_joint;
    (*this)(2,2) = cos_q_FR_hip_joint;
    (*this)(2,3) = ( ty_FR_hip_joint * cos_q_FR_hip_joint)+ ty_fr_FR_hip_COM;
    (*this)(2,4) = (- tz_fr_FR_hip_COM- tx_FR_hip_joint) * cos_q_FR_hip_joint;
    (*this)(2,5) = (- tz_fr_FR_hip_COM- tx_FR_hip_joint) * sin_q_FR_hip_joint;
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = sin_q_FR_hip_joint;
    (*this)(5,4) = -sin_q_FR_hip_joint;
    (*this)(5,5) = cos_q_FR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FR_thigh_COM::Type_fr_base_X_fr_FR_thigh_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FR_thigh_COM& ForceTransforms::Type_fr_base_X_fr_FR_thigh_COM::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    (*this)(0,0) = cos_q_FR_thigh_joint;
    (*this)(0,2) = sin_q_FR_thigh_joint;
    (*this)(0,3) = ((- ty_FR_hip_joint * cos_q_FR_hip_joint)- tz_fr_FR_thigh_COM- ty_FR_thigh_joint) * sin_q_FR_thigh_joint;
    (*this)(0,4) = ( tx_fr_FR_thigh_COM * sin_q_FR_thigh_joint)+( ty_fr_FR_thigh_COM * cos_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(0,5) = (( ty_FR_hip_joint * cos_q_FR_hip_joint)+ tz_fr_FR_thigh_COM+ ty_FR_thigh_joint) * cos_q_FR_thigh_joint;
    (*this)(1,0) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = -sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(1,3) = ( tx_FR_hip_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+(( tz_fr_FR_thigh_COM+ ty_FR_thigh_joint) * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( ty_fr_FR_thigh_COM * cos_q_FR_hip_joint);
    (*this)(1,4) = ( ty_fr_FR_thigh_COM * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-( tx_fr_FR_thigh_COM * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(1,5) = (( tz_fr_FR_thigh_COM+ ty_FR_thigh_joint) * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-( tx_FR_hip_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_fr_FR_thigh_COM * cos_q_FR_hip_joint);
    (*this)(2,0) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,2) = cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(2,3) = ( tx_FR_hip_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+((((- tz_fr_FR_thigh_COM- ty_FR_thigh_joint) * cos_q_FR_hip_joint)- ty_FR_hip_joint) * cos_q_FR_thigh_joint)-( ty_fr_FR_thigh_COM * sin_q_FR_hip_joint);
    (*this)(2,4) = (- ty_fr_FR_thigh_COM * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+( tx_fr_FR_thigh_COM * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)+( tx_FR_hip_joint * cos_q_FR_hip_joint);
    (*this)(2,5) = ((((- tz_fr_FR_thigh_COM- ty_FR_thigh_joint) * cos_q_FR_hip_joint)- ty_FR_hip_joint) * sin_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_fr_FR_thigh_COM * sin_q_FR_hip_joint);
    (*this)(3,3) = cos_q_FR_thigh_joint;
    (*this)(3,5) = sin_q_FR_thigh_joint;
    (*this)(4,3) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = -sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(5,3) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(5,4) = sin_q_FR_hip_joint;
    (*this)(5,5) = cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    return *this;
}
ForceTransforms::Type_fr_FR_thigh_COM_X_fr_base::Type_fr_FR_thigh_COM_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_FR_thigh_COM_X_fr_base& ForceTransforms::Type_fr_FR_thigh_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,0) = cos_q_FR_thigh_joint;
    (*this)(0,1) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(0,2) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(0,3) = ((- ty_FR_hip_joint * cos_q_FR_hip_joint)- tz_fr_FR_thigh_COM- ty_FR_thigh_joint) * sin_q_FR_thigh_joint;
    (*this)(0,4) = ( tx_FR_hip_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+(( tz_fr_FR_thigh_COM+ ty_FR_thigh_joint) * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( ty_fr_FR_thigh_COM * cos_q_FR_hip_joint);
    (*this)(0,5) = ( tx_FR_hip_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+((((- tz_fr_FR_thigh_COM- ty_FR_thigh_joint) * cos_q_FR_hip_joint)- ty_FR_hip_joint) * cos_q_FR_thigh_joint)-( ty_fr_FR_thigh_COM * sin_q_FR_hip_joint);
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = sin_q_FR_hip_joint;
    (*this)(1,3) = ( tx_fr_FR_thigh_COM * sin_q_FR_thigh_joint)+( ty_fr_FR_thigh_COM * cos_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(1,4) = ( ty_fr_FR_thigh_COM * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-( tx_fr_FR_thigh_COM * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(1,5) = (- ty_fr_FR_thigh_COM * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+( tx_fr_FR_thigh_COM * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)+( tx_FR_hip_joint * cos_q_FR_hip_joint);
    (*this)(2,0) = sin_q_FR_thigh_joint;
    (*this)(2,1) = -sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(2,2) = cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(2,3) = (( ty_FR_hip_joint * cos_q_FR_hip_joint)+ tz_fr_FR_thigh_COM+ ty_FR_thigh_joint) * cos_q_FR_thigh_joint;
    (*this)(2,4) = (( tz_fr_FR_thigh_COM+ ty_FR_thigh_joint) * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-( tx_FR_hip_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_fr_FR_thigh_COM * cos_q_FR_hip_joint);
    (*this)(2,5) = ((((- tz_fr_FR_thigh_COM- ty_FR_thigh_joint) * cos_q_FR_hip_joint)- ty_FR_hip_joint) * sin_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( tx_fr_FR_thigh_COM * sin_q_FR_hip_joint);
    (*this)(3,3) = cos_q_FR_thigh_joint;
    (*this)(3,4) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(3,5) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = sin_q_FR_hip_joint;
    (*this)(5,3) = sin_q_FR_thigh_joint;
    (*this)(5,4) = -sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(5,5) = cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FR_thigh_shoulder::Type_fr_base_X_fr_FR_thigh_shoulder()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FR_thigh_shoulder& ForceTransforms::Type_fr_base_X_fr_FR_thigh_shoulder::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,4) =  ty_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(0,5) = ( ty_FR_hip_joint * cos_q_FR_hip_joint)+ ty_fr_FR_thigh_shoulder;
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = -sin_q_FR_hip_joint;
    (*this)(1,3) =  ty_fr_FR_thigh_shoulder * sin_q_FR_hip_joint;
    (*this)(1,4) = - tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(1,5) = - tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,2) = cos_q_FR_hip_joint;
    (*this)(2,3) = (- ty_fr_FR_thigh_shoulder * cos_q_FR_hip_joint)- ty_FR_hip_joint;
    (*this)(2,4) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(2,5) = - tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = -sin_q_FR_hip_joint;
    (*this)(5,4) = sin_q_FR_hip_joint;
    (*this)(5,5) = cos_q_FR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_FR_thigh_shoulder_X_fr_base::Type_fr_FR_thigh_shoulder_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_FR_thigh_shoulder_X_fr_base& ForceTransforms::Type_fr_FR_thigh_shoulder_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,4) =  ty_fr_FR_thigh_shoulder * sin_q_FR_hip_joint;
    (*this)(0,5) = (- ty_fr_FR_thigh_shoulder * cos_q_FR_hip_joint)- ty_FR_hip_joint;
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = sin_q_FR_hip_joint;
    (*this)(1,3) =  ty_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(1,4) = - tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(1,5) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(2,1) = -sin_q_FR_hip_joint;
    (*this)(2,2) = cos_q_FR_hip_joint;
    (*this)(2,3) = ( ty_FR_hip_joint * cos_q_FR_hip_joint)+ ty_fr_FR_thigh_shoulder;
    (*this)(2,4) = - tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(2,5) = - tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(4,4) = cos_q_FR_hip_joint;
    (*this)(4,5) = sin_q_FR_hip_joint;
    (*this)(5,4) = -sin_q_FR_hip_joint;
    (*this)(5,5) = cos_q_FR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RL_calf_COM::Type_fr_base_X_fr_RL_calf_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RL_calf_COM& ForceTransforms::Type_fr_base_X_fr_RL_calf_COM::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    (*this)(0,0) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(0,2) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(0,3) = (((- ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(0,4) = ((( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)-( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_fr_RL_calf_COM * sin_q_RL_calf_joint)+( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * cos_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(0,5) = (((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(1,0) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,3) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * cos_q_RL_calf_joint)- ty_fr_RL_calf_COM) * cos_q_RL_hip_joint);
    (*this)(1,4) = ((( tx_fr_RL_calf_COM * sin_q_RL_calf_joint)+( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+((( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)-( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)) * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(1,5) = ((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * sin_q_RL_calf_joint)- tx_fr_RL_calf_COM) * cos_q_RL_hip_joint);
    (*this)(2,0) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,2) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(2,3) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * cos_q_RL_calf_joint)- ty_fr_RL_calf_COM) * sin_q_RL_hip_joint);
    (*this)(2,4) = (((- tx_fr_RL_calf_COM * sin_q_RL_calf_joint)-( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)- ty_RL_calf_joint) * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+((( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)-( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)) * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)+( tx_RL_hip_joint * cos_q_RL_hip_joint);
    (*this)(2,5) = ((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * sin_q_RL_calf_joint)- tx_fr_RL_calf_COM) * sin_q_RL_hip_joint);
    (*this)(3,3) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(3,5) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(4,3) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,3) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,4) = sin_q_RL_hip_joint;
    (*this)(5,5) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    return *this;
}
ForceTransforms::Type_fr_RL_calf_COM_X_fr_base::Type_fr_RL_calf_COM_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_RL_calf_COM_X_fr_base& ForceTransforms::Type_fr_RL_calf_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,0) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(0,1) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(0,2) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(0,3) = (((- ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(0,4) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * cos_q_RL_calf_joint)- ty_fr_RL_calf_COM) * cos_q_RL_hip_joint);
    (*this)(0,5) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * cos_q_RL_calf_joint)- ty_fr_RL_calf_COM) * sin_q_RL_hip_joint);
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = sin_q_RL_hip_joint;
    (*this)(1,3) = ((( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)-( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_fr_RL_calf_COM * sin_q_RL_calf_joint)+( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * cos_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(1,4) = ((( tx_fr_RL_calf_COM * sin_q_RL_calf_joint)+( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+((( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)-( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)) * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(1,5) = (((- tx_fr_RL_calf_COM * sin_q_RL_calf_joint)-( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)- ty_RL_calf_joint) * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+((( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)-( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)) * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)+( tx_RL_hip_joint * cos_q_RL_hip_joint);
    (*this)(2,0) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(2,1) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,2) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(2,3) = (((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(2,4) = ((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * sin_q_RL_calf_joint)- tx_fr_RL_calf_COM) * cos_q_RL_hip_joint);
    (*this)(2,5) = ((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * sin_q_RL_calf_joint)- tx_fr_RL_calf_COM) * sin_q_RL_hip_joint);
    (*this)(3,3) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(3,4) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(3,5) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = sin_q_RL_hip_joint;
    (*this)(5,3) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(5,4) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,5) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RL_foot::Type_fr_base_X_fr_RL_foot()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RL_foot& ForceTransforms::Type_fr_base_X_fr_RL_foot::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    (*this)(0,0) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(0,2) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(0,3) = (((- ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(0,4) = (- ty_fr_RL_foot * sin_q_RL_calf_joint * sin_q_RL_thigh_joint)+((( ty_fr_RL_foot * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * cos_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(0,5) = (((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(1,0) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,3) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * cos_q_RL_calf_joint)- ty_fr_RL_foot) * cos_q_RL_hip_joint);
    (*this)(1,4) = ((( ty_fr_RL_foot * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+( ty_fr_RL_foot * sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(1,5) = ((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint);
    (*this)(2,0) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,2) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(2,3) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * cos_q_RL_calf_joint)- ty_fr_RL_foot) * sin_q_RL_hip_joint);
    (*this)(2,4) = (((- ty_fr_RL_foot * cos_q_RL_calf_joint)- ty_RL_calf_joint) * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-( ty_fr_RL_foot * sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)+( tx_RL_hip_joint * cos_q_RL_hip_joint);
    (*this)(2,5) = ((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint);
    (*this)(3,3) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(3,5) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(4,3) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,3) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,4) = sin_q_RL_hip_joint;
    (*this)(5,5) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    return *this;
}
ForceTransforms::Type_fr_RL_foot_X_fr_base::Type_fr_RL_foot_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_RL_foot_X_fr_base& ForceTransforms::Type_fr_RL_foot_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,0) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(0,1) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(0,2) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(0,3) = (((- ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(0,4) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * cos_q_RL_calf_joint)- ty_fr_RL_foot) * cos_q_RL_hip_joint);
    (*this)(0,5) = ((( tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_hip_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+(((- ty_RL_calf_joint * cos_q_RL_calf_joint)- ty_fr_RL_foot) * sin_q_RL_hip_joint);
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = sin_q_RL_hip_joint;
    (*this)(1,3) = (- ty_fr_RL_foot * sin_q_RL_calf_joint * sin_q_RL_thigh_joint)+((( ty_fr_RL_foot * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * cos_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(1,4) = ((( ty_fr_RL_foot * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+( ty_fr_RL_foot * sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(1,5) = (((- ty_fr_RL_foot * cos_q_RL_calf_joint)- ty_RL_calf_joint) * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-( ty_fr_RL_foot * sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)+( tx_RL_hip_joint * cos_q_RL_hip_joint);
    (*this)(2,0) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(2,1) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,2) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(2,3) = (((- ty_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)+( ty_RL_thigh_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(2,4) = ((( ty_RL_thigh_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_thigh_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint);
    (*this)(2,5) = ((( tx_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * cos_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- tx_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)-( ty_RL_thigh_joint * sin_q_RL_calf_joint * cos_q_RL_hip_joint)-( ty_RL_hip_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint);
    (*this)(3,3) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(3,4) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(3,5) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = sin_q_RL_hip_joint;
    (*this)(5,3) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(5,4) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,5) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RL_hip_COM::Type_fr_base_X_fr_RL_hip_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RL_hip_COM& ForceTransforms::Type_fr_base_X_fr_RL_hip_COM::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,4) = ( ty_RL_hip_joint * sin_q_RL_hip_joint)+ tx_fr_RL_hip_COM;
    (*this)(0,5) = ( ty_RL_hip_joint * cos_q_RL_hip_joint)+ ty_fr_RL_hip_COM;
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = -sin_q_RL_hip_joint;
    (*this)(1,3) = ( ty_fr_RL_hip_COM * sin_q_RL_hip_joint)-( tx_fr_RL_hip_COM * cos_q_RL_hip_joint);
    (*this)(1,4) = (- tz_fr_RL_hip_COM- tx_RL_hip_joint) * sin_q_RL_hip_joint;
    (*this)(1,5) = (- tz_fr_RL_hip_COM- tx_RL_hip_joint) * cos_q_RL_hip_joint;
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,2) = cos_q_RL_hip_joint;
    (*this)(2,3) = (- tx_fr_RL_hip_COM * sin_q_RL_hip_joint)-( ty_fr_RL_hip_COM * cos_q_RL_hip_joint)- ty_RL_hip_joint;
    (*this)(2,4) = ( tz_fr_RL_hip_COM+ tx_RL_hip_joint) * cos_q_RL_hip_joint;
    (*this)(2,5) = (- tz_fr_RL_hip_COM- tx_RL_hip_joint) * sin_q_RL_hip_joint;
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = -sin_q_RL_hip_joint;
    (*this)(5,4) = sin_q_RL_hip_joint;
    (*this)(5,5) = cos_q_RL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_RL_hip_COM_X_fr_base::Type_fr_RL_hip_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_RL_hip_COM_X_fr_base& ForceTransforms::Type_fr_RL_hip_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,4) = ( ty_fr_RL_hip_COM * sin_q_RL_hip_joint)-( tx_fr_RL_hip_COM * cos_q_RL_hip_joint);
    (*this)(0,5) = (- tx_fr_RL_hip_COM * sin_q_RL_hip_joint)-( ty_fr_RL_hip_COM * cos_q_RL_hip_joint)- ty_RL_hip_joint;
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = sin_q_RL_hip_joint;
    (*this)(1,3) = ( ty_RL_hip_joint * sin_q_RL_hip_joint)+ tx_fr_RL_hip_COM;
    (*this)(1,4) = (- tz_fr_RL_hip_COM- tx_RL_hip_joint) * sin_q_RL_hip_joint;
    (*this)(1,5) = ( tz_fr_RL_hip_COM+ tx_RL_hip_joint) * cos_q_RL_hip_joint;
    (*this)(2,1) = -sin_q_RL_hip_joint;
    (*this)(2,2) = cos_q_RL_hip_joint;
    (*this)(2,3) = ( ty_RL_hip_joint * cos_q_RL_hip_joint)+ ty_fr_RL_hip_COM;
    (*this)(2,4) = (- tz_fr_RL_hip_COM- tx_RL_hip_joint) * cos_q_RL_hip_joint;
    (*this)(2,5) = (- tz_fr_RL_hip_COM- tx_RL_hip_joint) * sin_q_RL_hip_joint;
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = sin_q_RL_hip_joint;
    (*this)(5,4) = -sin_q_RL_hip_joint;
    (*this)(5,5) = cos_q_RL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RL_thigh_COM::Type_fr_base_X_fr_RL_thigh_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RL_thigh_COM& ForceTransforms::Type_fr_base_X_fr_RL_thigh_COM::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    (*this)(0,0) = cos_q_RL_thigh_joint;
    (*this)(0,2) = sin_q_RL_thigh_joint;
    (*this)(0,3) = ((- ty_RL_hip_joint * cos_q_RL_hip_joint)- tz_fr_RL_thigh_COM- ty_RL_thigh_joint) * sin_q_RL_thigh_joint;
    (*this)(0,4) = ( tx_fr_RL_thigh_COM * sin_q_RL_thigh_joint)+( ty_fr_RL_thigh_COM * cos_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(0,5) = (( ty_RL_hip_joint * cos_q_RL_hip_joint)+ tz_fr_RL_thigh_COM+ ty_RL_thigh_joint) * cos_q_RL_thigh_joint;
    (*this)(1,0) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = -sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(1,3) = ( tx_RL_hip_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+(( tz_fr_RL_thigh_COM+ ty_RL_thigh_joint) * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( ty_fr_RL_thigh_COM * cos_q_RL_hip_joint);
    (*this)(1,4) = ( ty_fr_RL_thigh_COM * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-( tx_fr_RL_thigh_COM * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(1,5) = (( tz_fr_RL_thigh_COM+ ty_RL_thigh_joint) * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-( tx_RL_hip_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_fr_RL_thigh_COM * cos_q_RL_hip_joint);
    (*this)(2,0) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,2) = cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(2,3) = ( tx_RL_hip_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+((((- tz_fr_RL_thigh_COM- ty_RL_thigh_joint) * cos_q_RL_hip_joint)- ty_RL_hip_joint) * cos_q_RL_thigh_joint)-( ty_fr_RL_thigh_COM * sin_q_RL_hip_joint);
    (*this)(2,4) = (- ty_fr_RL_thigh_COM * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+( tx_fr_RL_thigh_COM * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)+( tx_RL_hip_joint * cos_q_RL_hip_joint);
    (*this)(2,5) = ((((- tz_fr_RL_thigh_COM- ty_RL_thigh_joint) * cos_q_RL_hip_joint)- ty_RL_hip_joint) * sin_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_fr_RL_thigh_COM * sin_q_RL_hip_joint);
    (*this)(3,3) = cos_q_RL_thigh_joint;
    (*this)(3,5) = sin_q_RL_thigh_joint;
    (*this)(4,3) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = -sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(5,3) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(5,4) = sin_q_RL_hip_joint;
    (*this)(5,5) = cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    return *this;
}
ForceTransforms::Type_fr_RL_thigh_COM_X_fr_base::Type_fr_RL_thigh_COM_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_RL_thigh_COM_X_fr_base& ForceTransforms::Type_fr_RL_thigh_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,0) = cos_q_RL_thigh_joint;
    (*this)(0,1) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(0,2) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(0,3) = ((- ty_RL_hip_joint * cos_q_RL_hip_joint)- tz_fr_RL_thigh_COM- ty_RL_thigh_joint) * sin_q_RL_thigh_joint;
    (*this)(0,4) = ( tx_RL_hip_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+(( tz_fr_RL_thigh_COM+ ty_RL_thigh_joint) * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( ty_fr_RL_thigh_COM * cos_q_RL_hip_joint);
    (*this)(0,5) = ( tx_RL_hip_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+((((- tz_fr_RL_thigh_COM- ty_RL_thigh_joint) * cos_q_RL_hip_joint)- ty_RL_hip_joint) * cos_q_RL_thigh_joint)-( ty_fr_RL_thigh_COM * sin_q_RL_hip_joint);
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = sin_q_RL_hip_joint;
    (*this)(1,3) = ( tx_fr_RL_thigh_COM * sin_q_RL_thigh_joint)+( ty_fr_RL_thigh_COM * cos_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(1,4) = ( ty_fr_RL_thigh_COM * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-( tx_fr_RL_thigh_COM * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(1,5) = (- ty_fr_RL_thigh_COM * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+( tx_fr_RL_thigh_COM * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)+( tx_RL_hip_joint * cos_q_RL_hip_joint);
    (*this)(2,0) = sin_q_RL_thigh_joint;
    (*this)(2,1) = -sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(2,2) = cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(2,3) = (( ty_RL_hip_joint * cos_q_RL_hip_joint)+ tz_fr_RL_thigh_COM+ ty_RL_thigh_joint) * cos_q_RL_thigh_joint;
    (*this)(2,4) = (( tz_fr_RL_thigh_COM+ ty_RL_thigh_joint) * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-( tx_RL_hip_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_fr_RL_thigh_COM * cos_q_RL_hip_joint);
    (*this)(2,5) = ((((- tz_fr_RL_thigh_COM- ty_RL_thigh_joint) * cos_q_RL_hip_joint)- ty_RL_hip_joint) * sin_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( tx_fr_RL_thigh_COM * sin_q_RL_hip_joint);
    (*this)(3,3) = cos_q_RL_thigh_joint;
    (*this)(3,4) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(3,5) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = sin_q_RL_hip_joint;
    (*this)(5,3) = sin_q_RL_thigh_joint;
    (*this)(5,4) = -sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(5,5) = cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RL_thigh_shoulder::Type_fr_base_X_fr_RL_thigh_shoulder()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RL_thigh_shoulder& ForceTransforms::Type_fr_base_X_fr_RL_thigh_shoulder::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,4) =  ty_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(0,5) = ( ty_RL_hip_joint * cos_q_RL_hip_joint)+ ty_fr_RL_thigh_shoulder;
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = -sin_q_RL_hip_joint;
    (*this)(1,3) =  ty_fr_RL_thigh_shoulder * sin_q_RL_hip_joint;
    (*this)(1,4) = - tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(1,5) = - tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,2) = cos_q_RL_hip_joint;
    (*this)(2,3) = (- ty_fr_RL_thigh_shoulder * cos_q_RL_hip_joint)- ty_RL_hip_joint;
    (*this)(2,4) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(2,5) = - tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = -sin_q_RL_hip_joint;
    (*this)(5,4) = sin_q_RL_hip_joint;
    (*this)(5,5) = cos_q_RL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_RL_thigh_shoulder_X_fr_base::Type_fr_RL_thigh_shoulder_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_RL_thigh_shoulder_X_fr_base& ForceTransforms::Type_fr_RL_thigh_shoulder_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,4) =  ty_fr_RL_thigh_shoulder * sin_q_RL_hip_joint;
    (*this)(0,5) = (- ty_fr_RL_thigh_shoulder * cos_q_RL_hip_joint)- ty_RL_hip_joint;
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = sin_q_RL_hip_joint;
    (*this)(1,3) =  ty_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(1,4) = - tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(1,5) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(2,1) = -sin_q_RL_hip_joint;
    (*this)(2,2) = cos_q_RL_hip_joint;
    (*this)(2,3) = ( ty_RL_hip_joint * cos_q_RL_hip_joint)+ ty_fr_RL_thigh_shoulder;
    (*this)(2,4) = - tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(2,5) = - tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(4,4) = cos_q_RL_hip_joint;
    (*this)(4,5) = sin_q_RL_hip_joint;
    (*this)(5,4) = -sin_q_RL_hip_joint;
    (*this)(5,5) = cos_q_RL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RR_calf_COM::Type_fr_base_X_fr_RR_calf_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RR_calf_COM& ForceTransforms::Type_fr_base_X_fr_RR_calf_COM::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    (*this)(0,0) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(0,2) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(0,3) = (((- ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(0,4) = ((( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)-( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_fr_RR_calf_COM * sin_q_RR_calf_joint)+( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * cos_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(0,5) = (((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(1,0) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,3) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * cos_q_RR_calf_joint)- ty_fr_RR_calf_COM) * cos_q_RR_hip_joint);
    (*this)(1,4) = ((( tx_fr_RR_calf_COM * sin_q_RR_calf_joint)+( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+((( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)-( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)) * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(1,5) = ((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * sin_q_RR_calf_joint)- tx_fr_RR_calf_COM) * cos_q_RR_hip_joint);
    (*this)(2,0) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,2) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(2,3) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * cos_q_RR_calf_joint)- ty_fr_RR_calf_COM) * sin_q_RR_hip_joint);
    (*this)(2,4) = (((- tx_fr_RR_calf_COM * sin_q_RR_calf_joint)-( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)- ty_RR_calf_joint) * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+((( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)-( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)) * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)+( tx_RR_hip_joint * cos_q_RR_hip_joint);
    (*this)(2,5) = ((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * sin_q_RR_calf_joint)- tx_fr_RR_calf_COM) * sin_q_RR_hip_joint);
    (*this)(3,3) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(3,5) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(4,3) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,3) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,4) = sin_q_RR_hip_joint;
    (*this)(5,5) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    return *this;
}
ForceTransforms::Type_fr_RR_calf_COM_X_fr_base::Type_fr_RR_calf_COM_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_RR_calf_COM_X_fr_base& ForceTransforms::Type_fr_RR_calf_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,0) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(0,1) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(0,2) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(0,3) = (((- ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(0,4) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * cos_q_RR_calf_joint)- ty_fr_RR_calf_COM) * cos_q_RR_hip_joint);
    (*this)(0,5) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * cos_q_RR_calf_joint)- ty_fr_RR_calf_COM) * sin_q_RR_hip_joint);
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = sin_q_RR_hip_joint;
    (*this)(1,3) = ((( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)-( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_fr_RR_calf_COM * sin_q_RR_calf_joint)+( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * cos_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(1,4) = ((( tx_fr_RR_calf_COM * sin_q_RR_calf_joint)+( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+((( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)-( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)) * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(1,5) = (((- tx_fr_RR_calf_COM * sin_q_RR_calf_joint)-( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)- ty_RR_calf_joint) * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+((( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)-( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)) * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)+( tx_RR_hip_joint * cos_q_RR_hip_joint);
    (*this)(2,0) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(2,1) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,2) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(2,3) = (((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(2,4) = ((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * sin_q_RR_calf_joint)- tx_fr_RR_calf_COM) * cos_q_RR_hip_joint);
    (*this)(2,5) = ((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * sin_q_RR_calf_joint)- tx_fr_RR_calf_COM) * sin_q_RR_hip_joint);
    (*this)(3,3) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(3,4) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(3,5) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = sin_q_RR_hip_joint;
    (*this)(5,3) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(5,4) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,5) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RR_foot::Type_fr_base_X_fr_RR_foot()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RR_foot& ForceTransforms::Type_fr_base_X_fr_RR_foot::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    (*this)(0,0) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(0,2) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(0,3) = (((- ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(0,4) = (- ty_fr_RR_foot * sin_q_RR_calf_joint * sin_q_RR_thigh_joint)+((( ty_fr_RR_foot * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * cos_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(0,5) = (((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(1,0) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,3) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * cos_q_RR_calf_joint)- ty_fr_RR_foot) * cos_q_RR_hip_joint);
    (*this)(1,4) = ((( ty_fr_RR_foot * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+( ty_fr_RR_foot * sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(1,5) = ((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint);
    (*this)(2,0) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,2) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(2,3) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * cos_q_RR_calf_joint)- ty_fr_RR_foot) * sin_q_RR_hip_joint);
    (*this)(2,4) = (((- ty_fr_RR_foot * cos_q_RR_calf_joint)- ty_RR_calf_joint) * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-( ty_fr_RR_foot * sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)+( tx_RR_hip_joint * cos_q_RR_hip_joint);
    (*this)(2,5) = ((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint);
    (*this)(3,3) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(3,5) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(4,3) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,3) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,4) = sin_q_RR_hip_joint;
    (*this)(5,5) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    return *this;
}
ForceTransforms::Type_fr_RR_foot_X_fr_base::Type_fr_RR_foot_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_RR_foot_X_fr_base& ForceTransforms::Type_fr_RR_foot_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,0) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(0,1) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(0,2) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(0,3) = (((- ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(0,4) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * cos_q_RR_calf_joint)- ty_fr_RR_foot) * cos_q_RR_hip_joint);
    (*this)(0,5) = ((( tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_hip_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+(((- ty_RR_calf_joint * cos_q_RR_calf_joint)- ty_fr_RR_foot) * sin_q_RR_hip_joint);
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = sin_q_RR_hip_joint;
    (*this)(1,3) = (- ty_fr_RR_foot * sin_q_RR_calf_joint * sin_q_RR_thigh_joint)+((( ty_fr_RR_foot * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * cos_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(1,4) = ((( ty_fr_RR_foot * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+( ty_fr_RR_foot * sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(1,5) = (((- ty_fr_RR_foot * cos_q_RR_calf_joint)- ty_RR_calf_joint) * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-( ty_fr_RR_foot * sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)+( tx_RR_hip_joint * cos_q_RR_hip_joint);
    (*this)(2,0) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(2,1) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,2) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(2,3) = (((- ty_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)+( ty_RR_thigh_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(2,4) = ((( ty_RR_thigh_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_thigh_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint);
    (*this)(2,5) = ((( tx_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * cos_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- tx_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)-( ty_RR_thigh_joint * sin_q_RR_calf_joint * cos_q_RR_hip_joint)-( ty_RR_hip_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint);
    (*this)(3,3) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(3,4) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(3,5) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = sin_q_RR_hip_joint;
    (*this)(5,3) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(5,4) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,5) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RR_hip_COM::Type_fr_base_X_fr_RR_hip_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RR_hip_COM& ForceTransforms::Type_fr_base_X_fr_RR_hip_COM::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,4) = ( ty_RR_hip_joint * sin_q_RR_hip_joint)+ tx_fr_RR_hip_COM;
    (*this)(0,5) = ( ty_RR_hip_joint * cos_q_RR_hip_joint)+ ty_fr_RR_hip_COM;
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = -sin_q_RR_hip_joint;
    (*this)(1,3) = ( ty_fr_RR_hip_COM * sin_q_RR_hip_joint)-( tx_fr_RR_hip_COM * cos_q_RR_hip_joint);
    (*this)(1,4) = (- tz_fr_RR_hip_COM- tx_RR_hip_joint) * sin_q_RR_hip_joint;
    (*this)(1,5) = (- tz_fr_RR_hip_COM- tx_RR_hip_joint) * cos_q_RR_hip_joint;
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,2) = cos_q_RR_hip_joint;
    (*this)(2,3) = (- tx_fr_RR_hip_COM * sin_q_RR_hip_joint)-( ty_fr_RR_hip_COM * cos_q_RR_hip_joint)- ty_RR_hip_joint;
    (*this)(2,4) = ( tz_fr_RR_hip_COM+ tx_RR_hip_joint) * cos_q_RR_hip_joint;
    (*this)(2,5) = (- tz_fr_RR_hip_COM- tx_RR_hip_joint) * sin_q_RR_hip_joint;
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = -sin_q_RR_hip_joint;
    (*this)(5,4) = sin_q_RR_hip_joint;
    (*this)(5,5) = cos_q_RR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_RR_hip_COM_X_fr_base::Type_fr_RR_hip_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_RR_hip_COM_X_fr_base& ForceTransforms::Type_fr_RR_hip_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,4) = ( ty_fr_RR_hip_COM * sin_q_RR_hip_joint)-( tx_fr_RR_hip_COM * cos_q_RR_hip_joint);
    (*this)(0,5) = (- tx_fr_RR_hip_COM * sin_q_RR_hip_joint)-( ty_fr_RR_hip_COM * cos_q_RR_hip_joint)- ty_RR_hip_joint;
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = sin_q_RR_hip_joint;
    (*this)(1,3) = ( ty_RR_hip_joint * sin_q_RR_hip_joint)+ tx_fr_RR_hip_COM;
    (*this)(1,4) = (- tz_fr_RR_hip_COM- tx_RR_hip_joint) * sin_q_RR_hip_joint;
    (*this)(1,5) = ( tz_fr_RR_hip_COM+ tx_RR_hip_joint) * cos_q_RR_hip_joint;
    (*this)(2,1) = -sin_q_RR_hip_joint;
    (*this)(2,2) = cos_q_RR_hip_joint;
    (*this)(2,3) = ( ty_RR_hip_joint * cos_q_RR_hip_joint)+ ty_fr_RR_hip_COM;
    (*this)(2,4) = (- tz_fr_RR_hip_COM- tx_RR_hip_joint) * cos_q_RR_hip_joint;
    (*this)(2,5) = (- tz_fr_RR_hip_COM- tx_RR_hip_joint) * sin_q_RR_hip_joint;
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = sin_q_RR_hip_joint;
    (*this)(5,4) = -sin_q_RR_hip_joint;
    (*this)(5,5) = cos_q_RR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RR_thigh_COM::Type_fr_base_X_fr_RR_thigh_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RR_thigh_COM& ForceTransforms::Type_fr_base_X_fr_RR_thigh_COM::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    (*this)(0,0) = cos_q_RR_thigh_joint;
    (*this)(0,2) = sin_q_RR_thigh_joint;
    (*this)(0,3) = ((- ty_RR_hip_joint * cos_q_RR_hip_joint)- tz_fr_RR_thigh_COM- ty_RR_thigh_joint) * sin_q_RR_thigh_joint;
    (*this)(0,4) = ( tx_fr_RR_thigh_COM * sin_q_RR_thigh_joint)+( ty_fr_RR_thigh_COM * cos_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(0,5) = (( ty_RR_hip_joint * cos_q_RR_hip_joint)+ tz_fr_RR_thigh_COM+ ty_RR_thigh_joint) * cos_q_RR_thigh_joint;
    (*this)(1,0) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = -sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(1,3) = ( tx_RR_hip_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+(( tz_fr_RR_thigh_COM+ ty_RR_thigh_joint) * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( ty_fr_RR_thigh_COM * cos_q_RR_hip_joint);
    (*this)(1,4) = ( ty_fr_RR_thigh_COM * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-( tx_fr_RR_thigh_COM * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(1,5) = (( tz_fr_RR_thigh_COM+ ty_RR_thigh_joint) * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-( tx_RR_hip_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_fr_RR_thigh_COM * cos_q_RR_hip_joint);
    (*this)(2,0) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,2) = cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(2,3) = ( tx_RR_hip_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+((((- tz_fr_RR_thigh_COM- ty_RR_thigh_joint) * cos_q_RR_hip_joint)- ty_RR_hip_joint) * cos_q_RR_thigh_joint)-( ty_fr_RR_thigh_COM * sin_q_RR_hip_joint);
    (*this)(2,4) = (- ty_fr_RR_thigh_COM * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+( tx_fr_RR_thigh_COM * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)+( tx_RR_hip_joint * cos_q_RR_hip_joint);
    (*this)(2,5) = ((((- tz_fr_RR_thigh_COM- ty_RR_thigh_joint) * cos_q_RR_hip_joint)- ty_RR_hip_joint) * sin_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_fr_RR_thigh_COM * sin_q_RR_hip_joint);
    (*this)(3,3) = cos_q_RR_thigh_joint;
    (*this)(3,5) = sin_q_RR_thigh_joint;
    (*this)(4,3) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = -sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(5,3) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(5,4) = sin_q_RR_hip_joint;
    (*this)(5,5) = cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    return *this;
}
ForceTransforms::Type_fr_RR_thigh_COM_X_fr_base::Type_fr_RR_thigh_COM_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_RR_thigh_COM_X_fr_base& ForceTransforms::Type_fr_RR_thigh_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,0) = cos_q_RR_thigh_joint;
    (*this)(0,1) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(0,2) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(0,3) = ((- ty_RR_hip_joint * cos_q_RR_hip_joint)- tz_fr_RR_thigh_COM- ty_RR_thigh_joint) * sin_q_RR_thigh_joint;
    (*this)(0,4) = ( tx_RR_hip_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+(( tz_fr_RR_thigh_COM+ ty_RR_thigh_joint) * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( ty_fr_RR_thigh_COM * cos_q_RR_hip_joint);
    (*this)(0,5) = ( tx_RR_hip_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+((((- tz_fr_RR_thigh_COM- ty_RR_thigh_joint) * cos_q_RR_hip_joint)- ty_RR_hip_joint) * cos_q_RR_thigh_joint)-( ty_fr_RR_thigh_COM * sin_q_RR_hip_joint);
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = sin_q_RR_hip_joint;
    (*this)(1,3) = ( tx_fr_RR_thigh_COM * sin_q_RR_thigh_joint)+( ty_fr_RR_thigh_COM * cos_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(1,4) = ( ty_fr_RR_thigh_COM * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-( tx_fr_RR_thigh_COM * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(1,5) = (- ty_fr_RR_thigh_COM * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+( tx_fr_RR_thigh_COM * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)+( tx_RR_hip_joint * cos_q_RR_hip_joint);
    (*this)(2,0) = sin_q_RR_thigh_joint;
    (*this)(2,1) = -sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(2,2) = cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(2,3) = (( ty_RR_hip_joint * cos_q_RR_hip_joint)+ tz_fr_RR_thigh_COM+ ty_RR_thigh_joint) * cos_q_RR_thigh_joint;
    (*this)(2,4) = (( tz_fr_RR_thigh_COM+ ty_RR_thigh_joint) * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-( tx_RR_hip_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_fr_RR_thigh_COM * cos_q_RR_hip_joint);
    (*this)(2,5) = ((((- tz_fr_RR_thigh_COM- ty_RR_thigh_joint) * cos_q_RR_hip_joint)- ty_RR_hip_joint) * sin_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( tx_fr_RR_thigh_COM * sin_q_RR_hip_joint);
    (*this)(3,3) = cos_q_RR_thigh_joint;
    (*this)(3,4) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(3,5) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = sin_q_RR_hip_joint;
    (*this)(5,3) = sin_q_RR_thigh_joint;
    (*this)(5,4) = -sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(5,5) = cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RR_thigh_shoulder::Type_fr_base_X_fr_RR_thigh_shoulder()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RR_thigh_shoulder& ForceTransforms::Type_fr_base_X_fr_RR_thigh_shoulder::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,4) =  ty_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(0,5) = ( ty_RR_hip_joint * cos_q_RR_hip_joint)+ ty_fr_RR_thigh_shoulder;
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = -sin_q_RR_hip_joint;
    (*this)(1,3) =  ty_fr_RR_thigh_shoulder * sin_q_RR_hip_joint;
    (*this)(1,4) = - tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(1,5) = - tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,2) = cos_q_RR_hip_joint;
    (*this)(2,3) = (- ty_fr_RR_thigh_shoulder * cos_q_RR_hip_joint)- ty_RR_hip_joint;
    (*this)(2,4) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(2,5) = - tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = -sin_q_RR_hip_joint;
    (*this)(5,4) = sin_q_RR_hip_joint;
    (*this)(5,5) = cos_q_RR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_RR_thigh_shoulder_X_fr_base::Type_fr_RR_thigh_shoulder_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_RR_thigh_shoulder_X_fr_base& ForceTransforms::Type_fr_RR_thigh_shoulder_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,4) =  ty_fr_RR_thigh_shoulder * sin_q_RR_hip_joint;
    (*this)(0,5) = (- ty_fr_RR_thigh_shoulder * cos_q_RR_hip_joint)- ty_RR_hip_joint;
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = sin_q_RR_hip_joint;
    (*this)(1,3) =  ty_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(1,4) = - tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(1,5) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(2,1) = -sin_q_RR_hip_joint;
    (*this)(2,2) = cos_q_RR_hip_joint;
    (*this)(2,3) = ( ty_RR_hip_joint * cos_q_RR_hip_joint)+ ty_fr_RR_thigh_shoulder;
    (*this)(2,4) = - tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(2,5) = - tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(4,4) = cos_q_RR_hip_joint;
    (*this)(4,5) = sin_q_RR_hip_joint;
    (*this)(5,4) = -sin_q_RR_hip_joint;
    (*this)(5,5) = cos_q_RR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_base_COM::Type_fr_base_X_fr_base_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = - tz_fr_base_COM;    // Maxima DSL: -_k__tz_fr_base_COM
    (*this)(0,5) =  ty_fr_base_COM;    // Maxima DSL: _k__ty_fr_base_COM
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tz_fr_base_COM;    // Maxima DSL: _k__tz_fr_base_COM
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - ty_fr_base_COM;    // Maxima DSL: -_k__ty_fr_base_COM
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_X_fr_base_COM& ForceTransforms::Type_fr_base_X_fr_base_COM::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_COM_X_fr_base::Type_fr_base_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) =  tz_fr_base_COM;    // Maxima DSL: _k__tz_fr_base_COM
    (*this)(0,5) = - ty_fr_base_COM;    // Maxima DSL: -_k__ty_fr_base_COM
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = - tz_fr_base_COM;    // Maxima DSL: -_k__tz_fr_base_COM
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  ty_fr_base_COM;    // Maxima DSL: _k__ty_fr_base_COM
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_COM_X_fr_base& ForceTransforms::Type_fr_base_COM_X_fr_base::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_imu_link::Type_fr_base_X_fr_imu_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_X_fr_imu_link& ForceTransforms::Type_fr_base_X_fr_imu_link::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_imu_link_X_fr_base::Type_fr_imu_link_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_imu_link_X_fr_base& ForceTransforms::Type_fr_imu_link_X_fr_base::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_trunk::Type_fr_base_X_fr_trunk()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_X_fr_trunk& ForceTransforms::Type_fr_base_X_fr_trunk::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_trunk_X_fr_base::Type_fr_trunk_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_trunk_X_fr_base& ForceTransforms::Type_fr_trunk_X_fr_base::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FL_hip_joint::Type_fr_base_X_fr_FL_hip_joint()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - ty_FL_hip_joint;    // Maxima DSL: -_k__ty_FL_hip_joint
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tx_FL_hip_joint;    // Maxima DSL: _k__tx_FL_hip_joint
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) =  tx_FL_hip_joint;    // Maxima DSL: _k__tx_FL_hip_joint
    (*this)(2,5) = - ty_FL_hip_joint;    // Maxima DSL: -_k__ty_FL_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FL_hip_joint& ForceTransforms::Type_fr_base_X_fr_FL_hip_joint::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FL_thigh_joint::Type_fr_base_X_fr_FL_thigh_joint()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FL_thigh_joint& ForceTransforms::Type_fr_base_X_fr_FL_thigh_joint::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,4) = (- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_FL_thigh_joint;
    (*this)(0,5) =  ty_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(1,1) = sin_q_FL_hip_joint;
    (*this)(1,2) = cos_q_FL_hip_joint;
    (*this)(1,3) =  ty_FL_thigh_joint * sin_q_FL_hip_joint;
    (*this)(1,4) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(1,5) = - tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(2,1) = -cos_q_FL_hip_joint;
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(2,3) = (- ty_FL_thigh_joint * cos_q_FL_hip_joint)- ty_FL_hip_joint;
    (*this)(2,4) =  tx_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(2,5) =  tx_FL_hip_joint * cos_q_FL_hip_joint;
    (*this)(4,4) = sin_q_FL_hip_joint;
    (*this)(4,5) = cos_q_FL_hip_joint;
    (*this)(5,4) = -cos_q_FL_hip_joint;
    (*this)(5,5) = sin_q_FL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FL_calf_joint::Type_fr_base_X_fr_FL_calf_joint()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FL_calf_joint& ForceTransforms::Type_fr_base_X_fr_FL_calf_joint::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    (*this)(0,0) = cos_q_FL_thigh_joint;
    (*this)(0,1) = -sin_q_FL_thigh_joint;
    (*this)(0,3) = ((- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_FL_thigh_joint) * sin_q_FL_thigh_joint;
    (*this)(0,4) = ((- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_FL_thigh_joint) * cos_q_FL_thigh_joint;
    (*this)(0,5) = ( ty_FL_calf_joint * cos_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(1,0) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(1,1) = sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(1,2) = cos_q_FL_hip_joint;
    (*this)(1,3) = ( tx_FL_hip_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+( ty_FL_thigh_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * cos_q_FL_hip_joint);
    (*this)(1,4) = ( tx_FL_hip_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-( ty_FL_thigh_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(1,5) = ( ty_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-( tx_FL_hip_joint * sin_q_FL_hip_joint);
    (*this)(2,0) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(2,1) = -cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(2,3) = ( tx_FL_hip_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(((- ty_FL_thigh_joint * cos_q_FL_hip_joint)- ty_FL_hip_joint) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * sin_q_FL_hip_joint);
    (*this)(2,4) = ((( ty_FL_thigh_joint * cos_q_FL_hip_joint)+ ty_FL_hip_joint) * sin_q_FL_thigh_joint)+( tx_FL_hip_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,5) = ( tx_FL_hip_joint * cos_q_FL_hip_joint)-( ty_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(3,3) = cos_q_FL_thigh_joint;
    (*this)(3,4) = -sin_q_FL_thigh_joint;
    (*this)(4,3) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(4,4) = sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(4,5) = cos_q_FL_hip_joint;
    (*this)(5,3) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(5,4) = -cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(5,5) = sin_q_FL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FR_hip_joint::Type_fr_base_X_fr_FR_hip_joint()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - ty_FR_hip_joint;    // Maxima DSL: -_k__ty_FR_hip_joint
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tx_FR_hip_joint;    // Maxima DSL: _k__tx_FR_hip_joint
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) =  tx_FR_hip_joint;    // Maxima DSL: _k__tx_FR_hip_joint
    (*this)(2,5) = - ty_FR_hip_joint;    // Maxima DSL: -_k__ty_FR_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FR_hip_joint& ForceTransforms::Type_fr_base_X_fr_FR_hip_joint::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FR_thigh_joint::Type_fr_base_X_fr_FR_thigh_joint()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FR_thigh_joint& ForceTransforms::Type_fr_base_X_fr_FR_thigh_joint::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,4) = (- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_FR_thigh_joint;
    (*this)(0,5) =  ty_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(1,1) = sin_q_FR_hip_joint;
    (*this)(1,2) = cos_q_FR_hip_joint;
    (*this)(1,3) =  ty_FR_thigh_joint * sin_q_FR_hip_joint;
    (*this)(1,4) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(1,5) = - tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(2,1) = -cos_q_FR_hip_joint;
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(2,3) = (- ty_FR_thigh_joint * cos_q_FR_hip_joint)- ty_FR_hip_joint;
    (*this)(2,4) =  tx_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(2,5) =  tx_FR_hip_joint * cos_q_FR_hip_joint;
    (*this)(4,4) = sin_q_FR_hip_joint;
    (*this)(4,5) = cos_q_FR_hip_joint;
    (*this)(5,4) = -cos_q_FR_hip_joint;
    (*this)(5,5) = sin_q_FR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_FR_calf_joint::Type_fr_base_X_fr_FR_calf_joint()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_FR_calf_joint& ForceTransforms::Type_fr_base_X_fr_FR_calf_joint::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    (*this)(0,0) = cos_q_FR_thigh_joint;
    (*this)(0,1) = -sin_q_FR_thigh_joint;
    (*this)(0,3) = ((- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_FR_thigh_joint) * sin_q_FR_thigh_joint;
    (*this)(0,4) = ((- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_FR_thigh_joint) * cos_q_FR_thigh_joint;
    (*this)(0,5) = ( ty_FR_calf_joint * cos_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(1,0) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(1,1) = sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(1,2) = cos_q_FR_hip_joint;
    (*this)(1,3) = ( tx_FR_hip_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+( ty_FR_thigh_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * cos_q_FR_hip_joint);
    (*this)(1,4) = ( tx_FR_hip_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-( ty_FR_thigh_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(1,5) = ( ty_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-( tx_FR_hip_joint * sin_q_FR_hip_joint);
    (*this)(2,0) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(2,1) = -cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(2,3) = ( tx_FR_hip_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(((- ty_FR_thigh_joint * cos_q_FR_hip_joint)- ty_FR_hip_joint) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * sin_q_FR_hip_joint);
    (*this)(2,4) = ((( ty_FR_thigh_joint * cos_q_FR_hip_joint)+ ty_FR_hip_joint) * sin_q_FR_thigh_joint)+( tx_FR_hip_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,5) = ( tx_FR_hip_joint * cos_q_FR_hip_joint)-( ty_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(3,3) = cos_q_FR_thigh_joint;
    (*this)(3,4) = -sin_q_FR_thigh_joint;
    (*this)(4,3) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(4,4) = sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(4,5) = cos_q_FR_hip_joint;
    (*this)(5,3) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(5,4) = -cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(5,5) = sin_q_FR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RL_hip_joint::Type_fr_base_X_fr_RL_hip_joint()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - ty_RL_hip_joint;    // Maxima DSL: -_k__ty_RL_hip_joint
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tx_RL_hip_joint;    // Maxima DSL: _k__tx_RL_hip_joint
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) =  tx_RL_hip_joint;    // Maxima DSL: _k__tx_RL_hip_joint
    (*this)(2,5) = - ty_RL_hip_joint;    // Maxima DSL: -_k__ty_RL_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RL_hip_joint& ForceTransforms::Type_fr_base_X_fr_RL_hip_joint::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RL_thigh_joint::Type_fr_base_X_fr_RL_thigh_joint()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RL_thigh_joint& ForceTransforms::Type_fr_base_X_fr_RL_thigh_joint::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,4) = (- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_RL_thigh_joint;
    (*this)(0,5) =  ty_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(1,1) = sin_q_RL_hip_joint;
    (*this)(1,2) = cos_q_RL_hip_joint;
    (*this)(1,3) =  ty_RL_thigh_joint * sin_q_RL_hip_joint;
    (*this)(1,4) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(1,5) = - tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(2,1) = -cos_q_RL_hip_joint;
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(2,3) = (- ty_RL_thigh_joint * cos_q_RL_hip_joint)- ty_RL_hip_joint;
    (*this)(2,4) =  tx_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(2,5) =  tx_RL_hip_joint * cos_q_RL_hip_joint;
    (*this)(4,4) = sin_q_RL_hip_joint;
    (*this)(4,5) = cos_q_RL_hip_joint;
    (*this)(5,4) = -cos_q_RL_hip_joint;
    (*this)(5,5) = sin_q_RL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RL_calf_joint::Type_fr_base_X_fr_RL_calf_joint()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RL_calf_joint& ForceTransforms::Type_fr_base_X_fr_RL_calf_joint::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    (*this)(0,0) = cos_q_RL_thigh_joint;
    (*this)(0,1) = -sin_q_RL_thigh_joint;
    (*this)(0,3) = ((- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_RL_thigh_joint) * sin_q_RL_thigh_joint;
    (*this)(0,4) = ((- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_RL_thigh_joint) * cos_q_RL_thigh_joint;
    (*this)(0,5) = ( ty_RL_calf_joint * cos_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(1,0) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(1,1) = sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(1,2) = cos_q_RL_hip_joint;
    (*this)(1,3) = ( tx_RL_hip_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+( ty_RL_thigh_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * cos_q_RL_hip_joint);
    (*this)(1,4) = ( tx_RL_hip_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-( ty_RL_thigh_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(1,5) = ( ty_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-( tx_RL_hip_joint * sin_q_RL_hip_joint);
    (*this)(2,0) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(2,1) = -cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(2,3) = ( tx_RL_hip_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(((- ty_RL_thigh_joint * cos_q_RL_hip_joint)- ty_RL_hip_joint) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * sin_q_RL_hip_joint);
    (*this)(2,4) = ((( ty_RL_thigh_joint * cos_q_RL_hip_joint)+ ty_RL_hip_joint) * sin_q_RL_thigh_joint)+( tx_RL_hip_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,5) = ( tx_RL_hip_joint * cos_q_RL_hip_joint)-( ty_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(3,3) = cos_q_RL_thigh_joint;
    (*this)(3,4) = -sin_q_RL_thigh_joint;
    (*this)(4,3) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(4,4) = sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(4,5) = cos_q_RL_hip_joint;
    (*this)(5,3) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(5,4) = -cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(5,5) = sin_q_RL_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RR_hip_joint::Type_fr_base_X_fr_RR_hip_joint()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) = - ty_RR_hip_joint;    // Maxima DSL: -_k__ty_RR_hip_joint
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tx_RR_hip_joint;    // Maxima DSL: _k__tx_RR_hip_joint
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) =  tx_RR_hip_joint;    // Maxima DSL: _k__tx_RR_hip_joint
    (*this)(2,5) = - ty_RR_hip_joint;    // Maxima DSL: -_k__ty_RR_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 1.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = -1.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RR_hip_joint& ForceTransforms::Type_fr_base_X_fr_RR_hip_joint::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RR_thigh_joint::Type_fr_base_X_fr_RR_thigh_joint()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RR_thigh_joint& ForceTransforms::Type_fr_base_X_fr_RR_thigh_joint::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,4) = (- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_RR_thigh_joint;
    (*this)(0,5) =  ty_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(1,1) = sin_q_RR_hip_joint;
    (*this)(1,2) = cos_q_RR_hip_joint;
    (*this)(1,3) =  ty_RR_thigh_joint * sin_q_RR_hip_joint;
    (*this)(1,4) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(1,5) = - tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(2,1) = -cos_q_RR_hip_joint;
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(2,3) = (- ty_RR_thigh_joint * cos_q_RR_hip_joint)- ty_RR_hip_joint;
    (*this)(2,4) =  tx_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(2,5) =  tx_RR_hip_joint * cos_q_RR_hip_joint;
    (*this)(4,4) = sin_q_RR_hip_joint;
    (*this)(4,5) = cos_q_RR_hip_joint;
    (*this)(5,4) = -cos_q_RR_hip_joint;
    (*this)(5,5) = sin_q_RR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_base_X_fr_RR_calf_joint::Type_fr_base_X_fr_RR_calf_joint()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_X_fr_RR_calf_joint& ForceTransforms::Type_fr_base_X_fr_RR_calf_joint::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    (*this)(0,0) = cos_q_RR_thigh_joint;
    (*this)(0,1) = -sin_q_RR_thigh_joint;
    (*this)(0,3) = ((- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_RR_thigh_joint) * sin_q_RR_thigh_joint;
    (*this)(0,4) = ((- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_RR_thigh_joint) * cos_q_RR_thigh_joint;
    (*this)(0,5) = ( ty_RR_calf_joint * cos_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(1,0) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(1,1) = sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(1,2) = cos_q_RR_hip_joint;
    (*this)(1,3) = ( tx_RR_hip_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+( ty_RR_thigh_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * cos_q_RR_hip_joint);
    (*this)(1,4) = ( tx_RR_hip_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-( ty_RR_thigh_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(1,5) = ( ty_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-( tx_RR_hip_joint * sin_q_RR_hip_joint);
    (*this)(2,0) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(2,1) = -cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(2,3) = ( tx_RR_hip_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(((- ty_RR_thigh_joint * cos_q_RR_hip_joint)- ty_RR_hip_joint) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * sin_q_RR_hip_joint);
    (*this)(2,4) = ((( ty_RR_thigh_joint * cos_q_RR_hip_joint)+ ty_RR_hip_joint) * sin_q_RR_thigh_joint)+( tx_RR_hip_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,5) = ( tx_RR_hip_joint * cos_q_RR_hip_joint)-( ty_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(3,3) = cos_q_RR_thigh_joint;
    (*this)(3,4) = -sin_q_RR_thigh_joint;
    (*this)(4,3) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(4,4) = sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(4,5) = cos_q_RR_hip_joint;
    (*this)(5,3) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(5,4) = -cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(5,5) = sin_q_RR_hip_joint;
    return *this;
}
ForceTransforms::Type_fr_FR_thigh_X_fr_FR_hip::Type_fr_FR_thigh_X_fr_FR_hip()
{
    (*this)(0,1) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_FR_thigh_X_fr_FR_hip& ForceTransforms::Type_fr_FR_thigh_X_fr_FR_hip::update(const state_t& q)
{
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    (*this)(0,0) = sin_q_FR_thigh_joint;
    (*this)(0,2) = cos_q_FR_thigh_joint;
    (*this)(0,3) =  ty_FR_thigh_joint * cos_q_FR_thigh_joint;
    (*this)(0,5) = - ty_FR_thigh_joint * sin_q_FR_thigh_joint;
    (*this)(1,0) = cos_q_FR_thigh_joint;
    (*this)(1,2) = -sin_q_FR_thigh_joint;
    (*this)(1,3) = - ty_FR_thigh_joint * sin_q_FR_thigh_joint;
    (*this)(1,5) = - ty_FR_thigh_joint * cos_q_FR_thigh_joint;
    (*this)(3,3) = sin_q_FR_thigh_joint;
    (*this)(3,5) = cos_q_FR_thigh_joint;
    (*this)(4,3) = cos_q_FR_thigh_joint;
    (*this)(4,5) = -sin_q_FR_thigh_joint;
    return *this;
}
ForceTransforms::Type_fr_FR_hip_X_fr_FR_thigh::Type_fr_FR_hip_X_fr_FR_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_FR_hip_X_fr_FR_thigh& ForceTransforms::Type_fr_FR_hip_X_fr_FR_thigh::update(const state_t& q)
{
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    (*this)(0,0) = sin_q_FR_thigh_joint;
    (*this)(0,1) = cos_q_FR_thigh_joint;
    (*this)(0,3) =  ty_FR_thigh_joint * cos_q_FR_thigh_joint;
    (*this)(0,4) = - ty_FR_thigh_joint * sin_q_FR_thigh_joint;
    (*this)(2,0) = cos_q_FR_thigh_joint;
    (*this)(2,1) = -sin_q_FR_thigh_joint;
    (*this)(2,3) = - ty_FR_thigh_joint * sin_q_FR_thigh_joint;
    (*this)(2,4) = - ty_FR_thigh_joint * cos_q_FR_thigh_joint;
    (*this)(3,3) = sin_q_FR_thigh_joint;
    (*this)(3,4) = cos_q_FR_thigh_joint;
    (*this)(5,3) = cos_q_FR_thigh_joint;
    (*this)(5,4) = -sin_q_FR_thigh_joint;
    return *this;
}
ForceTransforms::Type_fr_FR_calf_X_fr_FR_thigh::Type_fr_FR_calf_X_fr_FR_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  ty_FR_calf_joint;    // Maxima DSL: _k__ty_FR_calf_joint
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_FR_calf_X_fr_FR_thigh& ForceTransforms::Type_fr_FR_calf_X_fr_FR_thigh::update(const state_t& q)
{
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    (*this)(0,0) = cos_q_FR_calf_joint;
    (*this)(0,1) = sin_q_FR_calf_joint;
    (*this)(0,5) = - ty_FR_calf_joint * cos_q_FR_calf_joint;
    (*this)(1,0) = -sin_q_FR_calf_joint;
    (*this)(1,1) = cos_q_FR_calf_joint;
    (*this)(1,5) =  ty_FR_calf_joint * sin_q_FR_calf_joint;
    (*this)(3,3) = cos_q_FR_calf_joint;
    (*this)(3,4) = sin_q_FR_calf_joint;
    (*this)(4,3) = -sin_q_FR_calf_joint;
    (*this)(4,4) = cos_q_FR_calf_joint;
    return *this;
}
ForceTransforms::Type_fr_FR_thigh_X_fr_FR_calf::Type_fr_FR_thigh_X_fr_FR_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) =  ty_FR_calf_joint;    // Maxima DSL: _k__ty_FR_calf_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_FR_thigh_X_fr_FR_calf& ForceTransforms::Type_fr_FR_thigh_X_fr_FR_calf::update(const state_t& q)
{
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    (*this)(0,0) = cos_q_FR_calf_joint;
    (*this)(0,1) = -sin_q_FR_calf_joint;
    (*this)(1,0) = sin_q_FR_calf_joint;
    (*this)(1,1) = cos_q_FR_calf_joint;
    (*this)(2,3) = - ty_FR_calf_joint * cos_q_FR_calf_joint;
    (*this)(2,4) =  ty_FR_calf_joint * sin_q_FR_calf_joint;
    (*this)(3,3) = cos_q_FR_calf_joint;
    (*this)(3,4) = -sin_q_FR_calf_joint;
    (*this)(4,3) = sin_q_FR_calf_joint;
    (*this)(4,4) = cos_q_FR_calf_joint;
    return *this;
}
ForceTransforms::Type_fr_FL_thigh_X_fr_FL_hip::Type_fr_FL_thigh_X_fr_FL_hip()
{
    (*this)(0,1) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_FL_thigh_X_fr_FL_hip& ForceTransforms::Type_fr_FL_thigh_X_fr_FL_hip::update(const state_t& q)
{
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    (*this)(0,0) = sin_q_FL_thigh_joint;
    (*this)(0,2) = cos_q_FL_thigh_joint;
    (*this)(0,3) =  ty_FL_thigh_joint * cos_q_FL_thigh_joint;
    (*this)(0,5) = - ty_FL_thigh_joint * sin_q_FL_thigh_joint;
    (*this)(1,0) = cos_q_FL_thigh_joint;
    (*this)(1,2) = -sin_q_FL_thigh_joint;
    (*this)(1,3) = - ty_FL_thigh_joint * sin_q_FL_thigh_joint;
    (*this)(1,5) = - ty_FL_thigh_joint * cos_q_FL_thigh_joint;
    (*this)(3,3) = sin_q_FL_thigh_joint;
    (*this)(3,5) = cos_q_FL_thigh_joint;
    (*this)(4,3) = cos_q_FL_thigh_joint;
    (*this)(4,5) = -sin_q_FL_thigh_joint;
    return *this;
}
ForceTransforms::Type_fr_FL_hip_X_fr_FL_thigh::Type_fr_FL_hip_X_fr_FL_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_FL_hip_X_fr_FL_thigh& ForceTransforms::Type_fr_FL_hip_X_fr_FL_thigh::update(const state_t& q)
{
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    (*this)(0,0) = sin_q_FL_thigh_joint;
    (*this)(0,1) = cos_q_FL_thigh_joint;
    (*this)(0,3) =  ty_FL_thigh_joint * cos_q_FL_thigh_joint;
    (*this)(0,4) = - ty_FL_thigh_joint * sin_q_FL_thigh_joint;
    (*this)(2,0) = cos_q_FL_thigh_joint;
    (*this)(2,1) = -sin_q_FL_thigh_joint;
    (*this)(2,3) = - ty_FL_thigh_joint * sin_q_FL_thigh_joint;
    (*this)(2,4) = - ty_FL_thigh_joint * cos_q_FL_thigh_joint;
    (*this)(3,3) = sin_q_FL_thigh_joint;
    (*this)(3,4) = cos_q_FL_thigh_joint;
    (*this)(5,3) = cos_q_FL_thigh_joint;
    (*this)(5,4) = -sin_q_FL_thigh_joint;
    return *this;
}
ForceTransforms::Type_fr_FL_calf_X_fr_FL_thigh::Type_fr_FL_calf_X_fr_FL_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  ty_FL_calf_joint;    // Maxima DSL: _k__ty_FL_calf_joint
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_FL_calf_X_fr_FL_thigh& ForceTransforms::Type_fr_FL_calf_X_fr_FL_thigh::update(const state_t& q)
{
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    (*this)(0,0) = cos_q_FL_calf_joint;
    (*this)(0,1) = sin_q_FL_calf_joint;
    (*this)(0,5) = - ty_FL_calf_joint * cos_q_FL_calf_joint;
    (*this)(1,0) = -sin_q_FL_calf_joint;
    (*this)(1,1) = cos_q_FL_calf_joint;
    (*this)(1,5) =  ty_FL_calf_joint * sin_q_FL_calf_joint;
    (*this)(3,3) = cos_q_FL_calf_joint;
    (*this)(3,4) = sin_q_FL_calf_joint;
    (*this)(4,3) = -sin_q_FL_calf_joint;
    (*this)(4,4) = cos_q_FL_calf_joint;
    return *this;
}
ForceTransforms::Type_fr_FL_thigh_X_fr_FL_calf::Type_fr_FL_thigh_X_fr_FL_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) =  ty_FL_calf_joint;    // Maxima DSL: _k__ty_FL_calf_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_FL_thigh_X_fr_FL_calf& ForceTransforms::Type_fr_FL_thigh_X_fr_FL_calf::update(const state_t& q)
{
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    (*this)(0,0) = cos_q_FL_calf_joint;
    (*this)(0,1) = -sin_q_FL_calf_joint;
    (*this)(1,0) = sin_q_FL_calf_joint;
    (*this)(1,1) = cos_q_FL_calf_joint;
    (*this)(2,3) = - ty_FL_calf_joint * cos_q_FL_calf_joint;
    (*this)(2,4) =  ty_FL_calf_joint * sin_q_FL_calf_joint;
    (*this)(3,3) = cos_q_FL_calf_joint;
    (*this)(3,4) = -sin_q_FL_calf_joint;
    (*this)(4,3) = sin_q_FL_calf_joint;
    (*this)(4,4) = cos_q_FL_calf_joint;
    return *this;
}
ForceTransforms::Type_fr_RR_thigh_X_fr_RR_hip::Type_fr_RR_thigh_X_fr_RR_hip()
{
    (*this)(0,1) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_RR_thigh_X_fr_RR_hip& ForceTransforms::Type_fr_RR_thigh_X_fr_RR_hip::update(const state_t& q)
{
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    (*this)(0,0) = sin_q_RR_thigh_joint;
    (*this)(0,2) = cos_q_RR_thigh_joint;
    (*this)(0,3) =  ty_RR_thigh_joint * cos_q_RR_thigh_joint;
    (*this)(0,5) = - ty_RR_thigh_joint * sin_q_RR_thigh_joint;
    (*this)(1,0) = cos_q_RR_thigh_joint;
    (*this)(1,2) = -sin_q_RR_thigh_joint;
    (*this)(1,3) = - ty_RR_thigh_joint * sin_q_RR_thigh_joint;
    (*this)(1,5) = - ty_RR_thigh_joint * cos_q_RR_thigh_joint;
    (*this)(3,3) = sin_q_RR_thigh_joint;
    (*this)(3,5) = cos_q_RR_thigh_joint;
    (*this)(4,3) = cos_q_RR_thigh_joint;
    (*this)(4,5) = -sin_q_RR_thigh_joint;
    return *this;
}
ForceTransforms::Type_fr_RR_hip_X_fr_RR_thigh::Type_fr_RR_hip_X_fr_RR_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_RR_hip_X_fr_RR_thigh& ForceTransforms::Type_fr_RR_hip_X_fr_RR_thigh::update(const state_t& q)
{
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    (*this)(0,0) = sin_q_RR_thigh_joint;
    (*this)(0,1) = cos_q_RR_thigh_joint;
    (*this)(0,3) =  ty_RR_thigh_joint * cos_q_RR_thigh_joint;
    (*this)(0,4) = - ty_RR_thigh_joint * sin_q_RR_thigh_joint;
    (*this)(2,0) = cos_q_RR_thigh_joint;
    (*this)(2,1) = -sin_q_RR_thigh_joint;
    (*this)(2,3) = - ty_RR_thigh_joint * sin_q_RR_thigh_joint;
    (*this)(2,4) = - ty_RR_thigh_joint * cos_q_RR_thigh_joint;
    (*this)(3,3) = sin_q_RR_thigh_joint;
    (*this)(3,4) = cos_q_RR_thigh_joint;
    (*this)(5,3) = cos_q_RR_thigh_joint;
    (*this)(5,4) = -sin_q_RR_thigh_joint;
    return *this;
}
ForceTransforms::Type_fr_RR_calf_X_fr_RR_thigh::Type_fr_RR_calf_X_fr_RR_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  ty_RR_calf_joint;    // Maxima DSL: _k__ty_RR_calf_joint
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_RR_calf_X_fr_RR_thigh& ForceTransforms::Type_fr_RR_calf_X_fr_RR_thigh::update(const state_t& q)
{
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    (*this)(0,0) = cos_q_RR_calf_joint;
    (*this)(0,1) = sin_q_RR_calf_joint;
    (*this)(0,5) = - ty_RR_calf_joint * cos_q_RR_calf_joint;
    (*this)(1,0) = -sin_q_RR_calf_joint;
    (*this)(1,1) = cos_q_RR_calf_joint;
    (*this)(1,5) =  ty_RR_calf_joint * sin_q_RR_calf_joint;
    (*this)(3,3) = cos_q_RR_calf_joint;
    (*this)(3,4) = sin_q_RR_calf_joint;
    (*this)(4,3) = -sin_q_RR_calf_joint;
    (*this)(4,4) = cos_q_RR_calf_joint;
    return *this;
}
ForceTransforms::Type_fr_RR_thigh_X_fr_RR_calf::Type_fr_RR_thigh_X_fr_RR_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) =  ty_RR_calf_joint;    // Maxima DSL: _k__ty_RR_calf_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_RR_thigh_X_fr_RR_calf& ForceTransforms::Type_fr_RR_thigh_X_fr_RR_calf::update(const state_t& q)
{
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    (*this)(0,0) = cos_q_RR_calf_joint;
    (*this)(0,1) = -sin_q_RR_calf_joint;
    (*this)(1,0) = sin_q_RR_calf_joint;
    (*this)(1,1) = cos_q_RR_calf_joint;
    (*this)(2,3) = - ty_RR_calf_joint * cos_q_RR_calf_joint;
    (*this)(2,4) =  ty_RR_calf_joint * sin_q_RR_calf_joint;
    (*this)(3,3) = cos_q_RR_calf_joint;
    (*this)(3,4) = -sin_q_RR_calf_joint;
    (*this)(4,3) = sin_q_RR_calf_joint;
    (*this)(4,4) = cos_q_RR_calf_joint;
    return *this;
}
ForceTransforms::Type_fr_RL_thigh_X_fr_RL_hip::Type_fr_RL_thigh_X_fr_RL_hip()
{
    (*this)(0,1) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_RL_thigh_X_fr_RL_hip& ForceTransforms::Type_fr_RL_thigh_X_fr_RL_hip::update(const state_t& q)
{
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    (*this)(0,0) = sin_q_RL_thigh_joint;
    (*this)(0,2) = cos_q_RL_thigh_joint;
    (*this)(0,3) =  ty_RL_thigh_joint * cos_q_RL_thigh_joint;
    (*this)(0,5) = - ty_RL_thigh_joint * sin_q_RL_thigh_joint;
    (*this)(1,0) = cos_q_RL_thigh_joint;
    (*this)(1,2) = -sin_q_RL_thigh_joint;
    (*this)(1,3) = - ty_RL_thigh_joint * sin_q_RL_thigh_joint;
    (*this)(1,5) = - ty_RL_thigh_joint * cos_q_RL_thigh_joint;
    (*this)(3,3) = sin_q_RL_thigh_joint;
    (*this)(3,5) = cos_q_RL_thigh_joint;
    (*this)(4,3) = cos_q_RL_thigh_joint;
    (*this)(4,5) = -sin_q_RL_thigh_joint;
    return *this;
}
ForceTransforms::Type_fr_RL_hip_X_fr_RL_thigh::Type_fr_RL_hip_X_fr_RL_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_RL_hip_X_fr_RL_thigh& ForceTransforms::Type_fr_RL_hip_X_fr_RL_thigh::update(const state_t& q)
{
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    (*this)(0,0) = sin_q_RL_thigh_joint;
    (*this)(0,1) = cos_q_RL_thigh_joint;
    (*this)(0,3) =  ty_RL_thigh_joint * cos_q_RL_thigh_joint;
    (*this)(0,4) = - ty_RL_thigh_joint * sin_q_RL_thigh_joint;
    (*this)(2,0) = cos_q_RL_thigh_joint;
    (*this)(2,1) = -sin_q_RL_thigh_joint;
    (*this)(2,3) = - ty_RL_thigh_joint * sin_q_RL_thigh_joint;
    (*this)(2,4) = - ty_RL_thigh_joint * cos_q_RL_thigh_joint;
    (*this)(3,3) = sin_q_RL_thigh_joint;
    (*this)(3,4) = cos_q_RL_thigh_joint;
    (*this)(5,3) = cos_q_RL_thigh_joint;
    (*this)(5,4) = -sin_q_RL_thigh_joint;
    return *this;
}
ForceTransforms::Type_fr_RL_calf_X_fr_RL_thigh::Type_fr_RL_calf_X_fr_RL_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  ty_RL_calf_joint;    // Maxima DSL: _k__ty_RL_calf_joint
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_RL_calf_X_fr_RL_thigh& ForceTransforms::Type_fr_RL_calf_X_fr_RL_thigh::update(const state_t& q)
{
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    (*this)(0,0) = cos_q_RL_calf_joint;
    (*this)(0,1) = sin_q_RL_calf_joint;
    (*this)(0,5) = - ty_RL_calf_joint * cos_q_RL_calf_joint;
    (*this)(1,0) = -sin_q_RL_calf_joint;
    (*this)(1,1) = cos_q_RL_calf_joint;
    (*this)(1,5) =  ty_RL_calf_joint * sin_q_RL_calf_joint;
    (*this)(3,3) = cos_q_RL_calf_joint;
    (*this)(3,4) = sin_q_RL_calf_joint;
    (*this)(4,3) = -sin_q_RL_calf_joint;
    (*this)(4,4) = cos_q_RL_calf_joint;
    return *this;
}
ForceTransforms::Type_fr_RL_thigh_X_fr_RL_calf::Type_fr_RL_thigh_X_fr_RL_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) =  ty_RL_calf_joint;    // Maxima DSL: _k__ty_RL_calf_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_RL_thigh_X_fr_RL_calf& ForceTransforms::Type_fr_RL_thigh_X_fr_RL_calf::update(const state_t& q)
{
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    (*this)(0,0) = cos_q_RL_calf_joint;
    (*this)(0,1) = -sin_q_RL_calf_joint;
    (*this)(1,0) = sin_q_RL_calf_joint;
    (*this)(1,1) = cos_q_RL_calf_joint;
    (*this)(2,3) = - ty_RL_calf_joint * cos_q_RL_calf_joint;
    (*this)(2,4) =  ty_RL_calf_joint * sin_q_RL_calf_joint;
    (*this)(3,3) = cos_q_RL_calf_joint;
    (*this)(3,4) = -sin_q_RL_calf_joint;
    (*this)(4,3) = sin_q_RL_calf_joint;
    (*this)(4,4) = cos_q_RL_calf_joint;
    return *this;
}

HomogeneousTransforms::Type_fr_base_X_fr_FL_calf::Type_fr_base_X_fr_FL_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FL_calf& HomogeneousTransforms::Type_fr_base_X_fr_FL_calf::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    (*this)(0,0) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(0,1) = (-cos_q_FL_calf_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(0,3) =  tx_FL_hip_joint-( ty_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(1,0) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,1) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(1,2) = cos_q_FL_hip_joint;
    (*this)(1,3) = ( ty_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)+( ty_FL_thigh_joint * cos_q_FL_hip_joint)+ ty_FL_hip_joint;
    (*this)(2,0) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,1) = (sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(2,3) = ( ty_FL_thigh_joint * sin_q_FL_hip_joint)-( ty_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_FL_calf_X_fr_base::Type_fr_FL_calf_X_fr_base()
{
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FL_calf_X_fr_base& HomogeneousTransforms::Type_fr_FL_calf_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,0) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(0,1) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(0,2) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(0,3) = ((( tx_FL_hip_joint * sin_q_FL_calf_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * sin_q_FL_calf_joint);
    (*this)(1,0) = (-cos_q_FL_calf_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(1,1) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(1,2) = (sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,3) = ((( ty_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)+( tx_FL_hip_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( tx_FL_hip_joint * sin_q_FL_calf_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * cos_q_FL_calf_joint);
    (*this)(2,1) = cos_q_FL_hip_joint;
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(2,3) = (- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_FL_thigh_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FL_hip::Type_fr_base_X_fr_FL_hip()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_FL_hip_joint;    // Maxima DSL: _k__tx_FL_hip_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_FL_hip_joint;    // Maxima DSL: _k__ty_FL_hip_joint
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FL_hip& HomogeneousTransforms::Type_fr_base_X_fr_FL_hip::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(1,0) = sin_q_FL_hip_joint;
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(2,0) = -cos_q_FL_hip_joint;
    (*this)(2,1) = sin_q_FL_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_FL_hip_X_fr_base::Type_fr_FL_hip_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tx_FL_hip_joint;    // Maxima DSL: -_k__tx_FL_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FL_hip_X_fr_base& HomogeneousTransforms::Type_fr_FL_hip_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,1) = sin_q_FL_hip_joint;
    (*this)(0,2) = -cos_q_FL_hip_joint;
    (*this)(0,3) = - ty_FL_hip_joint * sin_q_FL_hip_joint;
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = sin_q_FL_hip_joint;
    (*this)(1,3) = - ty_FL_hip_joint * cos_q_FL_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FL_thigh::Type_fr_base_X_fr_FL_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_FL_hip_joint;    // Maxima DSL: _k__tx_FL_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FL_thigh& HomogeneousTransforms::Type_fr_base_X_fr_FL_thigh::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    (*this)(0,0) = cos_q_FL_thigh_joint;
    (*this)(0,1) = -sin_q_FL_thigh_joint;
    (*this)(1,0) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(1,1) = sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(1,2) = cos_q_FL_hip_joint;
    (*this)(1,3) = ( ty_FL_thigh_joint * cos_q_FL_hip_joint)+ ty_FL_hip_joint;
    (*this)(2,0) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(2,1) = -cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(2,3) =  ty_FL_thigh_joint * sin_q_FL_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_FL_thigh_X_fr_base::Type_fr_FL_thigh_X_fr_base()
{
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FL_thigh_X_fr_base& HomogeneousTransforms::Type_fr_FL_thigh_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,0) = cos_q_FL_thigh_joint;
    (*this)(0,1) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(0,2) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(0,3) = (- ty_FL_hip_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-( tx_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,0) = -sin_q_FL_thigh_joint;
    (*this)(1,1) = sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(1,2) = -cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(1,3) = ( tx_FL_hip_joint * sin_q_FL_thigh_joint)-( ty_FL_hip_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,1) = cos_q_FL_hip_joint;
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(2,3) = (- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_FL_thigh_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FR_calf::Type_fr_base_X_fr_FR_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FR_calf& HomogeneousTransforms::Type_fr_base_X_fr_FR_calf::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    (*this)(0,0) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(0,1) = (-cos_q_FR_calf_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(0,3) =  tx_FR_hip_joint-( ty_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(1,0) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,1) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(1,2) = cos_q_FR_hip_joint;
    (*this)(1,3) = ( ty_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)+( ty_FR_thigh_joint * cos_q_FR_hip_joint)+ ty_FR_hip_joint;
    (*this)(2,0) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,1) = (sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(2,3) = ( ty_FR_thigh_joint * sin_q_FR_hip_joint)-( ty_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_FR_calf_X_fr_base::Type_fr_FR_calf_X_fr_base()
{
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FR_calf_X_fr_base& HomogeneousTransforms::Type_fr_FR_calf_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,0) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(0,1) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(0,2) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(0,3) = ((( tx_FR_hip_joint * sin_q_FR_calf_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * sin_q_FR_calf_joint);
    (*this)(1,0) = (-cos_q_FR_calf_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(1,1) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(1,2) = (sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,3) = ((( ty_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)+( tx_FR_hip_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( tx_FR_hip_joint * sin_q_FR_calf_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * cos_q_FR_calf_joint);
    (*this)(2,1) = cos_q_FR_hip_joint;
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(2,3) = (- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_FR_thigh_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FR_hip::Type_fr_base_X_fr_FR_hip()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_FR_hip_joint;    // Maxima DSL: _k__tx_FR_hip_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_FR_hip_joint;    // Maxima DSL: _k__ty_FR_hip_joint
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FR_hip& HomogeneousTransforms::Type_fr_base_X_fr_FR_hip::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(1,0) = sin_q_FR_hip_joint;
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(2,0) = -cos_q_FR_hip_joint;
    (*this)(2,1) = sin_q_FR_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_FR_hip_X_fr_base::Type_fr_FR_hip_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tx_FR_hip_joint;    // Maxima DSL: -_k__tx_FR_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FR_hip_X_fr_base& HomogeneousTransforms::Type_fr_FR_hip_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,1) = sin_q_FR_hip_joint;
    (*this)(0,2) = -cos_q_FR_hip_joint;
    (*this)(0,3) = - ty_FR_hip_joint * sin_q_FR_hip_joint;
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = sin_q_FR_hip_joint;
    (*this)(1,3) = - ty_FR_hip_joint * cos_q_FR_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FR_thigh::Type_fr_base_X_fr_FR_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_FR_hip_joint;    // Maxima DSL: _k__tx_FR_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FR_thigh& HomogeneousTransforms::Type_fr_base_X_fr_FR_thigh::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    (*this)(0,0) = cos_q_FR_thigh_joint;
    (*this)(0,1) = -sin_q_FR_thigh_joint;
    (*this)(1,0) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(1,1) = sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(1,2) = cos_q_FR_hip_joint;
    (*this)(1,3) = ( ty_FR_thigh_joint * cos_q_FR_hip_joint)+ ty_FR_hip_joint;
    (*this)(2,0) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(2,1) = -cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(2,3) =  ty_FR_thigh_joint * sin_q_FR_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_FR_thigh_X_fr_base::Type_fr_FR_thigh_X_fr_base()
{
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FR_thigh_X_fr_base& HomogeneousTransforms::Type_fr_FR_thigh_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,0) = cos_q_FR_thigh_joint;
    (*this)(0,1) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(0,2) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(0,3) = (- ty_FR_hip_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-( tx_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,0) = -sin_q_FR_thigh_joint;
    (*this)(1,1) = sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(1,2) = -cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(1,3) = ( tx_FR_hip_joint * sin_q_FR_thigh_joint)-( ty_FR_hip_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,1) = cos_q_FR_hip_joint;
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(2,3) = (- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_FR_thigh_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RL_calf::Type_fr_base_X_fr_RL_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RL_calf& HomogeneousTransforms::Type_fr_base_X_fr_RL_calf::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    (*this)(0,0) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(0,1) = (-cos_q_RL_calf_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(0,3) =  tx_RL_hip_joint-( ty_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(1,0) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,1) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(1,2) = cos_q_RL_hip_joint;
    (*this)(1,3) = ( ty_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)+( ty_RL_thigh_joint * cos_q_RL_hip_joint)+ ty_RL_hip_joint;
    (*this)(2,0) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,1) = (sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(2,3) = ( ty_RL_thigh_joint * sin_q_RL_hip_joint)-( ty_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_RL_calf_X_fr_base::Type_fr_RL_calf_X_fr_base()
{
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RL_calf_X_fr_base& HomogeneousTransforms::Type_fr_RL_calf_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,0) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(0,1) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(0,2) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(0,3) = ((( tx_RL_hip_joint * sin_q_RL_calf_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * sin_q_RL_calf_joint);
    (*this)(1,0) = (-cos_q_RL_calf_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(1,1) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(1,2) = (sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,3) = ((( ty_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)+( tx_RL_hip_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( tx_RL_hip_joint * sin_q_RL_calf_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * cos_q_RL_calf_joint);
    (*this)(2,1) = cos_q_RL_hip_joint;
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(2,3) = (- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_RL_thigh_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RL_hip::Type_fr_base_X_fr_RL_hip()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_RL_hip_joint;    // Maxima DSL: _k__tx_RL_hip_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_RL_hip_joint;    // Maxima DSL: _k__ty_RL_hip_joint
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RL_hip& HomogeneousTransforms::Type_fr_base_X_fr_RL_hip::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(1,0) = sin_q_RL_hip_joint;
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(2,0) = -cos_q_RL_hip_joint;
    (*this)(2,1) = sin_q_RL_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_RL_hip_X_fr_base::Type_fr_RL_hip_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tx_RL_hip_joint;    // Maxima DSL: -_k__tx_RL_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RL_hip_X_fr_base& HomogeneousTransforms::Type_fr_RL_hip_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,1) = sin_q_RL_hip_joint;
    (*this)(0,2) = -cos_q_RL_hip_joint;
    (*this)(0,3) = - ty_RL_hip_joint * sin_q_RL_hip_joint;
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = sin_q_RL_hip_joint;
    (*this)(1,3) = - ty_RL_hip_joint * cos_q_RL_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RL_thigh::Type_fr_base_X_fr_RL_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_RL_hip_joint;    // Maxima DSL: _k__tx_RL_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RL_thigh& HomogeneousTransforms::Type_fr_base_X_fr_RL_thigh::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    (*this)(0,0) = cos_q_RL_thigh_joint;
    (*this)(0,1) = -sin_q_RL_thigh_joint;
    (*this)(1,0) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(1,1) = sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(1,2) = cos_q_RL_hip_joint;
    (*this)(1,3) = ( ty_RL_thigh_joint * cos_q_RL_hip_joint)+ ty_RL_hip_joint;
    (*this)(2,0) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(2,1) = -cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(2,3) =  ty_RL_thigh_joint * sin_q_RL_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_RL_thigh_X_fr_base::Type_fr_RL_thigh_X_fr_base()
{
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RL_thigh_X_fr_base& HomogeneousTransforms::Type_fr_RL_thigh_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,0) = cos_q_RL_thigh_joint;
    (*this)(0,1) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(0,2) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(0,3) = (- ty_RL_hip_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-( tx_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,0) = -sin_q_RL_thigh_joint;
    (*this)(1,1) = sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(1,2) = -cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(1,3) = ( tx_RL_hip_joint * sin_q_RL_thigh_joint)-( ty_RL_hip_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,1) = cos_q_RL_hip_joint;
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(2,3) = (- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_RL_thigh_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RR_calf::Type_fr_base_X_fr_RR_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RR_calf& HomogeneousTransforms::Type_fr_base_X_fr_RR_calf::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    (*this)(0,0) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(0,1) = (-cos_q_RR_calf_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(0,3) =  tx_RR_hip_joint-( ty_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(1,0) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,1) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(1,2) = cos_q_RR_hip_joint;
    (*this)(1,3) = ( ty_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)+( ty_RR_thigh_joint * cos_q_RR_hip_joint)+ ty_RR_hip_joint;
    (*this)(2,0) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,1) = (sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(2,3) = ( ty_RR_thigh_joint * sin_q_RR_hip_joint)-( ty_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_RR_calf_X_fr_base::Type_fr_RR_calf_X_fr_base()
{
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RR_calf_X_fr_base& HomogeneousTransforms::Type_fr_RR_calf_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,0) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(0,1) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(0,2) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(0,3) = ((( tx_RR_hip_joint * sin_q_RR_calf_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * sin_q_RR_calf_joint);
    (*this)(1,0) = (-cos_q_RR_calf_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(1,1) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(1,2) = (sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,3) = ((( ty_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)+( tx_RR_hip_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( tx_RR_hip_joint * sin_q_RR_calf_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * cos_q_RR_calf_joint);
    (*this)(2,1) = cos_q_RR_hip_joint;
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(2,3) = (- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_RR_thigh_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RR_hip::Type_fr_base_X_fr_RR_hip()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_RR_hip_joint;    // Maxima DSL: _k__tx_RR_hip_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_RR_hip_joint;    // Maxima DSL: _k__ty_RR_hip_joint
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RR_hip& HomogeneousTransforms::Type_fr_base_X_fr_RR_hip::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(1,0) = sin_q_RR_hip_joint;
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(2,0) = -cos_q_RR_hip_joint;
    (*this)(2,1) = sin_q_RR_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_RR_hip_X_fr_base::Type_fr_RR_hip_X_fr_base()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tx_RR_hip_joint;    // Maxima DSL: -_k__tx_RR_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RR_hip_X_fr_base& HomogeneousTransforms::Type_fr_RR_hip_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,1) = sin_q_RR_hip_joint;
    (*this)(0,2) = -cos_q_RR_hip_joint;
    (*this)(0,3) = - ty_RR_hip_joint * sin_q_RR_hip_joint;
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = sin_q_RR_hip_joint;
    (*this)(1,3) = - ty_RR_hip_joint * cos_q_RR_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RR_thigh::Type_fr_base_X_fr_RR_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_RR_hip_joint;    // Maxima DSL: _k__tx_RR_hip_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RR_thigh& HomogeneousTransforms::Type_fr_base_X_fr_RR_thigh::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    (*this)(0,0) = cos_q_RR_thigh_joint;
    (*this)(0,1) = -sin_q_RR_thigh_joint;
    (*this)(1,0) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(1,1) = sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(1,2) = cos_q_RR_hip_joint;
    (*this)(1,3) = ( ty_RR_thigh_joint * cos_q_RR_hip_joint)+ ty_RR_hip_joint;
    (*this)(2,0) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(2,1) = -cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(2,3) =  ty_RR_thigh_joint * sin_q_RR_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_RR_thigh_X_fr_base::Type_fr_RR_thigh_X_fr_base()
{
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RR_thigh_X_fr_base& HomogeneousTransforms::Type_fr_RR_thigh_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,0) = cos_q_RR_thigh_joint;
    (*this)(0,1) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(0,2) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(0,3) = (- ty_RR_hip_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-( tx_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,0) = -sin_q_RR_thigh_joint;
    (*this)(1,1) = sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(1,2) = -cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(1,3) = ( tx_RR_hip_joint * sin_q_RR_thigh_joint)-( ty_RR_hip_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,1) = cos_q_RR_hip_joint;
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(2,3) = (- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_RR_thigh_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FL_calf_COM::Type_fr_base_X_fr_FL_calf_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FL_calf_COM& HomogeneousTransforms::Type_fr_base_X_fr_FL_calf_COM::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    (*this)(0,0) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(0,2) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(0,3) = (((- tx_fr_FL_calf_COM * sin_q_FL_calf_joint)-( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)- ty_FL_calf_joint) * sin_q_FL_thigh_joint)+((( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)-( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+ tx_FL_hip_joint;
    (*this)(1,0) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,3) = ((( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)-( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)) * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+((( tx_fr_FL_calf_COM * sin_q_FL_calf_joint)+( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)+( ty_FL_thigh_joint * cos_q_FL_hip_joint)+ ty_FL_hip_joint;
    (*this)(2,0) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,2) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(2,3) = ((( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)-( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)) * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+(((- tx_fr_FL_calf_COM * sin_q_FL_calf_joint)-( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)- ty_FL_calf_joint) * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)+( ty_FL_thigh_joint * sin_q_FL_hip_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_FL_calf_COM_X_fr_base::Type_fr_FL_calf_COM_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FL_calf_COM_X_fr_base& HomogeneousTransforms::Type_fr_FL_calf_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,0) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(0,1) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(0,2) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(0,3) = ((( tx_FL_hip_joint * sin_q_FL_calf_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * sin_q_FL_calf_joint)- tx_fr_FL_calf_COM;
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = sin_q_FL_hip_joint;
    (*this)(1,3) = (- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_FL_thigh_joint;
    (*this)(2,0) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(2,1) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,2) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(2,3) = (((- ty_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+( ty_FL_calf_joint * cos_q_FL_calf_joint)+ ty_fr_FL_calf_COM;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FL_foot::Type_fr_base_X_fr_FL_foot()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FL_foot& HomogeneousTransforms::Type_fr_base_X_fr_FL_foot::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    (*this)(0,0) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(0,2) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(0,3) = (((- ty_fr_FL_foot * cos_q_FL_calf_joint)- ty_FL_calf_joint) * sin_q_FL_thigh_joint)-( ty_fr_FL_foot * sin_q_FL_calf_joint * cos_q_FL_thigh_joint)+ tx_FL_hip_joint;
    (*this)(1,0) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(1,3) = (- ty_fr_FL_foot * sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+((( ty_fr_FL_foot * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)+( ty_FL_thigh_joint * cos_q_FL_hip_joint)+ ty_FL_hip_joint;
    (*this)(2,0) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,2) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(2,3) = ( ty_fr_FL_foot * sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+(((- ty_fr_FL_foot * cos_q_FL_calf_joint)- ty_FL_calf_joint) * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)+( ty_FL_thigh_joint * sin_q_FL_hip_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_FL_foot_X_fr_base::Type_fr_FL_foot_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FL_foot_X_fr_base& HomogeneousTransforms::Type_fr_FL_foot_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,0) = (cos_q_FL_calf_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(0,1) = (cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(0,2) = (-cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(0,3) = ((( tx_FL_hip_joint * sin_q_FL_calf_joint)-( ty_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)) * sin_q_FL_thigh_joint)+(((- ty_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint)-( ty_FL_calf_joint * sin_q_FL_calf_joint);
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = sin_q_FL_hip_joint;
    (*this)(1,3) = (- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_FL_thigh_joint;
    (*this)(2,0) = (cos_q_FL_calf_joint * sin_q_FL_thigh_joint)+(sin_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(2,1) = (sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-(cos_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(2,2) = (cos_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-(sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(2,3) = (((- ty_FL_hip_joint * sin_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+((( ty_FL_hip_joint * cos_q_FL_calf_joint * sin_q_FL_hip_joint)-( tx_FL_hip_joint * sin_q_FL_calf_joint)) * cos_q_FL_thigh_joint)+( ty_FL_calf_joint * cos_q_FL_calf_joint)+ ty_fr_FL_foot;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FL_hip_COM::Type_fr_base_X_fr_FL_hip_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tz_fr_FL_hip_COM+ tx_FL_hip_joint;    // Maxima DSL: _k__tz_fr_FL_hip_COM+_k__tx_FL_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FL_hip_COM& HomogeneousTransforms::Type_fr_base_X_fr_FL_hip_COM::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = -sin_q_FL_hip_joint;
    (*this)(1,3) = ( tx_fr_FL_hip_COM * sin_q_FL_hip_joint)+( ty_fr_FL_hip_COM * cos_q_FL_hip_joint)+ ty_FL_hip_joint;
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,2) = cos_q_FL_hip_joint;
    (*this)(2,3) = ( ty_fr_FL_hip_COM * sin_q_FL_hip_joint)-( tx_fr_FL_hip_COM * cos_q_FL_hip_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_FL_hip_COM_X_fr_base::Type_fr_FL_hip_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = - tz_fr_FL_hip_COM- tx_FL_hip_joint;    // Maxima DSL: (-_k__tz_fr_FL_hip_COM)-_k__tx_FL_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FL_hip_COM_X_fr_base& HomogeneousTransforms::Type_fr_FL_hip_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = sin_q_FL_hip_joint;
    (*this)(1,3) = (- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_fr_FL_hip_COM;
    (*this)(2,1) = -sin_q_FL_hip_joint;
    (*this)(2,2) = cos_q_FL_hip_joint;
    (*this)(2,3) = ( ty_FL_hip_joint * sin_q_FL_hip_joint)+ tx_fr_FL_hip_COM;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FL_thigh_COM::Type_fr_base_X_fr_FL_thigh_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FL_thigh_COM& HomogeneousTransforms::Type_fr_base_X_fr_FL_thigh_COM::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    (*this)(0,0) = cos_q_FL_thigh_joint;
    (*this)(0,2) = sin_q_FL_thigh_joint;
    (*this)(0,3) = (- ty_fr_FL_thigh_COM * sin_q_FL_thigh_joint)+( tx_fr_FL_thigh_COM * cos_q_FL_thigh_joint)+ tx_FL_hip_joint;
    (*this)(1,0) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = -sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(1,3) = ( tx_fr_FL_thigh_COM * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+( ty_fr_FL_thigh_COM * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)+(( tz_fr_FL_thigh_COM+ ty_FL_thigh_joint) * cos_q_FL_hip_joint)+ ty_FL_hip_joint;
    (*this)(2,0) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,2) = cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(2,3) = (- tx_fr_FL_thigh_COM * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-( ty_fr_FL_thigh_COM * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)+(( tz_fr_FL_thigh_COM+ ty_FL_thigh_joint) * sin_q_FL_hip_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_FL_thigh_COM_X_fr_base::Type_fr_FL_thigh_COM_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FL_thigh_COM_X_fr_base& HomogeneousTransforms::Type_fr_FL_thigh_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(0,0) = cos_q_FL_thigh_joint;
    (*this)(0,1) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(0,2) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(0,3) = (- ty_FL_hip_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-( tx_FL_hip_joint * cos_q_FL_thigh_joint)- tx_fr_FL_thigh_COM;
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = sin_q_FL_hip_joint;
    (*this)(1,3) = (- ty_FL_hip_joint * cos_q_FL_hip_joint)- tz_fr_FL_thigh_COM- ty_FL_thigh_joint;
    (*this)(2,0) = sin_q_FL_thigh_joint;
    (*this)(2,1) = -sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(2,2) = cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(2,3) = (- tx_FL_hip_joint * sin_q_FL_thigh_joint)+( ty_FL_hip_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)+ ty_fr_FL_thigh_COM;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FL_thigh_shoulder::Type_fr_base_X_fr_FL_thigh_shoulder()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_FL_hip_joint;    // Maxima DSL: _k__tx_FL_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FL_thigh_shoulder& HomogeneousTransforms::Type_fr_base_X_fr_FL_thigh_shoulder::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = -sin_q_FL_hip_joint;
    (*this)(1,3) = ( ty_fr_FL_thigh_shoulder * cos_q_FL_hip_joint)+ ty_FL_hip_joint;
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,2) = cos_q_FL_hip_joint;
    (*this)(2,3) =  ty_fr_FL_thigh_shoulder * sin_q_FL_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_FL_thigh_shoulder_X_fr_base::Type_fr_FL_thigh_shoulder_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = - tx_FL_hip_joint;    // Maxima DSL: -_k__tx_FL_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FL_thigh_shoulder_X_fr_base& HomogeneousTransforms::Type_fr_FL_thigh_shoulder_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = sin_q_FL_hip_joint;
    (*this)(1,3) = (- ty_FL_hip_joint * cos_q_FL_hip_joint)- ty_fr_FL_thigh_shoulder;
    (*this)(2,1) = -sin_q_FL_hip_joint;
    (*this)(2,2) = cos_q_FL_hip_joint;
    (*this)(2,3) =  ty_FL_hip_joint * sin_q_FL_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FR_calf_COM::Type_fr_base_X_fr_FR_calf_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FR_calf_COM& HomogeneousTransforms::Type_fr_base_X_fr_FR_calf_COM::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    (*this)(0,0) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(0,2) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(0,3) = (((- tx_fr_FR_calf_COM * sin_q_FR_calf_joint)-( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)- ty_FR_calf_joint) * sin_q_FR_thigh_joint)+((( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)-( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+ tx_FR_hip_joint;
    (*this)(1,0) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,3) = ((( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)-( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)) * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+((( tx_fr_FR_calf_COM * sin_q_FR_calf_joint)+( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)+( ty_FR_thigh_joint * cos_q_FR_hip_joint)+ ty_FR_hip_joint;
    (*this)(2,0) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,2) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(2,3) = ((( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)-( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)) * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+(((- tx_fr_FR_calf_COM * sin_q_FR_calf_joint)-( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)- ty_FR_calf_joint) * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)+( ty_FR_thigh_joint * sin_q_FR_hip_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_FR_calf_COM_X_fr_base::Type_fr_FR_calf_COM_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FR_calf_COM_X_fr_base& HomogeneousTransforms::Type_fr_FR_calf_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,0) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(0,1) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(0,2) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(0,3) = ((( tx_FR_hip_joint * sin_q_FR_calf_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * sin_q_FR_calf_joint)- tx_fr_FR_calf_COM;
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = sin_q_FR_hip_joint;
    (*this)(1,3) = (- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_FR_thigh_joint;
    (*this)(2,0) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(2,1) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,2) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(2,3) = (((- ty_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+( ty_FR_calf_joint * cos_q_FR_calf_joint)+ ty_fr_FR_calf_COM;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FR_foot::Type_fr_base_X_fr_FR_foot()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FR_foot& HomogeneousTransforms::Type_fr_base_X_fr_FR_foot::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    (*this)(0,0) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(0,2) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(0,3) = (((- ty_fr_FR_foot * cos_q_FR_calf_joint)- ty_FR_calf_joint) * sin_q_FR_thigh_joint)-( ty_fr_FR_foot * sin_q_FR_calf_joint * cos_q_FR_thigh_joint)+ tx_FR_hip_joint;
    (*this)(1,0) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(1,3) = (- ty_fr_FR_foot * sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+((( ty_fr_FR_foot * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)+( ty_FR_thigh_joint * cos_q_FR_hip_joint)+ ty_FR_hip_joint;
    (*this)(2,0) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,2) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(2,3) = ( ty_fr_FR_foot * sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+(((- ty_fr_FR_foot * cos_q_FR_calf_joint)- ty_FR_calf_joint) * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)+( ty_FR_thigh_joint * sin_q_FR_hip_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_FR_foot_X_fr_base::Type_fr_FR_foot_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FR_foot_X_fr_base& HomogeneousTransforms::Type_fr_FR_foot_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,0) = (cos_q_FR_calf_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(0,1) = (cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(0,2) = (-cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(0,3) = ((( tx_FR_hip_joint * sin_q_FR_calf_joint)-( ty_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)) * sin_q_FR_thigh_joint)+(((- ty_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint)-( ty_FR_calf_joint * sin_q_FR_calf_joint);
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = sin_q_FR_hip_joint;
    (*this)(1,3) = (- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_FR_thigh_joint;
    (*this)(2,0) = (cos_q_FR_calf_joint * sin_q_FR_thigh_joint)+(sin_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(2,1) = (sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-(cos_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(2,2) = (cos_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-(sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(2,3) = (((- ty_FR_hip_joint * sin_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+((( ty_FR_hip_joint * cos_q_FR_calf_joint * sin_q_FR_hip_joint)-( tx_FR_hip_joint * sin_q_FR_calf_joint)) * cos_q_FR_thigh_joint)+( ty_FR_calf_joint * cos_q_FR_calf_joint)+ ty_fr_FR_foot;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FR_hip_COM::Type_fr_base_X_fr_FR_hip_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tz_fr_FR_hip_COM+ tx_FR_hip_joint;    // Maxima DSL: _k__tz_fr_FR_hip_COM+_k__tx_FR_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FR_hip_COM& HomogeneousTransforms::Type_fr_base_X_fr_FR_hip_COM::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = -sin_q_FR_hip_joint;
    (*this)(1,3) = ( tx_fr_FR_hip_COM * sin_q_FR_hip_joint)+( ty_fr_FR_hip_COM * cos_q_FR_hip_joint)+ ty_FR_hip_joint;
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,2) = cos_q_FR_hip_joint;
    (*this)(2,3) = ( ty_fr_FR_hip_COM * sin_q_FR_hip_joint)-( tx_fr_FR_hip_COM * cos_q_FR_hip_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_FR_hip_COM_X_fr_base::Type_fr_FR_hip_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = - tz_fr_FR_hip_COM- tx_FR_hip_joint;    // Maxima DSL: (-_k__tz_fr_FR_hip_COM)-_k__tx_FR_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FR_hip_COM_X_fr_base& HomogeneousTransforms::Type_fr_FR_hip_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = sin_q_FR_hip_joint;
    (*this)(1,3) = (- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_fr_FR_hip_COM;
    (*this)(2,1) = -sin_q_FR_hip_joint;
    (*this)(2,2) = cos_q_FR_hip_joint;
    (*this)(2,3) = ( ty_FR_hip_joint * sin_q_FR_hip_joint)+ tx_fr_FR_hip_COM;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FR_thigh_COM::Type_fr_base_X_fr_FR_thigh_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FR_thigh_COM& HomogeneousTransforms::Type_fr_base_X_fr_FR_thigh_COM::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    (*this)(0,0) = cos_q_FR_thigh_joint;
    (*this)(0,2) = sin_q_FR_thigh_joint;
    (*this)(0,3) = (- ty_fr_FR_thigh_COM * sin_q_FR_thigh_joint)+( tx_fr_FR_thigh_COM * cos_q_FR_thigh_joint)+ tx_FR_hip_joint;
    (*this)(1,0) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = -sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(1,3) = ( tx_fr_FR_thigh_COM * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+( ty_fr_FR_thigh_COM * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)+(( tz_fr_FR_thigh_COM+ ty_FR_thigh_joint) * cos_q_FR_hip_joint)+ ty_FR_hip_joint;
    (*this)(2,0) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,2) = cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(2,3) = (- tx_fr_FR_thigh_COM * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-( ty_fr_FR_thigh_COM * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)+(( tz_fr_FR_thigh_COM+ ty_FR_thigh_joint) * sin_q_FR_hip_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_FR_thigh_COM_X_fr_base::Type_fr_FR_thigh_COM_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FR_thigh_COM_X_fr_base& HomogeneousTransforms::Type_fr_FR_thigh_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(0,0) = cos_q_FR_thigh_joint;
    (*this)(0,1) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(0,2) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(0,3) = (- ty_FR_hip_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-( tx_FR_hip_joint * cos_q_FR_thigh_joint)- tx_fr_FR_thigh_COM;
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = sin_q_FR_hip_joint;
    (*this)(1,3) = (- ty_FR_hip_joint * cos_q_FR_hip_joint)- tz_fr_FR_thigh_COM- ty_FR_thigh_joint;
    (*this)(2,0) = sin_q_FR_thigh_joint;
    (*this)(2,1) = -sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(2,2) = cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(2,3) = (- tx_FR_hip_joint * sin_q_FR_thigh_joint)+( ty_FR_hip_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)+ ty_fr_FR_thigh_COM;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FR_thigh_shoulder::Type_fr_base_X_fr_FR_thigh_shoulder()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_FR_hip_joint;    // Maxima DSL: _k__tx_FR_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FR_thigh_shoulder& HomogeneousTransforms::Type_fr_base_X_fr_FR_thigh_shoulder::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = -sin_q_FR_hip_joint;
    (*this)(1,3) = ( ty_fr_FR_thigh_shoulder * cos_q_FR_hip_joint)+ ty_FR_hip_joint;
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,2) = cos_q_FR_hip_joint;
    (*this)(2,3) =  ty_fr_FR_thigh_shoulder * sin_q_FR_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_FR_thigh_shoulder_X_fr_base::Type_fr_FR_thigh_shoulder_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = - tx_FR_hip_joint;    // Maxima DSL: -_k__tx_FR_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FR_thigh_shoulder_X_fr_base& HomogeneousTransforms::Type_fr_FR_thigh_shoulder_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = sin_q_FR_hip_joint;
    (*this)(1,3) = (- ty_FR_hip_joint * cos_q_FR_hip_joint)- ty_fr_FR_thigh_shoulder;
    (*this)(2,1) = -sin_q_FR_hip_joint;
    (*this)(2,2) = cos_q_FR_hip_joint;
    (*this)(2,3) =  ty_FR_hip_joint * sin_q_FR_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RL_calf_COM::Type_fr_base_X_fr_RL_calf_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RL_calf_COM& HomogeneousTransforms::Type_fr_base_X_fr_RL_calf_COM::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    (*this)(0,0) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(0,2) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(0,3) = (((- tx_fr_RL_calf_COM * sin_q_RL_calf_joint)-( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)- ty_RL_calf_joint) * sin_q_RL_thigh_joint)+((( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)-( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+ tx_RL_hip_joint;
    (*this)(1,0) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,3) = ((( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)-( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)) * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+((( tx_fr_RL_calf_COM * sin_q_RL_calf_joint)+( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)+( ty_RL_thigh_joint * cos_q_RL_hip_joint)+ ty_RL_hip_joint;
    (*this)(2,0) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,2) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(2,3) = ((( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)-( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)) * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+(((- tx_fr_RL_calf_COM * sin_q_RL_calf_joint)-( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)- ty_RL_calf_joint) * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)+( ty_RL_thigh_joint * sin_q_RL_hip_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_RL_calf_COM_X_fr_base::Type_fr_RL_calf_COM_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RL_calf_COM_X_fr_base& HomogeneousTransforms::Type_fr_RL_calf_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,0) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(0,1) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(0,2) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(0,3) = ((( tx_RL_hip_joint * sin_q_RL_calf_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * sin_q_RL_calf_joint)- tx_fr_RL_calf_COM;
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = sin_q_RL_hip_joint;
    (*this)(1,3) = (- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_RL_thigh_joint;
    (*this)(2,0) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(2,1) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,2) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(2,3) = (((- ty_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+( ty_RL_calf_joint * cos_q_RL_calf_joint)+ ty_fr_RL_calf_COM;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RL_foot::Type_fr_base_X_fr_RL_foot()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RL_foot& HomogeneousTransforms::Type_fr_base_X_fr_RL_foot::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    (*this)(0,0) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(0,2) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(0,3) = (((- ty_fr_RL_foot * cos_q_RL_calf_joint)- ty_RL_calf_joint) * sin_q_RL_thigh_joint)-( ty_fr_RL_foot * sin_q_RL_calf_joint * cos_q_RL_thigh_joint)+ tx_RL_hip_joint;
    (*this)(1,0) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(1,3) = (- ty_fr_RL_foot * sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+((( ty_fr_RL_foot * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)+( ty_RL_thigh_joint * cos_q_RL_hip_joint)+ ty_RL_hip_joint;
    (*this)(2,0) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,2) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(2,3) = ( ty_fr_RL_foot * sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+(((- ty_fr_RL_foot * cos_q_RL_calf_joint)- ty_RL_calf_joint) * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)+( ty_RL_thigh_joint * sin_q_RL_hip_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_RL_foot_X_fr_base::Type_fr_RL_foot_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RL_foot_X_fr_base& HomogeneousTransforms::Type_fr_RL_foot_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,0) = (cos_q_RL_calf_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(0,1) = (cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(0,2) = (-cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(0,3) = ((( tx_RL_hip_joint * sin_q_RL_calf_joint)-( ty_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)) * sin_q_RL_thigh_joint)+(((- ty_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint)-( ty_RL_calf_joint * sin_q_RL_calf_joint);
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = sin_q_RL_hip_joint;
    (*this)(1,3) = (- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_RL_thigh_joint;
    (*this)(2,0) = (cos_q_RL_calf_joint * sin_q_RL_thigh_joint)+(sin_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(2,1) = (sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-(cos_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(2,2) = (cos_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-(sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(2,3) = (((- ty_RL_hip_joint * sin_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+((( ty_RL_hip_joint * cos_q_RL_calf_joint * sin_q_RL_hip_joint)-( tx_RL_hip_joint * sin_q_RL_calf_joint)) * cos_q_RL_thigh_joint)+( ty_RL_calf_joint * cos_q_RL_calf_joint)+ ty_fr_RL_foot;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RL_hip_COM::Type_fr_base_X_fr_RL_hip_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tz_fr_RL_hip_COM+ tx_RL_hip_joint;    // Maxima DSL: _k__tz_fr_RL_hip_COM+_k__tx_RL_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RL_hip_COM& HomogeneousTransforms::Type_fr_base_X_fr_RL_hip_COM::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = -sin_q_RL_hip_joint;
    (*this)(1,3) = ( tx_fr_RL_hip_COM * sin_q_RL_hip_joint)+( ty_fr_RL_hip_COM * cos_q_RL_hip_joint)+ ty_RL_hip_joint;
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,2) = cos_q_RL_hip_joint;
    (*this)(2,3) = ( ty_fr_RL_hip_COM * sin_q_RL_hip_joint)-( tx_fr_RL_hip_COM * cos_q_RL_hip_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_RL_hip_COM_X_fr_base::Type_fr_RL_hip_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = - tz_fr_RL_hip_COM- tx_RL_hip_joint;    // Maxima DSL: (-_k__tz_fr_RL_hip_COM)-_k__tx_RL_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RL_hip_COM_X_fr_base& HomogeneousTransforms::Type_fr_RL_hip_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = sin_q_RL_hip_joint;
    (*this)(1,3) = (- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_fr_RL_hip_COM;
    (*this)(2,1) = -sin_q_RL_hip_joint;
    (*this)(2,2) = cos_q_RL_hip_joint;
    (*this)(2,3) = ( ty_RL_hip_joint * sin_q_RL_hip_joint)+ tx_fr_RL_hip_COM;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RL_thigh_COM::Type_fr_base_X_fr_RL_thigh_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RL_thigh_COM& HomogeneousTransforms::Type_fr_base_X_fr_RL_thigh_COM::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    (*this)(0,0) = cos_q_RL_thigh_joint;
    (*this)(0,2) = sin_q_RL_thigh_joint;
    (*this)(0,3) = (- ty_fr_RL_thigh_COM * sin_q_RL_thigh_joint)+( tx_fr_RL_thigh_COM * cos_q_RL_thigh_joint)+ tx_RL_hip_joint;
    (*this)(1,0) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = -sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(1,3) = ( tx_fr_RL_thigh_COM * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+( ty_fr_RL_thigh_COM * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)+(( tz_fr_RL_thigh_COM+ ty_RL_thigh_joint) * cos_q_RL_hip_joint)+ ty_RL_hip_joint;
    (*this)(2,0) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,2) = cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(2,3) = (- tx_fr_RL_thigh_COM * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-( ty_fr_RL_thigh_COM * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)+(( tz_fr_RL_thigh_COM+ ty_RL_thigh_joint) * sin_q_RL_hip_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_RL_thigh_COM_X_fr_base::Type_fr_RL_thigh_COM_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RL_thigh_COM_X_fr_base& HomogeneousTransforms::Type_fr_RL_thigh_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(0,0) = cos_q_RL_thigh_joint;
    (*this)(0,1) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(0,2) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(0,3) = (- ty_RL_hip_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-( tx_RL_hip_joint * cos_q_RL_thigh_joint)- tx_fr_RL_thigh_COM;
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = sin_q_RL_hip_joint;
    (*this)(1,3) = (- ty_RL_hip_joint * cos_q_RL_hip_joint)- tz_fr_RL_thigh_COM- ty_RL_thigh_joint;
    (*this)(2,0) = sin_q_RL_thigh_joint;
    (*this)(2,1) = -sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(2,2) = cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(2,3) = (- tx_RL_hip_joint * sin_q_RL_thigh_joint)+( ty_RL_hip_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)+ ty_fr_RL_thigh_COM;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RL_thigh_shoulder::Type_fr_base_X_fr_RL_thigh_shoulder()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_RL_hip_joint;    // Maxima DSL: _k__tx_RL_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RL_thigh_shoulder& HomogeneousTransforms::Type_fr_base_X_fr_RL_thigh_shoulder::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = -sin_q_RL_hip_joint;
    (*this)(1,3) = ( ty_fr_RL_thigh_shoulder * cos_q_RL_hip_joint)+ ty_RL_hip_joint;
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,2) = cos_q_RL_hip_joint;
    (*this)(2,3) =  ty_fr_RL_thigh_shoulder * sin_q_RL_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_RL_thigh_shoulder_X_fr_base::Type_fr_RL_thigh_shoulder_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = - tx_RL_hip_joint;    // Maxima DSL: -_k__tx_RL_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RL_thigh_shoulder_X_fr_base& HomogeneousTransforms::Type_fr_RL_thigh_shoulder_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = sin_q_RL_hip_joint;
    (*this)(1,3) = (- ty_RL_hip_joint * cos_q_RL_hip_joint)- ty_fr_RL_thigh_shoulder;
    (*this)(2,1) = -sin_q_RL_hip_joint;
    (*this)(2,2) = cos_q_RL_hip_joint;
    (*this)(2,3) =  ty_RL_hip_joint * sin_q_RL_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RR_calf_COM::Type_fr_base_X_fr_RR_calf_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RR_calf_COM& HomogeneousTransforms::Type_fr_base_X_fr_RR_calf_COM::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    (*this)(0,0) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(0,2) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(0,3) = (((- tx_fr_RR_calf_COM * sin_q_RR_calf_joint)-( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)- ty_RR_calf_joint) * sin_q_RR_thigh_joint)+((( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)-( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+ tx_RR_hip_joint;
    (*this)(1,0) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,3) = ((( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)-( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)) * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+((( tx_fr_RR_calf_COM * sin_q_RR_calf_joint)+( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)+( ty_RR_thigh_joint * cos_q_RR_hip_joint)+ ty_RR_hip_joint;
    (*this)(2,0) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,2) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(2,3) = ((( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)-( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)) * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+(((- tx_fr_RR_calf_COM * sin_q_RR_calf_joint)-( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)- ty_RR_calf_joint) * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)+( ty_RR_thigh_joint * sin_q_RR_hip_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_RR_calf_COM_X_fr_base::Type_fr_RR_calf_COM_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RR_calf_COM_X_fr_base& HomogeneousTransforms::Type_fr_RR_calf_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,0) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(0,1) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(0,2) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(0,3) = ((( tx_RR_hip_joint * sin_q_RR_calf_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * sin_q_RR_calf_joint)- tx_fr_RR_calf_COM;
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = sin_q_RR_hip_joint;
    (*this)(1,3) = (- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_RR_thigh_joint;
    (*this)(2,0) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(2,1) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,2) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(2,3) = (((- ty_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+( ty_RR_calf_joint * cos_q_RR_calf_joint)+ ty_fr_RR_calf_COM;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RR_foot::Type_fr_base_X_fr_RR_foot()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RR_foot& HomogeneousTransforms::Type_fr_base_X_fr_RR_foot::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    (*this)(0,0) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(0,2) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(0,3) = (((- ty_fr_RR_foot * cos_q_RR_calf_joint)- ty_RR_calf_joint) * sin_q_RR_thigh_joint)-( ty_fr_RR_foot * sin_q_RR_calf_joint * cos_q_RR_thigh_joint)+ tx_RR_hip_joint;
    (*this)(1,0) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(1,3) = (- ty_fr_RR_foot * sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+((( ty_fr_RR_foot * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)+( ty_RR_thigh_joint * cos_q_RR_hip_joint)+ ty_RR_hip_joint;
    (*this)(2,0) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,2) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(2,3) = ( ty_fr_RR_foot * sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+(((- ty_fr_RR_foot * cos_q_RR_calf_joint)- ty_RR_calf_joint) * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)+( ty_RR_thigh_joint * sin_q_RR_hip_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_RR_foot_X_fr_base::Type_fr_RR_foot_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RR_foot_X_fr_base& HomogeneousTransforms::Type_fr_RR_foot_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,0) = (cos_q_RR_calf_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(0,1) = (cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(0,2) = (-cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(0,3) = ((( tx_RR_hip_joint * sin_q_RR_calf_joint)-( ty_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)) * sin_q_RR_thigh_joint)+(((- ty_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint)-( ty_RR_calf_joint * sin_q_RR_calf_joint);
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = sin_q_RR_hip_joint;
    (*this)(1,3) = (- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_RR_thigh_joint;
    (*this)(2,0) = (cos_q_RR_calf_joint * sin_q_RR_thigh_joint)+(sin_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(2,1) = (sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-(cos_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(2,2) = (cos_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-(sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(2,3) = (((- ty_RR_hip_joint * sin_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+((( ty_RR_hip_joint * cos_q_RR_calf_joint * sin_q_RR_hip_joint)-( tx_RR_hip_joint * sin_q_RR_calf_joint)) * cos_q_RR_thigh_joint)+( ty_RR_calf_joint * cos_q_RR_calf_joint)+ ty_fr_RR_foot;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RR_hip_COM::Type_fr_base_X_fr_RR_hip_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tz_fr_RR_hip_COM+ tx_RR_hip_joint;    // Maxima DSL: _k__tz_fr_RR_hip_COM+_k__tx_RR_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RR_hip_COM& HomogeneousTransforms::Type_fr_base_X_fr_RR_hip_COM::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = -sin_q_RR_hip_joint;
    (*this)(1,3) = ( tx_fr_RR_hip_COM * sin_q_RR_hip_joint)+( ty_fr_RR_hip_COM * cos_q_RR_hip_joint)+ ty_RR_hip_joint;
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,2) = cos_q_RR_hip_joint;
    (*this)(2,3) = ( ty_fr_RR_hip_COM * sin_q_RR_hip_joint)-( tx_fr_RR_hip_COM * cos_q_RR_hip_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_RR_hip_COM_X_fr_base::Type_fr_RR_hip_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = - tz_fr_RR_hip_COM- tx_RR_hip_joint;    // Maxima DSL: (-_k__tz_fr_RR_hip_COM)-_k__tx_RR_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RR_hip_COM_X_fr_base& HomogeneousTransforms::Type_fr_RR_hip_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = sin_q_RR_hip_joint;
    (*this)(1,3) = (- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_fr_RR_hip_COM;
    (*this)(2,1) = -sin_q_RR_hip_joint;
    (*this)(2,2) = cos_q_RR_hip_joint;
    (*this)(2,3) = ( ty_RR_hip_joint * sin_q_RR_hip_joint)+ tx_fr_RR_hip_COM;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RR_thigh_COM::Type_fr_base_X_fr_RR_thigh_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RR_thigh_COM& HomogeneousTransforms::Type_fr_base_X_fr_RR_thigh_COM::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    (*this)(0,0) = cos_q_RR_thigh_joint;
    (*this)(0,2) = sin_q_RR_thigh_joint;
    (*this)(0,3) = (- ty_fr_RR_thigh_COM * sin_q_RR_thigh_joint)+( tx_fr_RR_thigh_COM * cos_q_RR_thigh_joint)+ tx_RR_hip_joint;
    (*this)(1,0) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = -sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(1,3) = ( tx_fr_RR_thigh_COM * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+( ty_fr_RR_thigh_COM * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)+(( tz_fr_RR_thigh_COM+ ty_RR_thigh_joint) * cos_q_RR_hip_joint)+ ty_RR_hip_joint;
    (*this)(2,0) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,2) = cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(2,3) = (- tx_fr_RR_thigh_COM * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-( ty_fr_RR_thigh_COM * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)+(( tz_fr_RR_thigh_COM+ ty_RR_thigh_joint) * sin_q_RR_hip_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_RR_thigh_COM_X_fr_base::Type_fr_RR_thigh_COM_X_fr_base()
{
    (*this)(1,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RR_thigh_COM_X_fr_base& HomogeneousTransforms::Type_fr_RR_thigh_COM_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(0,0) = cos_q_RR_thigh_joint;
    (*this)(0,1) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(0,2) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(0,3) = (- ty_RR_hip_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-( tx_RR_hip_joint * cos_q_RR_thigh_joint)- tx_fr_RR_thigh_COM;
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = sin_q_RR_hip_joint;
    (*this)(1,3) = (- ty_RR_hip_joint * cos_q_RR_hip_joint)- tz_fr_RR_thigh_COM- ty_RR_thigh_joint;
    (*this)(2,0) = sin_q_RR_thigh_joint;
    (*this)(2,1) = -sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(2,2) = cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(2,3) = (- tx_RR_hip_joint * sin_q_RR_thigh_joint)+( ty_RR_hip_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)+ ty_fr_RR_thigh_COM;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RR_thigh_shoulder::Type_fr_base_X_fr_RR_thigh_shoulder()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_RR_hip_joint;    // Maxima DSL: _k__tx_RR_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RR_thigh_shoulder& HomogeneousTransforms::Type_fr_base_X_fr_RR_thigh_shoulder::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = -sin_q_RR_hip_joint;
    (*this)(1,3) = ( ty_fr_RR_thigh_shoulder * cos_q_RR_hip_joint)+ ty_RR_hip_joint;
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,2) = cos_q_RR_hip_joint;
    (*this)(2,3) =  ty_fr_RR_thigh_shoulder * sin_q_RR_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_RR_thigh_shoulder_X_fr_base::Type_fr_RR_thigh_shoulder_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = - tx_RR_hip_joint;    // Maxima DSL: -_k__tx_RR_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RR_thigh_shoulder_X_fr_base& HomogeneousTransforms::Type_fr_RR_thigh_shoulder_X_fr_base::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = sin_q_RR_hip_joint;
    (*this)(1,3) = (- ty_RR_hip_joint * cos_q_RR_hip_joint)- ty_fr_RR_thigh_shoulder;
    (*this)(2,1) = -sin_q_RR_hip_joint;
    (*this)(2,2) = cos_q_RR_hip_joint;
    (*this)(2,3) =  ty_RR_hip_joint * sin_q_RR_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_base_COM::Type_fr_base_X_fr_base_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_fr_base_COM;    // Maxima DSL: _k__ty_fr_base_COM
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_fr_base_COM;    // Maxima DSL: _k__tz_fr_base_COM
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_base_COM& HomogeneousTransforms::Type_fr_base_X_fr_base_COM::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_COM_X_fr_base::Type_fr_base_COM_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = - ty_fr_base_COM;    // Maxima DSL: -_k__ty_fr_base_COM
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_fr_base_COM;    // Maxima DSL: -_k__tz_fr_base_COM
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_COM_X_fr_base& HomogeneousTransforms::Type_fr_base_COM_X_fr_base::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_imu_link::Type_fr_base_X_fr_imu_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_imu_link& HomogeneousTransforms::Type_fr_base_X_fr_imu_link::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_imu_link_X_fr_base::Type_fr_imu_link_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_imu_link_X_fr_base& HomogeneousTransforms::Type_fr_imu_link_X_fr_base::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_trunk::Type_fr_base_X_fr_trunk()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_trunk& HomogeneousTransforms::Type_fr_base_X_fr_trunk::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_trunk_X_fr_base::Type_fr_trunk_X_fr_base()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_trunk_X_fr_base& HomogeneousTransforms::Type_fr_trunk_X_fr_base::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FL_hip_joint::Type_fr_base_X_fr_FL_hip_joint()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_FL_hip_joint;    // Maxima DSL: _k__tx_FL_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_FL_hip_joint;    // Maxima DSL: _k__ty_FL_hip_joint
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FL_hip_joint& HomogeneousTransforms::Type_fr_base_X_fr_FL_hip_joint::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FL_thigh_joint::Type_fr_base_X_fr_FL_thigh_joint()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_FL_hip_joint;    // Maxima DSL: _k__tx_FL_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FL_thigh_joint& HomogeneousTransforms::Type_fr_base_X_fr_FL_thigh_joint::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(1,1) = sin_q_FL_hip_joint;
    (*this)(1,2) = cos_q_FL_hip_joint;
    (*this)(1,3) = ( ty_FL_thigh_joint * cos_q_FL_hip_joint)+ ty_FL_hip_joint;
    (*this)(2,1) = -cos_q_FL_hip_joint;
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(2,3) =  ty_FL_thigh_joint * sin_q_FL_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FL_calf_joint::Type_fr_base_X_fr_FL_calf_joint()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FL_calf_joint& HomogeneousTransforms::Type_fr_base_X_fr_FL_calf_joint::update(const state_t& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    (*this)(0,0) = cos_q_FL_thigh_joint;
    (*this)(0,1) = -sin_q_FL_thigh_joint;
    (*this)(0,3) =  tx_FL_hip_joint-( ty_FL_calf_joint * sin_q_FL_thigh_joint);
    (*this)(1,0) = sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(1,1) = sin_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(1,2) = cos_q_FL_hip_joint;
    (*this)(1,3) = ( ty_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)+( ty_FL_thigh_joint * cos_q_FL_hip_joint)+ ty_FL_hip_joint;
    (*this)(2,0) = -cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(2,1) = -cos_q_FL_hip_joint * cos_q_FL_thigh_joint;
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(2,3) = ( ty_FL_thigh_joint * sin_q_FL_hip_joint)-( ty_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FR_hip_joint::Type_fr_base_X_fr_FR_hip_joint()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_FR_hip_joint;    // Maxima DSL: _k__tx_FR_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_FR_hip_joint;    // Maxima DSL: _k__ty_FR_hip_joint
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FR_hip_joint& HomogeneousTransforms::Type_fr_base_X_fr_FR_hip_joint::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FR_thigh_joint::Type_fr_base_X_fr_FR_thigh_joint()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_FR_hip_joint;    // Maxima DSL: _k__tx_FR_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FR_thigh_joint& HomogeneousTransforms::Type_fr_base_X_fr_FR_thigh_joint::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(1,1) = sin_q_FR_hip_joint;
    (*this)(1,2) = cos_q_FR_hip_joint;
    (*this)(1,3) = ( ty_FR_thigh_joint * cos_q_FR_hip_joint)+ ty_FR_hip_joint;
    (*this)(2,1) = -cos_q_FR_hip_joint;
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(2,3) =  ty_FR_thigh_joint * sin_q_FR_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_FR_calf_joint::Type_fr_base_X_fr_FR_calf_joint()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_FR_calf_joint& HomogeneousTransforms::Type_fr_base_X_fr_FR_calf_joint::update(const state_t& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    (*this)(0,0) = cos_q_FR_thigh_joint;
    (*this)(0,1) = -sin_q_FR_thigh_joint;
    (*this)(0,3) =  tx_FR_hip_joint-( ty_FR_calf_joint * sin_q_FR_thigh_joint);
    (*this)(1,0) = sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(1,1) = sin_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(1,2) = cos_q_FR_hip_joint;
    (*this)(1,3) = ( ty_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)+( ty_FR_thigh_joint * cos_q_FR_hip_joint)+ ty_FR_hip_joint;
    (*this)(2,0) = -cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(2,1) = -cos_q_FR_hip_joint * cos_q_FR_thigh_joint;
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(2,3) = ( ty_FR_thigh_joint * sin_q_FR_hip_joint)-( ty_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RL_hip_joint::Type_fr_base_X_fr_RL_hip_joint()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_RL_hip_joint;    // Maxima DSL: _k__tx_RL_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_RL_hip_joint;    // Maxima DSL: _k__ty_RL_hip_joint
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RL_hip_joint& HomogeneousTransforms::Type_fr_base_X_fr_RL_hip_joint::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RL_thigh_joint::Type_fr_base_X_fr_RL_thigh_joint()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_RL_hip_joint;    // Maxima DSL: _k__tx_RL_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RL_thigh_joint& HomogeneousTransforms::Type_fr_base_X_fr_RL_thigh_joint::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(1,1) = sin_q_RL_hip_joint;
    (*this)(1,2) = cos_q_RL_hip_joint;
    (*this)(1,3) = ( ty_RL_thigh_joint * cos_q_RL_hip_joint)+ ty_RL_hip_joint;
    (*this)(2,1) = -cos_q_RL_hip_joint;
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(2,3) =  ty_RL_thigh_joint * sin_q_RL_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RL_calf_joint::Type_fr_base_X_fr_RL_calf_joint()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RL_calf_joint& HomogeneousTransforms::Type_fr_base_X_fr_RL_calf_joint::update(const state_t& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    (*this)(0,0) = cos_q_RL_thigh_joint;
    (*this)(0,1) = -sin_q_RL_thigh_joint;
    (*this)(0,3) =  tx_RL_hip_joint-( ty_RL_calf_joint * sin_q_RL_thigh_joint);
    (*this)(1,0) = sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(1,1) = sin_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(1,2) = cos_q_RL_hip_joint;
    (*this)(1,3) = ( ty_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)+( ty_RL_thigh_joint * cos_q_RL_hip_joint)+ ty_RL_hip_joint;
    (*this)(2,0) = -cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(2,1) = -cos_q_RL_hip_joint * cos_q_RL_thigh_joint;
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(2,3) = ( ty_RL_thigh_joint * sin_q_RL_hip_joint)-( ty_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RR_hip_joint::Type_fr_base_X_fr_RR_hip_joint()
{
    (*this)(0,0) = 0.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 1.0;
    (*this)(0,3) =  tx_RR_hip_joint;    // Maxima DSL: _k__tx_RR_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_RR_hip_joint;    // Maxima DSL: _k__ty_RR_hip_joint
    (*this)(2,0) = -1.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RR_hip_joint& HomogeneousTransforms::Type_fr_base_X_fr_RR_hip_joint::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RR_thigh_joint::Type_fr_base_X_fr_RR_thigh_joint()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_RR_hip_joint;    // Maxima DSL: _k__tx_RR_hip_joint
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RR_thigh_joint& HomogeneousTransforms::Type_fr_base_X_fr_RR_thigh_joint::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(1,1) = sin_q_RR_hip_joint;
    (*this)(1,2) = cos_q_RR_hip_joint;
    (*this)(1,3) = ( ty_RR_thigh_joint * cos_q_RR_hip_joint)+ ty_RR_hip_joint;
    (*this)(2,1) = -cos_q_RR_hip_joint;
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(2,3) =  ty_RR_thigh_joint * sin_q_RR_hip_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_X_fr_RR_calf_joint::Type_fr_base_X_fr_RR_calf_joint()
{
    (*this)(0,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_X_fr_RR_calf_joint& HomogeneousTransforms::Type_fr_base_X_fr_RR_calf_joint::update(const state_t& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    (*this)(0,0) = cos_q_RR_thigh_joint;
    (*this)(0,1) = -sin_q_RR_thigh_joint;
    (*this)(0,3) =  tx_RR_hip_joint-( ty_RR_calf_joint * sin_q_RR_thigh_joint);
    (*this)(1,0) = sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(1,1) = sin_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(1,2) = cos_q_RR_hip_joint;
    (*this)(1,3) = ( ty_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)+( ty_RR_thigh_joint * cos_q_RR_hip_joint)+ ty_RR_hip_joint;
    (*this)(2,0) = -cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(2,1) = -cos_q_RR_hip_joint * cos_q_RR_thigh_joint;
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(2,3) = ( ty_RR_thigh_joint * sin_q_RR_hip_joint)-( ty_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_FR_thigh_X_fr_FR_hip::Type_fr_FR_thigh_X_fr_FR_hip()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - ty_FR_thigh_joint;    // Maxima DSL: -_k__ty_FR_thigh_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FR_thigh_X_fr_FR_hip& HomogeneousTransforms::Type_fr_FR_thigh_X_fr_FR_hip::update(const state_t& q)
{
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    (*this)(0,0) = sin_q_FR_thigh_joint;
    (*this)(0,2) = cos_q_FR_thigh_joint;
    (*this)(1,0) = cos_q_FR_thigh_joint;
    (*this)(1,2) = -sin_q_FR_thigh_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_FR_hip_X_fr_FR_thigh::Type_fr_FR_hip_X_fr_FR_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) =  ty_FR_thigh_joint;    // Maxima DSL: _k__ty_FR_thigh_joint
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FR_hip_X_fr_FR_thigh& HomogeneousTransforms::Type_fr_FR_hip_X_fr_FR_thigh::update(const state_t& q)
{
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    (*this)(0,0) = sin_q_FR_thigh_joint;
    (*this)(0,1) = cos_q_FR_thigh_joint;
    (*this)(2,0) = cos_q_FR_thigh_joint;
    (*this)(2,1) = -sin_q_FR_thigh_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_FR_calf_X_fr_FR_thigh::Type_fr_FR_calf_X_fr_FR_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FR_calf_X_fr_FR_thigh& HomogeneousTransforms::Type_fr_FR_calf_X_fr_FR_thigh::update(const state_t& q)
{
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    (*this)(0,0) = cos_q_FR_calf_joint;
    (*this)(0,1) = sin_q_FR_calf_joint;
    (*this)(0,3) = - ty_FR_calf_joint * sin_q_FR_calf_joint;
    (*this)(1,0) = -sin_q_FR_calf_joint;
    (*this)(1,1) = cos_q_FR_calf_joint;
    (*this)(1,3) = - ty_FR_calf_joint * cos_q_FR_calf_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_FR_thigh_X_fr_FR_calf::Type_fr_FR_thigh_X_fr_FR_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_FR_calf_joint;    // Maxima DSL: _k__ty_FR_calf_joint
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FR_thigh_X_fr_FR_calf& HomogeneousTransforms::Type_fr_FR_thigh_X_fr_FR_calf::update(const state_t& q)
{
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    (*this)(0,0) = cos_q_FR_calf_joint;
    (*this)(0,1) = -sin_q_FR_calf_joint;
    (*this)(1,0) = sin_q_FR_calf_joint;
    (*this)(1,1) = cos_q_FR_calf_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_FL_thigh_X_fr_FL_hip::Type_fr_FL_thigh_X_fr_FL_hip()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - ty_FL_thigh_joint;    // Maxima DSL: -_k__ty_FL_thigh_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FL_thigh_X_fr_FL_hip& HomogeneousTransforms::Type_fr_FL_thigh_X_fr_FL_hip::update(const state_t& q)
{
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    (*this)(0,0) = sin_q_FL_thigh_joint;
    (*this)(0,2) = cos_q_FL_thigh_joint;
    (*this)(1,0) = cos_q_FL_thigh_joint;
    (*this)(1,2) = -sin_q_FL_thigh_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_FL_hip_X_fr_FL_thigh::Type_fr_FL_hip_X_fr_FL_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) =  ty_FL_thigh_joint;    // Maxima DSL: _k__ty_FL_thigh_joint
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FL_hip_X_fr_FL_thigh& HomogeneousTransforms::Type_fr_FL_hip_X_fr_FL_thigh::update(const state_t& q)
{
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    (*this)(0,0) = sin_q_FL_thigh_joint;
    (*this)(0,1) = cos_q_FL_thigh_joint;
    (*this)(2,0) = cos_q_FL_thigh_joint;
    (*this)(2,1) = -sin_q_FL_thigh_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_FL_calf_X_fr_FL_thigh::Type_fr_FL_calf_X_fr_FL_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FL_calf_X_fr_FL_thigh& HomogeneousTransforms::Type_fr_FL_calf_X_fr_FL_thigh::update(const state_t& q)
{
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    (*this)(0,0) = cos_q_FL_calf_joint;
    (*this)(0,1) = sin_q_FL_calf_joint;
    (*this)(0,3) = - ty_FL_calf_joint * sin_q_FL_calf_joint;
    (*this)(1,0) = -sin_q_FL_calf_joint;
    (*this)(1,1) = cos_q_FL_calf_joint;
    (*this)(1,3) = - ty_FL_calf_joint * cos_q_FL_calf_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_FL_thigh_X_fr_FL_calf::Type_fr_FL_thigh_X_fr_FL_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_FL_calf_joint;    // Maxima DSL: _k__ty_FL_calf_joint
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_FL_thigh_X_fr_FL_calf& HomogeneousTransforms::Type_fr_FL_thigh_X_fr_FL_calf::update(const state_t& q)
{
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    (*this)(0,0) = cos_q_FL_calf_joint;
    (*this)(0,1) = -sin_q_FL_calf_joint;
    (*this)(1,0) = sin_q_FL_calf_joint;
    (*this)(1,1) = cos_q_FL_calf_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_RR_thigh_X_fr_RR_hip::Type_fr_RR_thigh_X_fr_RR_hip()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - ty_RR_thigh_joint;    // Maxima DSL: -_k__ty_RR_thigh_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RR_thigh_X_fr_RR_hip& HomogeneousTransforms::Type_fr_RR_thigh_X_fr_RR_hip::update(const state_t& q)
{
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    (*this)(0,0) = sin_q_RR_thigh_joint;
    (*this)(0,2) = cos_q_RR_thigh_joint;
    (*this)(1,0) = cos_q_RR_thigh_joint;
    (*this)(1,2) = -sin_q_RR_thigh_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_RR_hip_X_fr_RR_thigh::Type_fr_RR_hip_X_fr_RR_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) =  ty_RR_thigh_joint;    // Maxima DSL: _k__ty_RR_thigh_joint
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RR_hip_X_fr_RR_thigh& HomogeneousTransforms::Type_fr_RR_hip_X_fr_RR_thigh::update(const state_t& q)
{
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    (*this)(0,0) = sin_q_RR_thigh_joint;
    (*this)(0,1) = cos_q_RR_thigh_joint;
    (*this)(2,0) = cos_q_RR_thigh_joint;
    (*this)(2,1) = -sin_q_RR_thigh_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_RR_calf_X_fr_RR_thigh::Type_fr_RR_calf_X_fr_RR_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RR_calf_X_fr_RR_thigh& HomogeneousTransforms::Type_fr_RR_calf_X_fr_RR_thigh::update(const state_t& q)
{
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    (*this)(0,0) = cos_q_RR_calf_joint;
    (*this)(0,1) = sin_q_RR_calf_joint;
    (*this)(0,3) = - ty_RR_calf_joint * sin_q_RR_calf_joint;
    (*this)(1,0) = -sin_q_RR_calf_joint;
    (*this)(1,1) = cos_q_RR_calf_joint;
    (*this)(1,3) = - ty_RR_calf_joint * cos_q_RR_calf_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_RR_thigh_X_fr_RR_calf::Type_fr_RR_thigh_X_fr_RR_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_RR_calf_joint;    // Maxima DSL: _k__ty_RR_calf_joint
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RR_thigh_X_fr_RR_calf& HomogeneousTransforms::Type_fr_RR_thigh_X_fr_RR_calf::update(const state_t& q)
{
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    (*this)(0,0) = cos_q_RR_calf_joint;
    (*this)(0,1) = -sin_q_RR_calf_joint;
    (*this)(1,0) = sin_q_RR_calf_joint;
    (*this)(1,1) = cos_q_RR_calf_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_RL_thigh_X_fr_RL_hip::Type_fr_RL_thigh_X_fr_RL_hip()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - ty_RL_thigh_joint;    // Maxima DSL: -_k__ty_RL_thigh_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RL_thigh_X_fr_RL_hip& HomogeneousTransforms::Type_fr_RL_thigh_X_fr_RL_hip::update(const state_t& q)
{
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    (*this)(0,0) = sin_q_RL_thigh_joint;
    (*this)(0,2) = cos_q_RL_thigh_joint;
    (*this)(1,0) = cos_q_RL_thigh_joint;
    (*this)(1,2) = -sin_q_RL_thigh_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_RL_hip_X_fr_RL_thigh::Type_fr_RL_hip_X_fr_RL_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) =  ty_RL_thigh_joint;    // Maxima DSL: _k__ty_RL_thigh_joint
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RL_hip_X_fr_RL_thigh& HomogeneousTransforms::Type_fr_RL_hip_X_fr_RL_thigh::update(const state_t& q)
{
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    (*this)(0,0) = sin_q_RL_thigh_joint;
    (*this)(0,1) = cos_q_RL_thigh_joint;
    (*this)(2,0) = cos_q_RL_thigh_joint;
    (*this)(2,1) = -sin_q_RL_thigh_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_RL_calf_X_fr_RL_thigh::Type_fr_RL_calf_X_fr_RL_thigh()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RL_calf_X_fr_RL_thigh& HomogeneousTransforms::Type_fr_RL_calf_X_fr_RL_thigh::update(const state_t& q)
{
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    (*this)(0,0) = cos_q_RL_calf_joint;
    (*this)(0,1) = sin_q_RL_calf_joint;
    (*this)(0,3) = - ty_RL_calf_joint * sin_q_RL_calf_joint;
    (*this)(1,0) = -sin_q_RL_calf_joint;
    (*this)(1,1) = cos_q_RL_calf_joint;
    (*this)(1,3) = - ty_RL_calf_joint * cos_q_RL_calf_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_RL_thigh_X_fr_RL_calf::Type_fr_RL_thigh_X_fr_RL_calf()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_RL_calf_joint;    // Maxima DSL: _k__ty_RL_calf_joint
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_RL_thigh_X_fr_RL_calf& HomogeneousTransforms::Type_fr_RL_thigh_X_fr_RL_calf::update(const state_t& q)
{
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    (*this)(0,0) = cos_q_RL_calf_joint;
    (*this)(0,1) = -sin_q_RL_calf_joint;
    (*this)(1,0) = sin_q_RL_calf_joint;
    (*this)(1,1) = cos_q_RL_calf_joint;
    return *this;
}

