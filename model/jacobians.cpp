#include "jacobians.h"

UnitreeA1::rcg::Jacobians::Jacobians()
:    fr_base_J_fr_FL_calf(), 
    fr_base_J_fr_FL_hip(), 
    fr_base_J_fr_FL_thigh(), 
    fr_base_J_fr_FR_calf(), 
    fr_base_J_fr_FR_hip(), 
    fr_base_J_fr_FR_thigh(), 
    fr_base_J_fr_RL_calf(), 
    fr_base_J_fr_RL_hip(), 
    fr_base_J_fr_RL_thigh(), 
    fr_base_J_fr_RR_calf(), 
    fr_base_J_fr_RR_hip(), 
    fr_base_J_fr_RR_thigh(), 
    fr_base_J_fr_FL_calf_COM(), 
    fr_base_J_fr_FL_foot(), 
    fr_base_J_fr_FL_hip_COM(), 
    fr_base_J_fr_FL_thigh_COM(), 
    fr_base_J_fr_FL_thigh_shoulder(), 
    fr_base_J_fr_FR_calf_COM(), 
    fr_base_J_fr_FR_foot(), 
    fr_base_J_fr_FR_hip_COM(), 
    fr_base_J_fr_FR_thigh_COM(), 
    fr_base_J_fr_FR_thigh_shoulder(), 
    fr_base_J_fr_RL_calf_COM(), 
    fr_base_J_fr_RL_foot(), 
    fr_base_J_fr_RL_hip_COM(), 
    fr_base_J_fr_RL_thigh_COM(), 
    fr_base_J_fr_RL_thigh_shoulder(), 
    fr_base_J_fr_RR_calf_COM(), 
    fr_base_J_fr_RR_foot(), 
    fr_base_J_fr_RR_hip_COM(), 
    fr_base_J_fr_RR_thigh_COM(), 
    fr_base_J_fr_RR_thigh_shoulder()
{}

void UnitreeA1::rcg::Jacobians::updateParameters(const Params_lengths& _lengths, const Params_angles& _angles)
{
    params.lengths = _lengths;
    params.angles = _angles;
    params.trig.update();
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_calf::Type_fr_base_J_fr_FL_calf()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,2) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_calf& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_calf::update(const JointState& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = cos_q_FL_hip_joint;
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(3,1) = - ty_FL_calf_joint * cos_q_FL_thigh_joint;
    (*this)(4,0) = ( ty_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-( ty_FL_thigh_joint * sin_q_FL_hip_joint);
    (*this)(4,1) = - ty_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint;
    (*this)(5,0) = ( ty_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)+( ty_FL_thigh_joint * cos_q_FL_hip_joint);
    (*this)(5,1) =  ty_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint;
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_hip::Type_fr_base_J_fr_FL_hip()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(5,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_hip& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_hip::update(const JointState& q)
{
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_thigh::Type_fr_base_J_fr_FL_thigh()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(5,1) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_thigh& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_thigh::update(const JointState& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(4,0) = - ty_FL_thigh_joint * sin_q_FL_hip_joint;
    (*this)(5,0) =  ty_FL_thigh_joint * cos_q_FL_hip_joint;
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_calf::Type_fr_base_J_fr_FR_calf()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,2) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_calf& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_calf::update(const JointState& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = cos_q_FR_hip_joint;
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(3,1) = - ty_FR_calf_joint * cos_q_FR_thigh_joint;
    (*this)(4,0) = ( ty_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-( ty_FR_thigh_joint * sin_q_FR_hip_joint);
    (*this)(4,1) = - ty_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint;
    (*this)(5,0) = ( ty_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)+( ty_FR_thigh_joint * cos_q_FR_hip_joint);
    (*this)(5,1) =  ty_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint;
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_hip::Type_fr_base_J_fr_FR_hip()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(5,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_hip& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_hip::update(const JointState& q)
{
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_thigh::Type_fr_base_J_fr_FR_thigh()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(5,1) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_thigh& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_thigh::update(const JointState& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(4,0) = - ty_FR_thigh_joint * sin_q_FR_hip_joint;
    (*this)(5,0) =  ty_FR_thigh_joint * cos_q_FR_hip_joint;
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_calf::Type_fr_base_J_fr_RL_calf()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,2) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_calf& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_calf::update(const JointState& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = cos_q_RL_hip_joint;
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(3,1) = - ty_RL_calf_joint * cos_q_RL_thigh_joint;
    (*this)(4,0) = ( ty_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-( ty_RL_thigh_joint * sin_q_RL_hip_joint);
    (*this)(4,1) = - ty_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint;
    (*this)(5,0) = ( ty_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)+( ty_RL_thigh_joint * cos_q_RL_hip_joint);
    (*this)(5,1) =  ty_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint;
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_hip::Type_fr_base_J_fr_RL_hip()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(5,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_hip& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_hip::update(const JointState& q)
{
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_thigh::Type_fr_base_J_fr_RL_thigh()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(5,1) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_thigh& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_thigh::update(const JointState& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(4,0) = - ty_RL_thigh_joint * sin_q_RL_hip_joint;
    (*this)(5,0) =  ty_RL_thigh_joint * cos_q_RL_hip_joint;
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_calf::Type_fr_base_J_fr_RR_calf()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,2) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_calf& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_calf::update(const JointState& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = cos_q_RR_hip_joint;
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(3,1) = - ty_RR_calf_joint * cos_q_RR_thigh_joint;
    (*this)(4,0) = ( ty_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-( ty_RR_thigh_joint * sin_q_RR_hip_joint);
    (*this)(4,1) = - ty_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint;
    (*this)(5,0) = ( ty_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)+( ty_RR_thigh_joint * cos_q_RR_hip_joint);
    (*this)(5,1) =  ty_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint;
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_hip::Type_fr_base_J_fr_RR_hip()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(5,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_hip& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_hip::update(const JointState& q)
{
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_thigh::Type_fr_base_J_fr_RR_thigh()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(5,1) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_thigh& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_thigh::update(const JointState& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(4,0) = - ty_RR_thigh_joint * sin_q_RR_hip_joint;
    (*this)(5,0) =  ty_RR_thigh_joint * cos_q_RR_hip_joint;
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_calf_COM::Type_fr_base_J_fr_FL_calf_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_calf_COM& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_calf_COM::update(const JointState& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = cos_q_FL_hip_joint;
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(3,1) = ((( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)-( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- tx_fr_FL_calf_COM * sin_q_FL_calf_joint)-( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)- ty_FL_calf_joint) * cos_q_FL_thigh_joint);
    (*this)(3,2) = ((( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)-( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)) * sin_q_FL_thigh_joint)+(((- tx_fr_FL_calf_COM * sin_q_FL_calf_joint)-( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)) * cos_q_FL_thigh_joint);
    (*this)(4,0) = ((( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)-( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)) * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+((( tx_fr_FL_calf_COM * sin_q_FL_calf_joint)+( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-( ty_FL_thigh_joint * sin_q_FL_hip_joint);
    (*this)(4,1) = (((- tx_fr_FL_calf_COM * sin_q_FL_calf_joint)-( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)- ty_FL_calf_joint) * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+((( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)-( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)) * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(4,2) = (((- tx_fr_FL_calf_COM * sin_q_FL_calf_joint)-( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)) * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+((( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)-( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)) * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,0) = ((( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)-( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)) * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+((( tx_fr_FL_calf_COM * sin_q_FL_calf_joint)+( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)+( ty_FL_thigh_joint * cos_q_FL_hip_joint);
    (*this)(5,1) = ((( tx_fr_FL_calf_COM * sin_q_FL_calf_joint)+( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+((( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)-( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)) * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,2) = ((( tx_fr_FL_calf_COM * sin_q_FL_calf_joint)+( ty_fr_FL_calf_COM * cos_q_FL_calf_joint)) * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+((( ty_fr_FL_calf_COM * sin_q_FL_calf_joint)-( tx_fr_FL_calf_COM * cos_q_FL_calf_joint)) * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_foot::Type_fr_base_J_fr_FL_foot()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_foot& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_foot::update(const JointState& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    Scalar sin_q_FL_calf_joint  = ScalarTraits::sin( q(FL_CALF_JOINT) );
    Scalar cos_q_FL_calf_joint  = ScalarTraits::cos( q(FL_CALF_JOINT) );
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(1,2) = cos_q_FL_hip_joint;
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(2,2) = sin_q_FL_hip_joint;
    (*this)(3,1) = ( ty_fr_FL_foot * sin_q_FL_calf_joint * sin_q_FL_thigh_joint)+(((- ty_fr_FL_foot * cos_q_FL_calf_joint)- ty_FL_calf_joint) * cos_q_FL_thigh_joint);
    (*this)(3,2) = ( ty_fr_FL_foot * sin_q_FL_calf_joint * sin_q_FL_thigh_joint)-( ty_fr_FL_foot * cos_q_FL_calf_joint * cos_q_FL_thigh_joint);
    (*this)(4,0) = (- ty_fr_FL_foot * sin_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+((( ty_fr_FL_foot * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)-( ty_FL_thigh_joint * sin_q_FL_hip_joint);
    (*this)(4,1) = (((- ty_fr_FL_foot * cos_q_FL_calf_joint)- ty_FL_calf_joint) * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-( ty_fr_FL_foot * sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(4,2) = (- ty_fr_FL_foot * cos_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)-( ty_fr_FL_foot * sin_q_FL_calf_joint * sin_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,0) = (- ty_fr_FL_foot * sin_q_FL_calf_joint * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+((( ty_fr_FL_foot * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)+( ty_FL_thigh_joint * cos_q_FL_hip_joint);
    (*this)(5,1) = ((( ty_fr_FL_foot * cos_q_FL_calf_joint)+ ty_FL_calf_joint) * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+( ty_fr_FL_foot * sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    (*this)(5,2) = ( ty_fr_FL_foot * cos_q_FL_calf_joint * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+( ty_fr_FL_foot * sin_q_FL_calf_joint * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_hip_COM::Type_fr_base_J_fr_FL_hip_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_hip_COM& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_hip_COM::update(const JointState& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(4,0) = ( tx_fr_FL_hip_COM * cos_q_FL_hip_joint)-( ty_fr_FL_hip_COM * sin_q_FL_hip_joint);
    (*this)(5,0) = ( tx_fr_FL_hip_COM * sin_q_FL_hip_joint)+( ty_fr_FL_hip_COM * cos_q_FL_hip_joint);
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_thigh_COM::Type_fr_base_J_fr_FL_thigh_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_thigh_COM& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_thigh_COM::update(const JointState& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    Scalar sin_q_FL_thigh_joint  = ScalarTraits::sin( q(FL_THIGH_JOINT) );
    Scalar cos_q_FL_thigh_joint  = ScalarTraits::cos( q(FL_THIGH_JOINT) );
    (*this)(1,1) = cos_q_FL_hip_joint;
    (*this)(2,1) = sin_q_FL_hip_joint;
    (*this)(3,1) = (- tx_fr_FL_thigh_COM * sin_q_FL_thigh_joint)-( ty_fr_FL_thigh_COM * cos_q_FL_thigh_joint);
    (*this)(4,0) = ( tx_fr_FL_thigh_COM * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)+( ty_fr_FL_thigh_COM * cos_q_FL_hip_joint * cos_q_FL_thigh_joint)+((- tz_fr_FL_thigh_COM- ty_FL_thigh_joint) * sin_q_FL_hip_joint);
    (*this)(4,1) = ( tx_fr_FL_thigh_COM * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)-( ty_fr_FL_thigh_COM * sin_q_FL_hip_joint * sin_q_FL_thigh_joint);
    (*this)(5,0) = ( tx_fr_FL_thigh_COM * sin_q_FL_hip_joint * sin_q_FL_thigh_joint)+( ty_fr_FL_thigh_COM * sin_q_FL_hip_joint * cos_q_FL_thigh_joint)+(( tz_fr_FL_thigh_COM+ ty_FL_thigh_joint) * cos_q_FL_hip_joint);
    (*this)(5,1) = ( ty_fr_FL_thigh_COM * cos_q_FL_hip_joint * sin_q_FL_thigh_joint)-( tx_fr_FL_thigh_COM * cos_q_FL_hip_joint * cos_q_FL_thigh_joint);
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_thigh_shoulder::Type_fr_base_J_fr_FL_thigh_shoulder()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_thigh_shoulder& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FL_thigh_shoulder::update(const JointState& q)
{
    Scalar sin_q_FL_hip_joint  = ScalarTraits::sin( q(FL_HIP_JOINT) );
    Scalar cos_q_FL_hip_joint  = ScalarTraits::cos( q(FL_HIP_JOINT) );
    (*this)(4,0) = - ty_fr_FL_thigh_shoulder * sin_q_FL_hip_joint;
    (*this)(5,0) =  ty_fr_FL_thigh_shoulder * cos_q_FL_hip_joint;
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_calf_COM::Type_fr_base_J_fr_FR_calf_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_calf_COM& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_calf_COM::update(const JointState& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = cos_q_FR_hip_joint;
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(3,1) = ((( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)-( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- tx_fr_FR_calf_COM * sin_q_FR_calf_joint)-( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)- ty_FR_calf_joint) * cos_q_FR_thigh_joint);
    (*this)(3,2) = ((( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)-( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)) * sin_q_FR_thigh_joint)+(((- tx_fr_FR_calf_COM * sin_q_FR_calf_joint)-( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)) * cos_q_FR_thigh_joint);
    (*this)(4,0) = ((( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)-( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)) * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+((( tx_fr_FR_calf_COM * sin_q_FR_calf_joint)+( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-( ty_FR_thigh_joint * sin_q_FR_hip_joint);
    (*this)(4,1) = (((- tx_fr_FR_calf_COM * sin_q_FR_calf_joint)-( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)- ty_FR_calf_joint) * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+((( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)-( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)) * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(4,2) = (((- tx_fr_FR_calf_COM * sin_q_FR_calf_joint)-( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)) * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+((( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)-( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)) * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,0) = ((( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)-( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)) * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+((( tx_fr_FR_calf_COM * sin_q_FR_calf_joint)+( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)+( ty_FR_thigh_joint * cos_q_FR_hip_joint);
    (*this)(5,1) = ((( tx_fr_FR_calf_COM * sin_q_FR_calf_joint)+( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+((( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)-( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)) * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,2) = ((( tx_fr_FR_calf_COM * sin_q_FR_calf_joint)+( ty_fr_FR_calf_COM * cos_q_FR_calf_joint)) * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+((( ty_fr_FR_calf_COM * sin_q_FR_calf_joint)-( tx_fr_FR_calf_COM * cos_q_FR_calf_joint)) * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_foot::Type_fr_base_J_fr_FR_foot()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_foot& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_foot::update(const JointState& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    Scalar sin_q_FR_calf_joint  = ScalarTraits::sin( q(FR_CALF_JOINT) );
    Scalar cos_q_FR_calf_joint  = ScalarTraits::cos( q(FR_CALF_JOINT) );
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(1,2) = cos_q_FR_hip_joint;
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(2,2) = sin_q_FR_hip_joint;
    (*this)(3,1) = ( ty_fr_FR_foot * sin_q_FR_calf_joint * sin_q_FR_thigh_joint)+(((- ty_fr_FR_foot * cos_q_FR_calf_joint)- ty_FR_calf_joint) * cos_q_FR_thigh_joint);
    (*this)(3,2) = ( ty_fr_FR_foot * sin_q_FR_calf_joint * sin_q_FR_thigh_joint)-( ty_fr_FR_foot * cos_q_FR_calf_joint * cos_q_FR_thigh_joint);
    (*this)(4,0) = (- ty_fr_FR_foot * sin_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+((( ty_fr_FR_foot * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)-( ty_FR_thigh_joint * sin_q_FR_hip_joint);
    (*this)(4,1) = (((- ty_fr_FR_foot * cos_q_FR_calf_joint)- ty_FR_calf_joint) * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-( ty_fr_FR_foot * sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(4,2) = (- ty_fr_FR_foot * cos_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)-( ty_fr_FR_foot * sin_q_FR_calf_joint * sin_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,0) = (- ty_fr_FR_foot * sin_q_FR_calf_joint * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+((( ty_fr_FR_foot * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)+( ty_FR_thigh_joint * cos_q_FR_hip_joint);
    (*this)(5,1) = ((( ty_fr_FR_foot * cos_q_FR_calf_joint)+ ty_FR_calf_joint) * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+( ty_fr_FR_foot * sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    (*this)(5,2) = ( ty_fr_FR_foot * cos_q_FR_calf_joint * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+( ty_fr_FR_foot * sin_q_FR_calf_joint * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_hip_COM::Type_fr_base_J_fr_FR_hip_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_hip_COM& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_hip_COM::update(const JointState& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(4,0) = ( tx_fr_FR_hip_COM * cos_q_FR_hip_joint)-( ty_fr_FR_hip_COM * sin_q_FR_hip_joint);
    (*this)(5,0) = ( tx_fr_FR_hip_COM * sin_q_FR_hip_joint)+( ty_fr_FR_hip_COM * cos_q_FR_hip_joint);
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_thigh_COM::Type_fr_base_J_fr_FR_thigh_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_thigh_COM& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_thigh_COM::update(const JointState& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    Scalar sin_q_FR_thigh_joint  = ScalarTraits::sin( q(FR_THIGH_JOINT) );
    Scalar cos_q_FR_thigh_joint  = ScalarTraits::cos( q(FR_THIGH_JOINT) );
    (*this)(1,1) = cos_q_FR_hip_joint;
    (*this)(2,1) = sin_q_FR_hip_joint;
    (*this)(3,1) = (- tx_fr_FR_thigh_COM * sin_q_FR_thigh_joint)-( ty_fr_FR_thigh_COM * cos_q_FR_thigh_joint);
    (*this)(4,0) = ( tx_fr_FR_thigh_COM * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)+( ty_fr_FR_thigh_COM * cos_q_FR_hip_joint * cos_q_FR_thigh_joint)+((- tz_fr_FR_thigh_COM- ty_FR_thigh_joint) * sin_q_FR_hip_joint);
    (*this)(4,1) = ( tx_fr_FR_thigh_COM * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)-( ty_fr_FR_thigh_COM * sin_q_FR_hip_joint * sin_q_FR_thigh_joint);
    (*this)(5,0) = ( tx_fr_FR_thigh_COM * sin_q_FR_hip_joint * sin_q_FR_thigh_joint)+( ty_fr_FR_thigh_COM * sin_q_FR_hip_joint * cos_q_FR_thigh_joint)+(( tz_fr_FR_thigh_COM+ ty_FR_thigh_joint) * cos_q_FR_hip_joint);
    (*this)(5,1) = ( ty_fr_FR_thigh_COM * cos_q_FR_hip_joint * sin_q_FR_thigh_joint)-( tx_fr_FR_thigh_COM * cos_q_FR_hip_joint * cos_q_FR_thigh_joint);
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_thigh_shoulder::Type_fr_base_J_fr_FR_thigh_shoulder()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_thigh_shoulder& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_FR_thigh_shoulder::update(const JointState& q)
{
    Scalar sin_q_FR_hip_joint  = ScalarTraits::sin( q(FR_HIP_JOINT) );
    Scalar cos_q_FR_hip_joint  = ScalarTraits::cos( q(FR_HIP_JOINT) );
    (*this)(4,0) = - ty_fr_FR_thigh_shoulder * sin_q_FR_hip_joint;
    (*this)(5,0) =  ty_fr_FR_thigh_shoulder * cos_q_FR_hip_joint;
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_calf_COM::Type_fr_base_J_fr_RL_calf_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_calf_COM& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_calf_COM::update(const JointState& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = cos_q_RL_hip_joint;
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(3,1) = ((( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)-( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- tx_fr_RL_calf_COM * sin_q_RL_calf_joint)-( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)- ty_RL_calf_joint) * cos_q_RL_thigh_joint);
    (*this)(3,2) = ((( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)-( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)) * sin_q_RL_thigh_joint)+(((- tx_fr_RL_calf_COM * sin_q_RL_calf_joint)-( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)) * cos_q_RL_thigh_joint);
    (*this)(4,0) = ((( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)-( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)) * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+((( tx_fr_RL_calf_COM * sin_q_RL_calf_joint)+( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-( ty_RL_thigh_joint * sin_q_RL_hip_joint);
    (*this)(4,1) = (((- tx_fr_RL_calf_COM * sin_q_RL_calf_joint)-( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)- ty_RL_calf_joint) * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+((( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)-( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)) * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(4,2) = (((- tx_fr_RL_calf_COM * sin_q_RL_calf_joint)-( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)) * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+((( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)-( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)) * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,0) = ((( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)-( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)) * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+((( tx_fr_RL_calf_COM * sin_q_RL_calf_joint)+( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)+( ty_RL_thigh_joint * cos_q_RL_hip_joint);
    (*this)(5,1) = ((( tx_fr_RL_calf_COM * sin_q_RL_calf_joint)+( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+((( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)-( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)) * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,2) = ((( tx_fr_RL_calf_COM * sin_q_RL_calf_joint)+( ty_fr_RL_calf_COM * cos_q_RL_calf_joint)) * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+((( ty_fr_RL_calf_COM * sin_q_RL_calf_joint)-( tx_fr_RL_calf_COM * cos_q_RL_calf_joint)) * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_foot::Type_fr_base_J_fr_RL_foot()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_foot& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_foot::update(const JointState& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    Scalar sin_q_RL_calf_joint  = ScalarTraits::sin( q(RL_CALF_JOINT) );
    Scalar cos_q_RL_calf_joint  = ScalarTraits::cos( q(RL_CALF_JOINT) );
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(1,2) = cos_q_RL_hip_joint;
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(2,2) = sin_q_RL_hip_joint;
    (*this)(3,1) = ( ty_fr_RL_foot * sin_q_RL_calf_joint * sin_q_RL_thigh_joint)+(((- ty_fr_RL_foot * cos_q_RL_calf_joint)- ty_RL_calf_joint) * cos_q_RL_thigh_joint);
    (*this)(3,2) = ( ty_fr_RL_foot * sin_q_RL_calf_joint * sin_q_RL_thigh_joint)-( ty_fr_RL_foot * cos_q_RL_calf_joint * cos_q_RL_thigh_joint);
    (*this)(4,0) = (- ty_fr_RL_foot * sin_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+((( ty_fr_RL_foot * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)-( ty_RL_thigh_joint * sin_q_RL_hip_joint);
    (*this)(4,1) = (((- ty_fr_RL_foot * cos_q_RL_calf_joint)- ty_RL_calf_joint) * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-( ty_fr_RL_foot * sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(4,2) = (- ty_fr_RL_foot * cos_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)-( ty_fr_RL_foot * sin_q_RL_calf_joint * sin_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,0) = (- ty_fr_RL_foot * sin_q_RL_calf_joint * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+((( ty_fr_RL_foot * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)+( ty_RL_thigh_joint * cos_q_RL_hip_joint);
    (*this)(5,1) = ((( ty_fr_RL_foot * cos_q_RL_calf_joint)+ ty_RL_calf_joint) * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+( ty_fr_RL_foot * sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    (*this)(5,2) = ( ty_fr_RL_foot * cos_q_RL_calf_joint * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+( ty_fr_RL_foot * sin_q_RL_calf_joint * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_hip_COM::Type_fr_base_J_fr_RL_hip_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_hip_COM& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_hip_COM::update(const JointState& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(4,0) = ( tx_fr_RL_hip_COM * cos_q_RL_hip_joint)-( ty_fr_RL_hip_COM * sin_q_RL_hip_joint);
    (*this)(5,0) = ( tx_fr_RL_hip_COM * sin_q_RL_hip_joint)+( ty_fr_RL_hip_COM * cos_q_RL_hip_joint);
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_thigh_COM::Type_fr_base_J_fr_RL_thigh_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_thigh_COM& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_thigh_COM::update(const JointState& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    Scalar sin_q_RL_thigh_joint  = ScalarTraits::sin( q(RL_THIGH_JOINT) );
    Scalar cos_q_RL_thigh_joint  = ScalarTraits::cos( q(RL_THIGH_JOINT) );
    (*this)(1,1) = cos_q_RL_hip_joint;
    (*this)(2,1) = sin_q_RL_hip_joint;
    (*this)(3,1) = (- tx_fr_RL_thigh_COM * sin_q_RL_thigh_joint)-( ty_fr_RL_thigh_COM * cos_q_RL_thigh_joint);
    (*this)(4,0) = ( tx_fr_RL_thigh_COM * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)+( ty_fr_RL_thigh_COM * cos_q_RL_hip_joint * cos_q_RL_thigh_joint)+((- tz_fr_RL_thigh_COM- ty_RL_thigh_joint) * sin_q_RL_hip_joint);
    (*this)(4,1) = ( tx_fr_RL_thigh_COM * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)-( ty_fr_RL_thigh_COM * sin_q_RL_hip_joint * sin_q_RL_thigh_joint);
    (*this)(5,0) = ( tx_fr_RL_thigh_COM * sin_q_RL_hip_joint * sin_q_RL_thigh_joint)+( ty_fr_RL_thigh_COM * sin_q_RL_hip_joint * cos_q_RL_thigh_joint)+(( tz_fr_RL_thigh_COM+ ty_RL_thigh_joint) * cos_q_RL_hip_joint);
    (*this)(5,1) = ( ty_fr_RL_thigh_COM * cos_q_RL_hip_joint * sin_q_RL_thigh_joint)-( tx_fr_RL_thigh_COM * cos_q_RL_hip_joint * cos_q_RL_thigh_joint);
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_thigh_shoulder::Type_fr_base_J_fr_RL_thigh_shoulder()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_thigh_shoulder& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RL_thigh_shoulder::update(const JointState& q)
{
    Scalar sin_q_RL_hip_joint  = ScalarTraits::sin( q(RL_HIP_JOINT) );
    Scalar cos_q_RL_hip_joint  = ScalarTraits::cos( q(RL_HIP_JOINT) );
    (*this)(4,0) = - ty_fr_RL_thigh_shoulder * sin_q_RL_hip_joint;
    (*this)(5,0) =  ty_fr_RL_thigh_shoulder * cos_q_RL_hip_joint;
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_calf_COM::Type_fr_base_J_fr_RR_calf_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_calf_COM& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_calf_COM::update(const JointState& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = cos_q_RR_hip_joint;
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(3,1) = ((( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)-( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- tx_fr_RR_calf_COM * sin_q_RR_calf_joint)-( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)- ty_RR_calf_joint) * cos_q_RR_thigh_joint);
    (*this)(3,2) = ((( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)-( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)) * sin_q_RR_thigh_joint)+(((- tx_fr_RR_calf_COM * sin_q_RR_calf_joint)-( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)) * cos_q_RR_thigh_joint);
    (*this)(4,0) = ((( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)-( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)) * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+((( tx_fr_RR_calf_COM * sin_q_RR_calf_joint)+( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-( ty_RR_thigh_joint * sin_q_RR_hip_joint);
    (*this)(4,1) = (((- tx_fr_RR_calf_COM * sin_q_RR_calf_joint)-( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)- ty_RR_calf_joint) * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+((( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)-( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)) * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(4,2) = (((- tx_fr_RR_calf_COM * sin_q_RR_calf_joint)-( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)) * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+((( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)-( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)) * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,0) = ((( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)-( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)) * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+((( tx_fr_RR_calf_COM * sin_q_RR_calf_joint)+( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)+( ty_RR_thigh_joint * cos_q_RR_hip_joint);
    (*this)(5,1) = ((( tx_fr_RR_calf_COM * sin_q_RR_calf_joint)+( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+((( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)-( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)) * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,2) = ((( tx_fr_RR_calf_COM * sin_q_RR_calf_joint)+( ty_fr_RR_calf_COM * cos_q_RR_calf_joint)) * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+((( ty_fr_RR_calf_COM * sin_q_RR_calf_joint)-( tx_fr_RR_calf_COM * cos_q_RR_calf_joint)) * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_foot::Type_fr_base_J_fr_RR_foot()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_foot& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_foot::update(const JointState& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    Scalar sin_q_RR_calf_joint  = ScalarTraits::sin( q(RR_CALF_JOINT) );
    Scalar cos_q_RR_calf_joint  = ScalarTraits::cos( q(RR_CALF_JOINT) );
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(1,2) = cos_q_RR_hip_joint;
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(2,2) = sin_q_RR_hip_joint;
    (*this)(3,1) = ( ty_fr_RR_foot * sin_q_RR_calf_joint * sin_q_RR_thigh_joint)+(((- ty_fr_RR_foot * cos_q_RR_calf_joint)- ty_RR_calf_joint) * cos_q_RR_thigh_joint);
    (*this)(3,2) = ( ty_fr_RR_foot * sin_q_RR_calf_joint * sin_q_RR_thigh_joint)-( ty_fr_RR_foot * cos_q_RR_calf_joint * cos_q_RR_thigh_joint);
    (*this)(4,0) = (- ty_fr_RR_foot * sin_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+((( ty_fr_RR_foot * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)-( ty_RR_thigh_joint * sin_q_RR_hip_joint);
    (*this)(4,1) = (((- ty_fr_RR_foot * cos_q_RR_calf_joint)- ty_RR_calf_joint) * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-( ty_fr_RR_foot * sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(4,2) = (- ty_fr_RR_foot * cos_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)-( ty_fr_RR_foot * sin_q_RR_calf_joint * sin_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,0) = (- ty_fr_RR_foot * sin_q_RR_calf_joint * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+((( ty_fr_RR_foot * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)+( ty_RR_thigh_joint * cos_q_RR_hip_joint);
    (*this)(5,1) = ((( ty_fr_RR_foot * cos_q_RR_calf_joint)+ ty_RR_calf_joint) * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+( ty_fr_RR_foot * sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    (*this)(5,2) = ( ty_fr_RR_foot * cos_q_RR_calf_joint * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+( ty_fr_RR_foot * sin_q_RR_calf_joint * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_hip_COM::Type_fr_base_J_fr_RR_hip_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_hip_COM& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_hip_COM::update(const JointState& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(4,0) = ( tx_fr_RR_hip_COM * cos_q_RR_hip_joint)-( ty_fr_RR_hip_COM * sin_q_RR_hip_joint);
    (*this)(5,0) = ( tx_fr_RR_hip_COM * sin_q_RR_hip_joint)+( ty_fr_RR_hip_COM * cos_q_RR_hip_joint);
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_thigh_COM::Type_fr_base_J_fr_RR_thigh_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_thigh_COM& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_thigh_COM::update(const JointState& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    Scalar sin_q_RR_thigh_joint  = ScalarTraits::sin( q(RR_THIGH_JOINT) );
    Scalar cos_q_RR_thigh_joint  = ScalarTraits::cos( q(RR_THIGH_JOINT) );
    (*this)(1,1) = cos_q_RR_hip_joint;
    (*this)(2,1) = sin_q_RR_hip_joint;
    (*this)(3,1) = (- tx_fr_RR_thigh_COM * sin_q_RR_thigh_joint)-( ty_fr_RR_thigh_COM * cos_q_RR_thigh_joint);
    (*this)(4,0) = ( tx_fr_RR_thigh_COM * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)+( ty_fr_RR_thigh_COM * cos_q_RR_hip_joint * cos_q_RR_thigh_joint)+((- tz_fr_RR_thigh_COM- ty_RR_thigh_joint) * sin_q_RR_hip_joint);
    (*this)(4,1) = ( tx_fr_RR_thigh_COM * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)-( ty_fr_RR_thigh_COM * sin_q_RR_hip_joint * sin_q_RR_thigh_joint);
    (*this)(5,0) = ( tx_fr_RR_thigh_COM * sin_q_RR_hip_joint * sin_q_RR_thigh_joint)+( ty_fr_RR_thigh_COM * sin_q_RR_hip_joint * cos_q_RR_thigh_joint)+(( tz_fr_RR_thigh_COM+ ty_RR_thigh_joint) * cos_q_RR_hip_joint);
    (*this)(5,1) = ( ty_fr_RR_thigh_COM * cos_q_RR_hip_joint * sin_q_RR_thigh_joint)-( tx_fr_RR_thigh_COM * cos_q_RR_hip_joint * cos_q_RR_thigh_joint);
    return *this;
}

UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_thigh_shoulder::Type_fr_base_J_fr_RR_thigh_shoulder()
{
    (*this)(0,0) = 1.0;
    (*this)(1,0) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
}

const UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_thigh_shoulder& UnitreeA1::rcg::Jacobians::Type_fr_base_J_fr_RR_thigh_shoulder::update(const JointState& q)
{
    Scalar sin_q_RR_hip_joint  = ScalarTraits::sin( q(RR_HIP_JOINT) );
    Scalar cos_q_RR_hip_joint  = ScalarTraits::cos( q(RR_HIP_JOINT) );
    (*this)(4,0) = - ty_fr_RR_thigh_shoulder * sin_q_RR_hip_joint;
    (*this)(5,0) =  ty_fr_RR_thigh_shoulder * cos_q_RR_hip_joint;
    return *this;
}

