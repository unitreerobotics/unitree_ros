#include <iit/rbd/robcogen_commons.h>

#include "inverse_dynamics.h"
#include "inertia_properties.h"
#ifndef EIGEN_NO_DEBUG
    #include <iostream>
#endif
using namespace std;
using namespace iit::rbd;
using namespace UnitreeA1::rcg;

// Initialization of static-const data
const UnitreeA1::rcg::InverseDynamics::ExtForces
UnitreeA1::rcg::InverseDynamics::zeroExtForces(Force::Zero());

UnitreeA1::rcg::InverseDynamics::InverseDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    FR_hip_I(inertiaProps->getTensor_FR_hip() ),
    FR_thigh_I(inertiaProps->getTensor_FR_thigh() ),
    FR_calf_I(inertiaProps->getTensor_FR_calf() ),
    FL_hip_I(inertiaProps->getTensor_FL_hip() ),
    FL_thigh_I(inertiaProps->getTensor_FL_thigh() ),
    FL_calf_I(inertiaProps->getTensor_FL_calf() ),
    RR_hip_I(inertiaProps->getTensor_RR_hip() ),
    RR_thigh_I(inertiaProps->getTensor_RR_thigh() ),
    RR_calf_I(inertiaProps->getTensor_RR_calf() ),
    RL_hip_I(inertiaProps->getTensor_RL_hip() ),
    RL_thigh_I(inertiaProps->getTensor_RL_thigh() ),
    RL_calf_I(inertiaProps->getTensor_RL_calf() )
    ,
        base_I( inertiaProps->getTensor_base() ),
        FR_calf_Ic(FR_calf_I),
        FL_calf_Ic(FL_calf_I),
        RR_calf_Ic(RR_calf_I),
        RL_calf_Ic(RL_calf_I)
{
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot UnitreeA1, InverseDynamics::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    FR_hip_v.setZero();
    FR_thigh_v.setZero();
    FR_calf_v.setZero();
    FL_hip_v.setZero();
    FL_thigh_v.setZero();
    FL_calf_v.setZero();
    RR_hip_v.setZero();
    RR_thigh_v.setZero();
    RR_calf_v.setZero();
    RL_hip_v.setZero();
    RL_thigh_v.setZero();
    RL_calf_v.setZero();

    vcross.setZero();
}

void UnitreeA1::rcg::InverseDynamics::id(
    JointState& jForces, Acceleration& base_a,
    const Acceleration& g, const Velocity& base_v,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    base_Ic = base_I;
    FR_hip_Ic = FR_hip_I;
    FR_thigh_Ic = FR_thigh_I;
    FL_hip_Ic = FL_hip_I;
    FL_thigh_Ic = FL_thigh_I;
    RR_hip_Ic = RR_hip_I;
    RR_thigh_Ic = RR_thigh_I;
    RL_hip_Ic = RL_hip_I;
    RL_thigh_Ic = RL_thigh_I;

    // First pass, link 'FR_hip'
    FR_hip_v = ((xm->fr_FR_hip_X_fr_base) * base_v);
    FR_hip_v(iit::rbd::AZ) += qd(FR_HIP_JOINT);
    
    motionCrossProductMx<Scalar>(FR_hip_v, vcross);
    
    FR_hip_a = (vcross.col(iit::rbd::AZ) * qd(FR_HIP_JOINT));
    FR_hip_a(iit::rbd::AZ) += qdd(FR_HIP_JOINT);
    
    FR_hip_f = FR_hip_I * FR_hip_a + vxIv(FR_hip_v, FR_hip_I);
    
    // First pass, link 'FR_thigh'
    FR_thigh_v = ((xm->fr_FR_thigh_X_fr_FR_hip) * FR_hip_v);
    FR_thigh_v(iit::rbd::AZ) += qd(FR_THIGH_JOINT);
    
    motionCrossProductMx<Scalar>(FR_thigh_v, vcross);
    
    FR_thigh_a = (xm->fr_FR_thigh_X_fr_FR_hip) * FR_hip_a + vcross.col(iit::rbd::AZ) * qd(FR_THIGH_JOINT);
    FR_thigh_a(iit::rbd::AZ) += qdd(FR_THIGH_JOINT);
    
    FR_thigh_f = FR_thigh_I * FR_thigh_a + vxIv(FR_thigh_v, FR_thigh_I);
    
    // First pass, link 'FR_calf'
    FR_calf_v = ((xm->fr_FR_calf_X_fr_FR_thigh) * FR_thigh_v);
    FR_calf_v(iit::rbd::AZ) += qd(FR_CALF_JOINT);
    
    motionCrossProductMx<Scalar>(FR_calf_v, vcross);
    
    FR_calf_a = (xm->fr_FR_calf_X_fr_FR_thigh) * FR_thigh_a + vcross.col(iit::rbd::AZ) * qd(FR_CALF_JOINT);
    FR_calf_a(iit::rbd::AZ) += qdd(FR_CALF_JOINT);
    
    FR_calf_f = FR_calf_I * FR_calf_a + vxIv(FR_calf_v, FR_calf_I);
    
    // First pass, link 'FL_hip'
    FL_hip_v = ((xm->fr_FL_hip_X_fr_base) * base_v);
    FL_hip_v(iit::rbd::AZ) += qd(FL_HIP_JOINT);
    
    motionCrossProductMx<Scalar>(FL_hip_v, vcross);
    
    FL_hip_a = (vcross.col(iit::rbd::AZ) * qd(FL_HIP_JOINT));
    FL_hip_a(iit::rbd::AZ) += qdd(FL_HIP_JOINT);
    
    FL_hip_f = FL_hip_I * FL_hip_a + vxIv(FL_hip_v, FL_hip_I);
    
    // First pass, link 'FL_thigh'
    FL_thigh_v = ((xm->fr_FL_thigh_X_fr_FL_hip) * FL_hip_v);
    FL_thigh_v(iit::rbd::AZ) += qd(FL_THIGH_JOINT);
    
    motionCrossProductMx<Scalar>(FL_thigh_v, vcross);
    
    FL_thigh_a = (xm->fr_FL_thigh_X_fr_FL_hip) * FL_hip_a + vcross.col(iit::rbd::AZ) * qd(FL_THIGH_JOINT);
    FL_thigh_a(iit::rbd::AZ) += qdd(FL_THIGH_JOINT);
    
    FL_thigh_f = FL_thigh_I * FL_thigh_a + vxIv(FL_thigh_v, FL_thigh_I);
    
    // First pass, link 'FL_calf'
    FL_calf_v = ((xm->fr_FL_calf_X_fr_FL_thigh) * FL_thigh_v);
    FL_calf_v(iit::rbd::AZ) += qd(FL_CALF_JOINT);
    
    motionCrossProductMx<Scalar>(FL_calf_v, vcross);
    
    FL_calf_a = (xm->fr_FL_calf_X_fr_FL_thigh) * FL_thigh_a + vcross.col(iit::rbd::AZ) * qd(FL_CALF_JOINT);
    FL_calf_a(iit::rbd::AZ) += qdd(FL_CALF_JOINT);
    
    FL_calf_f = FL_calf_I * FL_calf_a + vxIv(FL_calf_v, FL_calf_I);
    
    // First pass, link 'RR_hip'
    RR_hip_v = ((xm->fr_RR_hip_X_fr_base) * base_v);
    RR_hip_v(iit::rbd::AZ) += qd(RR_HIP_JOINT);
    
    motionCrossProductMx<Scalar>(RR_hip_v, vcross);
    
    RR_hip_a = (vcross.col(iit::rbd::AZ) * qd(RR_HIP_JOINT));
    RR_hip_a(iit::rbd::AZ) += qdd(RR_HIP_JOINT);
    
    RR_hip_f = RR_hip_I * RR_hip_a + vxIv(RR_hip_v, RR_hip_I);
    
    // First pass, link 'RR_thigh'
    RR_thigh_v = ((xm->fr_RR_thigh_X_fr_RR_hip) * RR_hip_v);
    RR_thigh_v(iit::rbd::AZ) += qd(RR_THIGH_JOINT);
    
    motionCrossProductMx<Scalar>(RR_thigh_v, vcross);
    
    RR_thigh_a = (xm->fr_RR_thigh_X_fr_RR_hip) * RR_hip_a + vcross.col(iit::rbd::AZ) * qd(RR_THIGH_JOINT);
    RR_thigh_a(iit::rbd::AZ) += qdd(RR_THIGH_JOINT);
    
    RR_thigh_f = RR_thigh_I * RR_thigh_a + vxIv(RR_thigh_v, RR_thigh_I);
    
    // First pass, link 'RR_calf'
    RR_calf_v = ((xm->fr_RR_calf_X_fr_RR_thigh) * RR_thigh_v);
    RR_calf_v(iit::rbd::AZ) += qd(RR_CALF_JOINT);
    
    motionCrossProductMx<Scalar>(RR_calf_v, vcross);
    
    RR_calf_a = (xm->fr_RR_calf_X_fr_RR_thigh) * RR_thigh_a + vcross.col(iit::rbd::AZ) * qd(RR_CALF_JOINT);
    RR_calf_a(iit::rbd::AZ) += qdd(RR_CALF_JOINT);
    
    RR_calf_f = RR_calf_I * RR_calf_a + vxIv(RR_calf_v, RR_calf_I);
    
    // First pass, link 'RL_hip'
    RL_hip_v = ((xm->fr_RL_hip_X_fr_base) * base_v);
    RL_hip_v(iit::rbd::AZ) += qd(RL_HIP_JOINT);
    
    motionCrossProductMx<Scalar>(RL_hip_v, vcross);
    
    RL_hip_a = (vcross.col(iit::rbd::AZ) * qd(RL_HIP_JOINT));
    RL_hip_a(iit::rbd::AZ) += qdd(RL_HIP_JOINT);
    
    RL_hip_f = RL_hip_I * RL_hip_a + vxIv(RL_hip_v, RL_hip_I);
    
    // First pass, link 'RL_thigh'
    RL_thigh_v = ((xm->fr_RL_thigh_X_fr_RL_hip) * RL_hip_v);
    RL_thigh_v(iit::rbd::AZ) += qd(RL_THIGH_JOINT);
    
    motionCrossProductMx<Scalar>(RL_thigh_v, vcross);
    
    RL_thigh_a = (xm->fr_RL_thigh_X_fr_RL_hip) * RL_hip_a + vcross.col(iit::rbd::AZ) * qd(RL_THIGH_JOINT);
    RL_thigh_a(iit::rbd::AZ) += qdd(RL_THIGH_JOINT);
    
    RL_thigh_f = RL_thigh_I * RL_thigh_a + vxIv(RL_thigh_v, RL_thigh_I);
    
    // First pass, link 'RL_calf'
    RL_calf_v = ((xm->fr_RL_calf_X_fr_RL_thigh) * RL_thigh_v);
    RL_calf_v(iit::rbd::AZ) += qd(RL_CALF_JOINT);
    
    motionCrossProductMx<Scalar>(RL_calf_v, vcross);
    
    RL_calf_a = (xm->fr_RL_calf_X_fr_RL_thigh) * RL_thigh_a + vcross.col(iit::rbd::AZ) * qd(RL_CALF_JOINT);
    RL_calf_a(iit::rbd::AZ) += qdd(RL_CALF_JOINT);
    
    RL_calf_f = RL_calf_I * RL_calf_a + vxIv(RL_calf_v, RL_calf_I);
    
    // The force exerted on the floating base by the links
    base_f = vxIv(base_v, base_I);
    

    // Add the external forces:
    base_f -= fext[BASE];
    FR_hip_f -= fext[FR_HIP];
    FR_thigh_f -= fext[FR_THIGH];
    FR_calf_f -= fext[FR_CALF];
    FL_hip_f -= fext[FL_HIP];
    FL_thigh_f -= fext[FL_THIGH];
    FL_calf_f -= fext[FL_CALF];
    RR_hip_f -= fext[RR_HIP];
    RR_thigh_f -= fext[RR_THIGH];
    RR_calf_f -= fext[RR_CALF];
    RL_hip_f -= fext[RL_HIP];
    RL_thigh_f -= fext[RL_THIGH];
    RL_calf_f -= fext[RL_CALF];

    InertiaMatrix Ic_spare;
    iit::rbd::transformInertia<Scalar>(RL_calf_Ic, (xm->fr_RL_calf_X_fr_RL_thigh).transpose(), Ic_spare);
    RL_thigh_Ic += Ic_spare;
    RL_thigh_f = RL_thigh_f + (xm->fr_RL_calf_X_fr_RL_thigh).transpose() * RL_calf_f;
    
    iit::rbd::transformInertia<Scalar>(RL_thigh_Ic, (xm->fr_RL_thigh_X_fr_RL_hip).transpose(), Ic_spare);
    RL_hip_Ic += Ic_spare;
    RL_hip_f = RL_hip_f + (xm->fr_RL_thigh_X_fr_RL_hip).transpose() * RL_thigh_f;
    
    iit::rbd::transformInertia<Scalar>(RL_hip_Ic, (xm->fr_RL_hip_X_fr_base).transpose(), Ic_spare);
    base_Ic += Ic_spare;
    base_f = base_f + (xm->fr_RL_hip_X_fr_base).transpose() * RL_hip_f;
    
    iit::rbd::transformInertia<Scalar>(RR_calf_Ic, (xm->fr_RR_calf_X_fr_RR_thigh).transpose(), Ic_spare);
    RR_thigh_Ic += Ic_spare;
    RR_thigh_f = RR_thigh_f + (xm->fr_RR_calf_X_fr_RR_thigh).transpose() * RR_calf_f;
    
    iit::rbd::transformInertia<Scalar>(RR_thigh_Ic, (xm->fr_RR_thigh_X_fr_RR_hip).transpose(), Ic_spare);
    RR_hip_Ic += Ic_spare;
    RR_hip_f = RR_hip_f + (xm->fr_RR_thigh_X_fr_RR_hip).transpose() * RR_thigh_f;
    
    iit::rbd::transformInertia<Scalar>(RR_hip_Ic, (xm->fr_RR_hip_X_fr_base).transpose(), Ic_spare);
    base_Ic += Ic_spare;
    base_f = base_f + (xm->fr_RR_hip_X_fr_base).transpose() * RR_hip_f;
    
    iit::rbd::transformInertia<Scalar>(FL_calf_Ic, (xm->fr_FL_calf_X_fr_FL_thigh).transpose(), Ic_spare);
    FL_thigh_Ic += Ic_spare;
    FL_thigh_f = FL_thigh_f + (xm->fr_FL_calf_X_fr_FL_thigh).transpose() * FL_calf_f;
    
    iit::rbd::transformInertia<Scalar>(FL_thigh_Ic, (xm->fr_FL_thigh_X_fr_FL_hip).transpose(), Ic_spare);
    FL_hip_Ic += Ic_spare;
    FL_hip_f = FL_hip_f + (xm->fr_FL_thigh_X_fr_FL_hip).transpose() * FL_thigh_f;
    
    iit::rbd::transformInertia<Scalar>(FL_hip_Ic, (xm->fr_FL_hip_X_fr_base).transpose(), Ic_spare);
    base_Ic += Ic_spare;
    base_f = base_f + (xm->fr_FL_hip_X_fr_base).transpose() * FL_hip_f;
    
    iit::rbd::transformInertia<Scalar>(FR_calf_Ic, (xm->fr_FR_calf_X_fr_FR_thigh).transpose(), Ic_spare);
    FR_thigh_Ic += Ic_spare;
    FR_thigh_f = FR_thigh_f + (xm->fr_FR_calf_X_fr_FR_thigh).transpose() * FR_calf_f;
    
    iit::rbd::transformInertia<Scalar>(FR_thigh_Ic, (xm->fr_FR_thigh_X_fr_FR_hip).transpose(), Ic_spare);
    FR_hip_Ic += Ic_spare;
    FR_hip_f = FR_hip_f + (xm->fr_FR_thigh_X_fr_FR_hip).transpose() * FR_thigh_f;
    
    iit::rbd::transformInertia<Scalar>(FR_hip_Ic, (xm->fr_FR_hip_X_fr_base).transpose(), Ic_spare);
    base_Ic += Ic_spare;
    base_f = base_f + (xm->fr_FR_hip_X_fr_base).transpose() * FR_hip_f;
    

    // The base acceleration due to the force due to the movement of the links
    base_a = - base_Ic.inverse() * base_f;
    
    FR_hip_a = xm->fr_FR_hip_X_fr_base * base_a;
    jForces(FR_HIP_JOINT) = (FR_hip_Ic.row(iit::rbd::AZ) * FR_hip_a + FR_hip_f(iit::rbd::AZ));
    
    FR_thigh_a = xm->fr_FR_thigh_X_fr_FR_hip * FR_hip_a;
    jForces(FR_THIGH_JOINT) = (FR_thigh_Ic.row(iit::rbd::AZ) * FR_thigh_a + FR_thigh_f(iit::rbd::AZ));
    
    FR_calf_a = xm->fr_FR_calf_X_fr_FR_thigh * FR_thigh_a;
    jForces(FR_CALF_JOINT) = (FR_calf_Ic.row(iit::rbd::AZ) * FR_calf_a + FR_calf_f(iit::rbd::AZ));
    
    FL_hip_a = xm->fr_FL_hip_X_fr_base * base_a;
    jForces(FL_HIP_JOINT) = (FL_hip_Ic.row(iit::rbd::AZ) * FL_hip_a + FL_hip_f(iit::rbd::AZ));
    
    FL_thigh_a = xm->fr_FL_thigh_X_fr_FL_hip * FL_hip_a;
    jForces(FL_THIGH_JOINT) = (FL_thigh_Ic.row(iit::rbd::AZ) * FL_thigh_a + FL_thigh_f(iit::rbd::AZ));
    
    FL_calf_a = xm->fr_FL_calf_X_fr_FL_thigh * FL_thigh_a;
    jForces(FL_CALF_JOINT) = (FL_calf_Ic.row(iit::rbd::AZ) * FL_calf_a + FL_calf_f(iit::rbd::AZ));
    
    RR_hip_a = xm->fr_RR_hip_X_fr_base * base_a;
    jForces(RR_HIP_JOINT) = (RR_hip_Ic.row(iit::rbd::AZ) * RR_hip_a + RR_hip_f(iit::rbd::AZ));
    
    RR_thigh_a = xm->fr_RR_thigh_X_fr_RR_hip * RR_hip_a;
    jForces(RR_THIGH_JOINT) = (RR_thigh_Ic.row(iit::rbd::AZ) * RR_thigh_a + RR_thigh_f(iit::rbd::AZ));
    
    RR_calf_a = xm->fr_RR_calf_X_fr_RR_thigh * RR_thigh_a;
    jForces(RR_CALF_JOINT) = (RR_calf_Ic.row(iit::rbd::AZ) * RR_calf_a + RR_calf_f(iit::rbd::AZ));
    
    RL_hip_a = xm->fr_RL_hip_X_fr_base * base_a;
    jForces(RL_HIP_JOINT) = (RL_hip_Ic.row(iit::rbd::AZ) * RL_hip_a + RL_hip_f(iit::rbd::AZ));
    
    RL_thigh_a = xm->fr_RL_thigh_X_fr_RL_hip * RL_hip_a;
    jForces(RL_THIGH_JOINT) = (RL_thigh_Ic.row(iit::rbd::AZ) * RL_thigh_a + RL_thigh_f(iit::rbd::AZ));
    
    RL_calf_a = xm->fr_RL_calf_X_fr_RL_thigh * RL_thigh_a;
    jForces(RL_CALF_JOINT) = (RL_calf_Ic.row(iit::rbd::AZ) * RL_calf_a + RL_calf_f(iit::rbd::AZ));
    

    base_a += g;
}


void UnitreeA1::rcg::InverseDynamics::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g)
{
    const Acceleration& base_a = -g;

    // Link 'FR_hip'
    FR_hip_a = (xm->fr_FR_hip_X_fr_base) * base_a;
    FR_hip_f = FR_hip_I * FR_hip_a;
    // Link 'FR_thigh'
    FR_thigh_a = (xm->fr_FR_thigh_X_fr_FR_hip) * FR_hip_a;
    FR_thigh_f = FR_thigh_I * FR_thigh_a;
    // Link 'FR_calf'
    FR_calf_a = (xm->fr_FR_calf_X_fr_FR_thigh) * FR_thigh_a;
    FR_calf_f = FR_calf_I * FR_calf_a;
    // Link 'FL_hip'
    FL_hip_a = (xm->fr_FL_hip_X_fr_base) * base_a;
    FL_hip_f = FL_hip_I * FL_hip_a;
    // Link 'FL_thigh'
    FL_thigh_a = (xm->fr_FL_thigh_X_fr_FL_hip) * FL_hip_a;
    FL_thigh_f = FL_thigh_I * FL_thigh_a;
    // Link 'FL_calf'
    FL_calf_a = (xm->fr_FL_calf_X_fr_FL_thigh) * FL_thigh_a;
    FL_calf_f = FL_calf_I * FL_calf_a;
    // Link 'RR_hip'
    RR_hip_a = (xm->fr_RR_hip_X_fr_base) * base_a;
    RR_hip_f = RR_hip_I * RR_hip_a;
    // Link 'RR_thigh'
    RR_thigh_a = (xm->fr_RR_thigh_X_fr_RR_hip) * RR_hip_a;
    RR_thigh_f = RR_thigh_I * RR_thigh_a;
    // Link 'RR_calf'
    RR_calf_a = (xm->fr_RR_calf_X_fr_RR_thigh) * RR_thigh_a;
    RR_calf_f = RR_calf_I * RR_calf_a;
    // Link 'RL_hip'
    RL_hip_a = (xm->fr_RL_hip_X_fr_base) * base_a;
    RL_hip_f = RL_hip_I * RL_hip_a;
    // Link 'RL_thigh'
    RL_thigh_a = (xm->fr_RL_thigh_X_fr_RL_hip) * RL_hip_a;
    RL_thigh_f = RL_thigh_I * RL_thigh_a;
    // Link 'RL_calf'
    RL_calf_a = (xm->fr_RL_calf_X_fr_RL_thigh) * RL_thigh_a;
    RL_calf_f = RL_calf_I * RL_calf_a;

    base_f = base_I * base_a;

    secondPass_fullyActuated(jForces);

    baseWrench = base_f;
}

void UnitreeA1::rcg::InverseDynamics::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& base_v, const JointState& qd)
{
    // Link 'FR_hip'
    FR_hip_v = ((xm->fr_FR_hip_X_fr_base) * base_v);
    FR_hip_v(iit::rbd::AZ) += qd(FR_HIP_JOINT);
    motionCrossProductMx<Scalar>(FR_hip_v, vcross);
    FR_hip_a = (vcross.col(iit::rbd::AZ) * qd(FR_HIP_JOINT));
    FR_hip_f = FR_hip_I * FR_hip_a + vxIv(FR_hip_v, FR_hip_I);
    
    // Link 'FR_thigh'
    FR_thigh_v = ((xm->fr_FR_thigh_X_fr_FR_hip) * FR_hip_v);
    FR_thigh_v(iit::rbd::AZ) += qd(FR_THIGH_JOINT);
    motionCrossProductMx<Scalar>(FR_thigh_v, vcross);
    FR_thigh_a = (xm->fr_FR_thigh_X_fr_FR_hip) * FR_hip_a + vcross.col(iit::rbd::AZ) * qd(FR_THIGH_JOINT);
    FR_thigh_f = FR_thigh_I * FR_thigh_a + vxIv(FR_thigh_v, FR_thigh_I);
    
    // Link 'FR_calf'
    FR_calf_v = ((xm->fr_FR_calf_X_fr_FR_thigh) * FR_thigh_v);
    FR_calf_v(iit::rbd::AZ) += qd(FR_CALF_JOINT);
    motionCrossProductMx<Scalar>(FR_calf_v, vcross);
    FR_calf_a = (xm->fr_FR_calf_X_fr_FR_thigh) * FR_thigh_a + vcross.col(iit::rbd::AZ) * qd(FR_CALF_JOINT);
    FR_calf_f = FR_calf_I * FR_calf_a + vxIv(FR_calf_v, FR_calf_I);
    
    // Link 'FL_hip'
    FL_hip_v = ((xm->fr_FL_hip_X_fr_base) * base_v);
    FL_hip_v(iit::rbd::AZ) += qd(FL_HIP_JOINT);
    motionCrossProductMx<Scalar>(FL_hip_v, vcross);
    FL_hip_a = (vcross.col(iit::rbd::AZ) * qd(FL_HIP_JOINT));
    FL_hip_f = FL_hip_I * FL_hip_a + vxIv(FL_hip_v, FL_hip_I);
    
    // Link 'FL_thigh'
    FL_thigh_v = ((xm->fr_FL_thigh_X_fr_FL_hip) * FL_hip_v);
    FL_thigh_v(iit::rbd::AZ) += qd(FL_THIGH_JOINT);
    motionCrossProductMx<Scalar>(FL_thigh_v, vcross);
    FL_thigh_a = (xm->fr_FL_thigh_X_fr_FL_hip) * FL_hip_a + vcross.col(iit::rbd::AZ) * qd(FL_THIGH_JOINT);
    FL_thigh_f = FL_thigh_I * FL_thigh_a + vxIv(FL_thigh_v, FL_thigh_I);
    
    // Link 'FL_calf'
    FL_calf_v = ((xm->fr_FL_calf_X_fr_FL_thigh) * FL_thigh_v);
    FL_calf_v(iit::rbd::AZ) += qd(FL_CALF_JOINT);
    motionCrossProductMx<Scalar>(FL_calf_v, vcross);
    FL_calf_a = (xm->fr_FL_calf_X_fr_FL_thigh) * FL_thigh_a + vcross.col(iit::rbd::AZ) * qd(FL_CALF_JOINT);
    FL_calf_f = FL_calf_I * FL_calf_a + vxIv(FL_calf_v, FL_calf_I);
    
    // Link 'RR_hip'
    RR_hip_v = ((xm->fr_RR_hip_X_fr_base) * base_v);
    RR_hip_v(iit::rbd::AZ) += qd(RR_HIP_JOINT);
    motionCrossProductMx<Scalar>(RR_hip_v, vcross);
    RR_hip_a = (vcross.col(iit::rbd::AZ) * qd(RR_HIP_JOINT));
    RR_hip_f = RR_hip_I * RR_hip_a + vxIv(RR_hip_v, RR_hip_I);
    
    // Link 'RR_thigh'
    RR_thigh_v = ((xm->fr_RR_thigh_X_fr_RR_hip) * RR_hip_v);
    RR_thigh_v(iit::rbd::AZ) += qd(RR_THIGH_JOINT);
    motionCrossProductMx<Scalar>(RR_thigh_v, vcross);
    RR_thigh_a = (xm->fr_RR_thigh_X_fr_RR_hip) * RR_hip_a + vcross.col(iit::rbd::AZ) * qd(RR_THIGH_JOINT);
    RR_thigh_f = RR_thigh_I * RR_thigh_a + vxIv(RR_thigh_v, RR_thigh_I);
    
    // Link 'RR_calf'
    RR_calf_v = ((xm->fr_RR_calf_X_fr_RR_thigh) * RR_thigh_v);
    RR_calf_v(iit::rbd::AZ) += qd(RR_CALF_JOINT);
    motionCrossProductMx<Scalar>(RR_calf_v, vcross);
    RR_calf_a = (xm->fr_RR_calf_X_fr_RR_thigh) * RR_thigh_a + vcross.col(iit::rbd::AZ) * qd(RR_CALF_JOINT);
    RR_calf_f = RR_calf_I * RR_calf_a + vxIv(RR_calf_v, RR_calf_I);
    
    // Link 'RL_hip'
    RL_hip_v = ((xm->fr_RL_hip_X_fr_base) * base_v);
    RL_hip_v(iit::rbd::AZ) += qd(RL_HIP_JOINT);
    motionCrossProductMx<Scalar>(RL_hip_v, vcross);
    RL_hip_a = (vcross.col(iit::rbd::AZ) * qd(RL_HIP_JOINT));
    RL_hip_f = RL_hip_I * RL_hip_a + vxIv(RL_hip_v, RL_hip_I);
    
    // Link 'RL_thigh'
    RL_thigh_v = ((xm->fr_RL_thigh_X_fr_RL_hip) * RL_hip_v);
    RL_thigh_v(iit::rbd::AZ) += qd(RL_THIGH_JOINT);
    motionCrossProductMx<Scalar>(RL_thigh_v, vcross);
    RL_thigh_a = (xm->fr_RL_thigh_X_fr_RL_hip) * RL_hip_a + vcross.col(iit::rbd::AZ) * qd(RL_THIGH_JOINT);
    RL_thigh_f = RL_thigh_I * RL_thigh_a + vxIv(RL_thigh_v, RL_thigh_I);
    
    // Link 'RL_calf'
    RL_calf_v = ((xm->fr_RL_calf_X_fr_RL_thigh) * RL_thigh_v);
    RL_calf_v(iit::rbd::AZ) += qd(RL_CALF_JOINT);
    motionCrossProductMx<Scalar>(RL_calf_v, vcross);
    RL_calf_a = (xm->fr_RL_calf_X_fr_RL_thigh) * RL_thigh_a + vcross.col(iit::rbd::AZ) * qd(RL_CALF_JOINT);
    RL_calf_f = RL_calf_I * RL_calf_a + vxIv(RL_calf_v, RL_calf_I);
    

    base_f = vxIv(base_v, base_I);

    secondPass_fullyActuated(jForces);

    baseWrench = base_f;
}

void UnitreeA1::rcg::InverseDynamics::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    Acceleration base_a = baseAccel -g;

    // First pass, link 'FR_hip'
    FR_hip_v = ((xm->fr_FR_hip_X_fr_base) * base_v);
    FR_hip_v(iit::rbd::AZ) += qd(FR_HIP_JOINT);
    
    motionCrossProductMx<Scalar>(FR_hip_v, vcross);
    
    FR_hip_a = (xm->fr_FR_hip_X_fr_base) * base_a + vcross.col(iit::rbd::AZ) * qd(FR_HIP_JOINT);
    FR_hip_a(iit::rbd::AZ) += qdd(FR_HIP_JOINT);
    
    FR_hip_f = FR_hip_I * FR_hip_a + vxIv(FR_hip_v, FR_hip_I) - fext[FR_HIP];
    
    // First pass, link 'FR_thigh'
    FR_thigh_v = ((xm->fr_FR_thigh_X_fr_FR_hip) * FR_hip_v);
    FR_thigh_v(iit::rbd::AZ) += qd(FR_THIGH_JOINT);
    
    motionCrossProductMx<Scalar>(FR_thigh_v, vcross);
    
    FR_thigh_a = (xm->fr_FR_thigh_X_fr_FR_hip) * FR_hip_a + vcross.col(iit::rbd::AZ) * qd(FR_THIGH_JOINT);
    FR_thigh_a(iit::rbd::AZ) += qdd(FR_THIGH_JOINT);
    
    FR_thigh_f = FR_thigh_I * FR_thigh_a + vxIv(FR_thigh_v, FR_thigh_I) - fext[FR_THIGH];
    
    // First pass, link 'FR_calf'
    FR_calf_v = ((xm->fr_FR_calf_X_fr_FR_thigh) * FR_thigh_v);
    FR_calf_v(iit::rbd::AZ) += qd(FR_CALF_JOINT);
    
    motionCrossProductMx<Scalar>(FR_calf_v, vcross);
    
    FR_calf_a = (xm->fr_FR_calf_X_fr_FR_thigh) * FR_thigh_a + vcross.col(iit::rbd::AZ) * qd(FR_CALF_JOINT);
    FR_calf_a(iit::rbd::AZ) += qdd(FR_CALF_JOINT);
    
    FR_calf_f = FR_calf_I * FR_calf_a + vxIv(FR_calf_v, FR_calf_I) - fext[FR_CALF];
    
    // First pass, link 'FL_hip'
    FL_hip_v = ((xm->fr_FL_hip_X_fr_base) * base_v);
    FL_hip_v(iit::rbd::AZ) += qd(FL_HIP_JOINT);
    
    motionCrossProductMx<Scalar>(FL_hip_v, vcross);
    
    FL_hip_a = (xm->fr_FL_hip_X_fr_base) * base_a + vcross.col(iit::rbd::AZ) * qd(FL_HIP_JOINT);
    FL_hip_a(iit::rbd::AZ) += qdd(FL_HIP_JOINT);
    
    FL_hip_f = FL_hip_I * FL_hip_a + vxIv(FL_hip_v, FL_hip_I) - fext[FL_HIP];
    
    // First pass, link 'FL_thigh'
    FL_thigh_v = ((xm->fr_FL_thigh_X_fr_FL_hip) * FL_hip_v);
    FL_thigh_v(iit::rbd::AZ) += qd(FL_THIGH_JOINT);
    
    motionCrossProductMx<Scalar>(FL_thigh_v, vcross);
    
    FL_thigh_a = (xm->fr_FL_thigh_X_fr_FL_hip) * FL_hip_a + vcross.col(iit::rbd::AZ) * qd(FL_THIGH_JOINT);
    FL_thigh_a(iit::rbd::AZ) += qdd(FL_THIGH_JOINT);
    
    FL_thigh_f = FL_thigh_I * FL_thigh_a + vxIv(FL_thigh_v, FL_thigh_I) - fext[FL_THIGH];
    
    // First pass, link 'FL_calf'
    FL_calf_v = ((xm->fr_FL_calf_X_fr_FL_thigh) * FL_thigh_v);
    FL_calf_v(iit::rbd::AZ) += qd(FL_CALF_JOINT);
    
    motionCrossProductMx<Scalar>(FL_calf_v, vcross);
    
    FL_calf_a = (xm->fr_FL_calf_X_fr_FL_thigh) * FL_thigh_a + vcross.col(iit::rbd::AZ) * qd(FL_CALF_JOINT);
    FL_calf_a(iit::rbd::AZ) += qdd(FL_CALF_JOINT);
    
    FL_calf_f = FL_calf_I * FL_calf_a + vxIv(FL_calf_v, FL_calf_I) - fext[FL_CALF];
    
    // First pass, link 'RR_hip'
    RR_hip_v = ((xm->fr_RR_hip_X_fr_base) * base_v);
    RR_hip_v(iit::rbd::AZ) += qd(RR_HIP_JOINT);
    
    motionCrossProductMx<Scalar>(RR_hip_v, vcross);
    
    RR_hip_a = (xm->fr_RR_hip_X_fr_base) * base_a + vcross.col(iit::rbd::AZ) * qd(RR_HIP_JOINT);
    RR_hip_a(iit::rbd::AZ) += qdd(RR_HIP_JOINT);
    
    RR_hip_f = RR_hip_I * RR_hip_a + vxIv(RR_hip_v, RR_hip_I) - fext[RR_HIP];
    
    // First pass, link 'RR_thigh'
    RR_thigh_v = ((xm->fr_RR_thigh_X_fr_RR_hip) * RR_hip_v);
    RR_thigh_v(iit::rbd::AZ) += qd(RR_THIGH_JOINT);
    
    motionCrossProductMx<Scalar>(RR_thigh_v, vcross);
    
    RR_thigh_a = (xm->fr_RR_thigh_X_fr_RR_hip) * RR_hip_a + vcross.col(iit::rbd::AZ) * qd(RR_THIGH_JOINT);
    RR_thigh_a(iit::rbd::AZ) += qdd(RR_THIGH_JOINT);
    
    RR_thigh_f = RR_thigh_I * RR_thigh_a + vxIv(RR_thigh_v, RR_thigh_I) - fext[RR_THIGH];
    
    // First pass, link 'RR_calf'
    RR_calf_v = ((xm->fr_RR_calf_X_fr_RR_thigh) * RR_thigh_v);
    RR_calf_v(iit::rbd::AZ) += qd(RR_CALF_JOINT);
    
    motionCrossProductMx<Scalar>(RR_calf_v, vcross);
    
    RR_calf_a = (xm->fr_RR_calf_X_fr_RR_thigh) * RR_thigh_a + vcross.col(iit::rbd::AZ) * qd(RR_CALF_JOINT);
    RR_calf_a(iit::rbd::AZ) += qdd(RR_CALF_JOINT);
    
    RR_calf_f = RR_calf_I * RR_calf_a + vxIv(RR_calf_v, RR_calf_I) - fext[RR_CALF];
    
    // First pass, link 'RL_hip'
    RL_hip_v = ((xm->fr_RL_hip_X_fr_base) * base_v);
    RL_hip_v(iit::rbd::AZ) += qd(RL_HIP_JOINT);
    
    motionCrossProductMx<Scalar>(RL_hip_v, vcross);
    
    RL_hip_a = (xm->fr_RL_hip_X_fr_base) * base_a + vcross.col(iit::rbd::AZ) * qd(RL_HIP_JOINT);
    RL_hip_a(iit::rbd::AZ) += qdd(RL_HIP_JOINT);
    
    RL_hip_f = RL_hip_I * RL_hip_a + vxIv(RL_hip_v, RL_hip_I) - fext[RL_HIP];
    
    // First pass, link 'RL_thigh'
    RL_thigh_v = ((xm->fr_RL_thigh_X_fr_RL_hip) * RL_hip_v);
    RL_thigh_v(iit::rbd::AZ) += qd(RL_THIGH_JOINT);
    
    motionCrossProductMx<Scalar>(RL_thigh_v, vcross);
    
    RL_thigh_a = (xm->fr_RL_thigh_X_fr_RL_hip) * RL_hip_a + vcross.col(iit::rbd::AZ) * qd(RL_THIGH_JOINT);
    RL_thigh_a(iit::rbd::AZ) += qdd(RL_THIGH_JOINT);
    
    RL_thigh_f = RL_thigh_I * RL_thigh_a + vxIv(RL_thigh_v, RL_thigh_I) - fext[RL_THIGH];
    
    // First pass, link 'RL_calf'
    RL_calf_v = ((xm->fr_RL_calf_X_fr_RL_thigh) * RL_thigh_v);
    RL_calf_v(iit::rbd::AZ) += qd(RL_CALF_JOINT);
    
    motionCrossProductMx<Scalar>(RL_calf_v, vcross);
    
    RL_calf_a = (xm->fr_RL_calf_X_fr_RL_thigh) * RL_thigh_a + vcross.col(iit::rbd::AZ) * qd(RL_CALF_JOINT);
    RL_calf_a(iit::rbd::AZ) += qdd(RL_CALF_JOINT);
    
    RL_calf_f = RL_calf_I * RL_calf_a + vxIv(RL_calf_v, RL_calf_I) - fext[RL_CALF];
    

    // The base
    base_f = base_I * base_a + vxIv(base_v, base_I) - fext[BASE];

    secondPass_fullyActuated(jForces);

    baseWrench = base_f;
}


void UnitreeA1::rcg::InverseDynamics::secondPass_fullyActuated(JointState& jForces)
{
    // Link 'RL_calf'
    jForces(RL_CALF_JOINT) = RL_calf_f(iit::rbd::AZ);
    RL_thigh_f += xm->fr_RL_calf_X_fr_RL_thigh.transpose() * RL_calf_f;
    // Link 'RL_thigh'
    jForces(RL_THIGH_JOINT) = RL_thigh_f(iit::rbd::AZ);
    RL_hip_f += xm->fr_RL_thigh_X_fr_RL_hip.transpose() * RL_thigh_f;
    // Link 'RL_hip'
    jForces(RL_HIP_JOINT) = RL_hip_f(iit::rbd::AZ);
    base_f += xm->fr_RL_hip_X_fr_base.transpose() * RL_hip_f;
    // Link 'RR_calf'
    jForces(RR_CALF_JOINT) = RR_calf_f(iit::rbd::AZ);
    RR_thigh_f += xm->fr_RR_calf_X_fr_RR_thigh.transpose() * RR_calf_f;
    // Link 'RR_thigh'
    jForces(RR_THIGH_JOINT) = RR_thigh_f(iit::rbd::AZ);
    RR_hip_f += xm->fr_RR_thigh_X_fr_RR_hip.transpose() * RR_thigh_f;
    // Link 'RR_hip'
    jForces(RR_HIP_JOINT) = RR_hip_f(iit::rbd::AZ);
    base_f += xm->fr_RR_hip_X_fr_base.transpose() * RR_hip_f;
    // Link 'FL_calf'
    jForces(FL_CALF_JOINT) = FL_calf_f(iit::rbd::AZ);
    FL_thigh_f += xm->fr_FL_calf_X_fr_FL_thigh.transpose() * FL_calf_f;
    // Link 'FL_thigh'
    jForces(FL_THIGH_JOINT) = FL_thigh_f(iit::rbd::AZ);
    FL_hip_f += xm->fr_FL_thigh_X_fr_FL_hip.transpose() * FL_thigh_f;
    // Link 'FL_hip'
    jForces(FL_HIP_JOINT) = FL_hip_f(iit::rbd::AZ);
    base_f += xm->fr_FL_hip_X_fr_base.transpose() * FL_hip_f;
    // Link 'FR_calf'
    jForces(FR_CALF_JOINT) = FR_calf_f(iit::rbd::AZ);
    FR_thigh_f += xm->fr_FR_calf_X_fr_FR_thigh.transpose() * FR_calf_f;
    // Link 'FR_thigh'
    jForces(FR_THIGH_JOINT) = FR_thigh_f(iit::rbd::AZ);
    FR_hip_f += xm->fr_FR_thigh_X_fr_FR_hip.transpose() * FR_thigh_f;
    // Link 'FR_hip'
    jForces(FR_HIP_JOINT) = FR_hip_f(iit::rbd::AZ);
    base_f += xm->fr_FR_hip_X_fr_base.transpose() * FR_hip_f;
}
