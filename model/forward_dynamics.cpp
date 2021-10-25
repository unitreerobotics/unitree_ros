#include "forward_dynamics.h"

#include <Eigen/Cholesky>
#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

// Initialization of static-const data
const UnitreeA1::rcg::ForwardDynamics::ExtForces
    UnitreeA1::rcg::ForwardDynamics::zeroExtForces(Force::Zero());

UnitreeA1::rcg::ForwardDynamics::ForwardDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    FR_hip_v.setZero();
    FR_hip_c.setZero();
    FR_thigh_v.setZero();
    FR_thigh_c.setZero();
    FR_calf_v.setZero();
    FR_calf_c.setZero();
    FL_hip_v.setZero();
    FL_hip_c.setZero();
    FL_thigh_v.setZero();
    FL_thigh_c.setZero();
    FL_calf_v.setZero();
    FL_calf_c.setZero();
    RR_hip_v.setZero();
    RR_hip_c.setZero();
    RR_thigh_v.setZero();
    RR_thigh_c.setZero();
    RR_calf_v.setZero();
    RR_calf_c.setZero();
    RL_hip_v.setZero();
    RL_hip_c.setZero();
    RL_thigh_v.setZero();
    RL_thigh_c.setZero();
    RL_calf_v.setZero();
    RL_calf_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

void UnitreeA1::rcg::ForwardDynamics::fd(
    JointState& qdd,
    Acceleration& base_a,
    const Velocity& base_v,
    const Acceleration& g,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    base_AI = inertiaProps->getTensor_base();
    base_p = - fext[BASE];
    FR_hip_AI = inertiaProps->getTensor_FR_hip();
    FR_hip_p = - fext[FR_HIP];
    FR_thigh_AI = inertiaProps->getTensor_FR_thigh();
    FR_thigh_p = - fext[FR_THIGH];
    FR_calf_AI = inertiaProps->getTensor_FR_calf();
    FR_calf_p = - fext[FR_CALF];
    FL_hip_AI = inertiaProps->getTensor_FL_hip();
    FL_hip_p = - fext[FL_HIP];
    FL_thigh_AI = inertiaProps->getTensor_FL_thigh();
    FL_thigh_p = - fext[FL_THIGH];
    FL_calf_AI = inertiaProps->getTensor_FL_calf();
    FL_calf_p = - fext[FL_CALF];
    RR_hip_AI = inertiaProps->getTensor_RR_hip();
    RR_hip_p = - fext[RR_HIP];
    RR_thigh_AI = inertiaProps->getTensor_RR_thigh();
    RR_thigh_p = - fext[RR_THIGH];
    RR_calf_AI = inertiaProps->getTensor_RR_calf();
    RR_calf_p = - fext[RR_CALF];
    RL_hip_AI = inertiaProps->getTensor_RL_hip();
    RL_hip_p = - fext[RL_HIP];
    RL_thigh_AI = inertiaProps->getTensor_RL_thigh();
    RL_thigh_p = - fext[RL_THIGH];
    RL_calf_AI = inertiaProps->getTensor_RL_calf();
    RL_calf_p = - fext[RL_CALF];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link FR_hip
    //  - The spatial velocity:
    FR_hip_v = (motionTransforms-> fr_FR_hip_X_fr_base) * base_v;
    FR_hip_v(AZ) += qd(FR_HIP_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(FR_hip_v, vcross);
    FR_hip_c = vcross.col(AZ) * qd(FR_HIP_JOINT);
    
    //  - The bias force term:
    FR_hip_p += vxIv(FR_hip_v, FR_hip_AI);
    
    // + Link FR_thigh
    //  - The spatial velocity:
    FR_thigh_v = (motionTransforms-> fr_FR_thigh_X_fr_FR_hip) * FR_hip_v;
    FR_thigh_v(AZ) += qd(FR_THIGH_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(FR_thigh_v, vcross);
    FR_thigh_c = vcross.col(AZ) * qd(FR_THIGH_JOINT);
    
    //  - The bias force term:
    FR_thigh_p += vxIv(FR_thigh_v, FR_thigh_AI);
    
    // + Link FR_calf
    //  - The spatial velocity:
    FR_calf_v = (motionTransforms-> fr_FR_calf_X_fr_FR_thigh) * FR_thigh_v;
    FR_calf_v(AZ) += qd(FR_CALF_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(FR_calf_v, vcross);
    FR_calf_c = vcross.col(AZ) * qd(FR_CALF_JOINT);
    
    //  - The bias force term:
    FR_calf_p += vxIv(FR_calf_v, FR_calf_AI);
    
    // + Link FL_hip
    //  - The spatial velocity:
    FL_hip_v = (motionTransforms-> fr_FL_hip_X_fr_base) * base_v;
    FL_hip_v(AZ) += qd(FL_HIP_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(FL_hip_v, vcross);
    FL_hip_c = vcross.col(AZ) * qd(FL_HIP_JOINT);
    
    //  - The bias force term:
    FL_hip_p += vxIv(FL_hip_v, FL_hip_AI);
    
    // + Link FL_thigh
    //  - The spatial velocity:
    FL_thigh_v = (motionTransforms-> fr_FL_thigh_X_fr_FL_hip) * FL_hip_v;
    FL_thigh_v(AZ) += qd(FL_THIGH_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(FL_thigh_v, vcross);
    FL_thigh_c = vcross.col(AZ) * qd(FL_THIGH_JOINT);
    
    //  - The bias force term:
    FL_thigh_p += vxIv(FL_thigh_v, FL_thigh_AI);
    
    // + Link FL_calf
    //  - The spatial velocity:
    FL_calf_v = (motionTransforms-> fr_FL_calf_X_fr_FL_thigh) * FL_thigh_v;
    FL_calf_v(AZ) += qd(FL_CALF_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(FL_calf_v, vcross);
    FL_calf_c = vcross.col(AZ) * qd(FL_CALF_JOINT);
    
    //  - The bias force term:
    FL_calf_p += vxIv(FL_calf_v, FL_calf_AI);
    
    // + Link RR_hip
    //  - The spatial velocity:
    RR_hip_v = (motionTransforms-> fr_RR_hip_X_fr_base) * base_v;
    RR_hip_v(AZ) += qd(RR_HIP_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(RR_hip_v, vcross);
    RR_hip_c = vcross.col(AZ) * qd(RR_HIP_JOINT);
    
    //  - The bias force term:
    RR_hip_p += vxIv(RR_hip_v, RR_hip_AI);
    
    // + Link RR_thigh
    //  - The spatial velocity:
    RR_thigh_v = (motionTransforms-> fr_RR_thigh_X_fr_RR_hip) * RR_hip_v;
    RR_thigh_v(AZ) += qd(RR_THIGH_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(RR_thigh_v, vcross);
    RR_thigh_c = vcross.col(AZ) * qd(RR_THIGH_JOINT);
    
    //  - The bias force term:
    RR_thigh_p += vxIv(RR_thigh_v, RR_thigh_AI);
    
    // + Link RR_calf
    //  - The spatial velocity:
    RR_calf_v = (motionTransforms-> fr_RR_calf_X_fr_RR_thigh) * RR_thigh_v;
    RR_calf_v(AZ) += qd(RR_CALF_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(RR_calf_v, vcross);
    RR_calf_c = vcross.col(AZ) * qd(RR_CALF_JOINT);
    
    //  - The bias force term:
    RR_calf_p += vxIv(RR_calf_v, RR_calf_AI);
    
    // + Link RL_hip
    //  - The spatial velocity:
    RL_hip_v = (motionTransforms-> fr_RL_hip_X_fr_base) * base_v;
    RL_hip_v(AZ) += qd(RL_HIP_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(RL_hip_v, vcross);
    RL_hip_c = vcross.col(AZ) * qd(RL_HIP_JOINT);
    
    //  - The bias force term:
    RL_hip_p += vxIv(RL_hip_v, RL_hip_AI);
    
    // + Link RL_thigh
    //  - The spatial velocity:
    RL_thigh_v = (motionTransforms-> fr_RL_thigh_X_fr_RL_hip) * RL_hip_v;
    RL_thigh_v(AZ) += qd(RL_THIGH_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(RL_thigh_v, vcross);
    RL_thigh_c = vcross.col(AZ) * qd(RL_THIGH_JOINT);
    
    //  - The bias force term:
    RL_thigh_p += vxIv(RL_thigh_v, RL_thigh_AI);
    
    // + Link RL_calf
    //  - The spatial velocity:
    RL_calf_v = (motionTransforms-> fr_RL_calf_X_fr_RL_thigh) * RL_thigh_v;
    RL_calf_v(AZ) += qd(RL_CALF_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(RL_calf_v, vcross);
    RL_calf_c = vcross.col(AZ) * qd(RL_CALF_JOINT);
    
    //  - The bias force term:
    RL_calf_p += vxIv(RL_calf_v, RL_calf_AI);
    
    // + The floating base body
    base_p += vxIv(base_v, base_AI);
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66 IaB;
    Force pa;
    
    // + Link RL_calf
    RL_calf_u = tau(RL_CALF_JOINT) - RL_calf_p(AZ);
    RL_calf_U = RL_calf_AI.col(AZ);
    RL_calf_D = RL_calf_U(AZ);
    
    compute_Ia_revolute(RL_calf_AI, RL_calf_U, RL_calf_D, Ia_r);  // same as: Ia_r = RL_calf_AI - RL_calf_U/RL_calf_D * RL_calf_U.transpose();
    pa = RL_calf_p + Ia_r * RL_calf_c + RL_calf_U * RL_calf_u/RL_calf_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RL_calf_X_fr_RL_thigh, IaB);
    RL_thigh_AI += IaB;
    RL_thigh_p += (motionTransforms-> fr_RL_calf_X_fr_RL_thigh).transpose() * pa;
    
    // + Link RL_thigh
    RL_thigh_u = tau(RL_THIGH_JOINT) - RL_thigh_p(AZ);
    RL_thigh_U = RL_thigh_AI.col(AZ);
    RL_thigh_D = RL_thigh_U(AZ);
    
    compute_Ia_revolute(RL_thigh_AI, RL_thigh_U, RL_thigh_D, Ia_r);  // same as: Ia_r = RL_thigh_AI - RL_thigh_U/RL_thigh_D * RL_thigh_U.transpose();
    pa = RL_thigh_p + Ia_r * RL_thigh_c + RL_thigh_U * RL_thigh_u/RL_thigh_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RL_thigh_X_fr_RL_hip, IaB);
    RL_hip_AI += IaB;
    RL_hip_p += (motionTransforms-> fr_RL_thigh_X_fr_RL_hip).transpose() * pa;
    
    // + Link RL_hip
    RL_hip_u = tau(RL_HIP_JOINT) - RL_hip_p(AZ);
    RL_hip_U = RL_hip_AI.col(AZ);
    RL_hip_D = RL_hip_U(AZ);
    
    compute_Ia_revolute(RL_hip_AI, RL_hip_U, RL_hip_D, Ia_r);  // same as: Ia_r = RL_hip_AI - RL_hip_U/RL_hip_D * RL_hip_U.transpose();
    pa = RL_hip_p + Ia_r * RL_hip_c + RL_hip_U * RL_hip_u/RL_hip_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RL_hip_X_fr_base, IaB);
    base_AI += IaB;
    base_p += (motionTransforms-> fr_RL_hip_X_fr_base).transpose() * pa;
    
    // + Link RR_calf
    RR_calf_u = tau(RR_CALF_JOINT) - RR_calf_p(AZ);
    RR_calf_U = RR_calf_AI.col(AZ);
    RR_calf_D = RR_calf_U(AZ);
    
    compute_Ia_revolute(RR_calf_AI, RR_calf_U, RR_calf_D, Ia_r);  // same as: Ia_r = RR_calf_AI - RR_calf_U/RR_calf_D * RR_calf_U.transpose();
    pa = RR_calf_p + Ia_r * RR_calf_c + RR_calf_U * RR_calf_u/RR_calf_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RR_calf_X_fr_RR_thigh, IaB);
    RR_thigh_AI += IaB;
    RR_thigh_p += (motionTransforms-> fr_RR_calf_X_fr_RR_thigh).transpose() * pa;
    
    // + Link RR_thigh
    RR_thigh_u = tau(RR_THIGH_JOINT) - RR_thigh_p(AZ);
    RR_thigh_U = RR_thigh_AI.col(AZ);
    RR_thigh_D = RR_thigh_U(AZ);
    
    compute_Ia_revolute(RR_thigh_AI, RR_thigh_U, RR_thigh_D, Ia_r);  // same as: Ia_r = RR_thigh_AI - RR_thigh_U/RR_thigh_D * RR_thigh_U.transpose();
    pa = RR_thigh_p + Ia_r * RR_thigh_c + RR_thigh_U * RR_thigh_u/RR_thigh_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RR_thigh_X_fr_RR_hip, IaB);
    RR_hip_AI += IaB;
    RR_hip_p += (motionTransforms-> fr_RR_thigh_X_fr_RR_hip).transpose() * pa;
    
    // + Link RR_hip
    RR_hip_u = tau(RR_HIP_JOINT) - RR_hip_p(AZ);
    RR_hip_U = RR_hip_AI.col(AZ);
    RR_hip_D = RR_hip_U(AZ);
    
    compute_Ia_revolute(RR_hip_AI, RR_hip_U, RR_hip_D, Ia_r);  // same as: Ia_r = RR_hip_AI - RR_hip_U/RR_hip_D * RR_hip_U.transpose();
    pa = RR_hip_p + Ia_r * RR_hip_c + RR_hip_U * RR_hip_u/RR_hip_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RR_hip_X_fr_base, IaB);
    base_AI += IaB;
    base_p += (motionTransforms-> fr_RR_hip_X_fr_base).transpose() * pa;
    
    // + Link FL_calf
    FL_calf_u = tau(FL_CALF_JOINT) - FL_calf_p(AZ);
    FL_calf_U = FL_calf_AI.col(AZ);
    FL_calf_D = FL_calf_U(AZ);
    
    compute_Ia_revolute(FL_calf_AI, FL_calf_U, FL_calf_D, Ia_r);  // same as: Ia_r = FL_calf_AI - FL_calf_U/FL_calf_D * FL_calf_U.transpose();
    pa = FL_calf_p + Ia_r * FL_calf_c + FL_calf_U * FL_calf_u/FL_calf_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_FL_calf_X_fr_FL_thigh, IaB);
    FL_thigh_AI += IaB;
    FL_thigh_p += (motionTransforms-> fr_FL_calf_X_fr_FL_thigh).transpose() * pa;
    
    // + Link FL_thigh
    FL_thigh_u = tau(FL_THIGH_JOINT) - FL_thigh_p(AZ);
    FL_thigh_U = FL_thigh_AI.col(AZ);
    FL_thigh_D = FL_thigh_U(AZ);
    
    compute_Ia_revolute(FL_thigh_AI, FL_thigh_U, FL_thigh_D, Ia_r);  // same as: Ia_r = FL_thigh_AI - FL_thigh_U/FL_thigh_D * FL_thigh_U.transpose();
    pa = FL_thigh_p + Ia_r * FL_thigh_c + FL_thigh_U * FL_thigh_u/FL_thigh_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_FL_thigh_X_fr_FL_hip, IaB);
    FL_hip_AI += IaB;
    FL_hip_p += (motionTransforms-> fr_FL_thigh_X_fr_FL_hip).transpose() * pa;
    
    // + Link FL_hip
    FL_hip_u = tau(FL_HIP_JOINT) - FL_hip_p(AZ);
    FL_hip_U = FL_hip_AI.col(AZ);
    FL_hip_D = FL_hip_U(AZ);
    
    compute_Ia_revolute(FL_hip_AI, FL_hip_U, FL_hip_D, Ia_r);  // same as: Ia_r = FL_hip_AI - FL_hip_U/FL_hip_D * FL_hip_U.transpose();
    pa = FL_hip_p + Ia_r * FL_hip_c + FL_hip_U * FL_hip_u/FL_hip_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_FL_hip_X_fr_base, IaB);
    base_AI += IaB;
    base_p += (motionTransforms-> fr_FL_hip_X_fr_base).transpose() * pa;
    
    // + Link FR_calf
    FR_calf_u = tau(FR_CALF_JOINT) - FR_calf_p(AZ);
    FR_calf_U = FR_calf_AI.col(AZ);
    FR_calf_D = FR_calf_U(AZ);
    
    compute_Ia_revolute(FR_calf_AI, FR_calf_U, FR_calf_D, Ia_r);  // same as: Ia_r = FR_calf_AI - FR_calf_U/FR_calf_D * FR_calf_U.transpose();
    pa = FR_calf_p + Ia_r * FR_calf_c + FR_calf_U * FR_calf_u/FR_calf_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_FR_calf_X_fr_FR_thigh, IaB);
    FR_thigh_AI += IaB;
    FR_thigh_p += (motionTransforms-> fr_FR_calf_X_fr_FR_thigh).transpose() * pa;
    
    // + Link FR_thigh
    FR_thigh_u = tau(FR_THIGH_JOINT) - FR_thigh_p(AZ);
    FR_thigh_U = FR_thigh_AI.col(AZ);
    FR_thigh_D = FR_thigh_U(AZ);
    
    compute_Ia_revolute(FR_thigh_AI, FR_thigh_U, FR_thigh_D, Ia_r);  // same as: Ia_r = FR_thigh_AI - FR_thigh_U/FR_thigh_D * FR_thigh_U.transpose();
    pa = FR_thigh_p + Ia_r * FR_thigh_c + FR_thigh_U * FR_thigh_u/FR_thigh_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_FR_thigh_X_fr_FR_hip, IaB);
    FR_hip_AI += IaB;
    FR_hip_p += (motionTransforms-> fr_FR_thigh_X_fr_FR_hip).transpose() * pa;
    
    // + Link FR_hip
    FR_hip_u = tau(FR_HIP_JOINT) - FR_hip_p(AZ);
    FR_hip_U = FR_hip_AI.col(AZ);
    FR_hip_D = FR_hip_U(AZ);
    
    compute_Ia_revolute(FR_hip_AI, FR_hip_U, FR_hip_D, Ia_r);  // same as: Ia_r = FR_hip_AI - FR_hip_U/FR_hip_D * FR_hip_U.transpose();
    pa = FR_hip_p + Ia_r * FR_hip_c + FR_hip_U * FR_hip_u/FR_hip_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_FR_hip_X_fr_base, IaB);
    base_AI += IaB;
    base_p += (motionTransforms-> fr_FR_hip_X_fr_base).transpose() * pa;
    
    // + The acceleration of the floating base base, without gravity
    base_a = - base_AI.llt().solve(base_p);  // base_a = - IA^-1 * base_p
    
    // ---------------------- THIRD PASS ---------------------- //
    FR_hip_a = (motionTransforms-> fr_FR_hip_X_fr_base) * base_a + FR_hip_c;
    qdd(FR_HIP_JOINT) = (FR_hip_u - FR_hip_U.dot(FR_hip_a)) / FR_hip_D;
    FR_hip_a(AZ) += qdd(FR_HIP_JOINT);
    
    FR_thigh_a = (motionTransforms-> fr_FR_thigh_X_fr_FR_hip) * FR_hip_a + FR_thigh_c;
    qdd(FR_THIGH_JOINT) = (FR_thigh_u - FR_thigh_U.dot(FR_thigh_a)) / FR_thigh_D;
    FR_thigh_a(AZ) += qdd(FR_THIGH_JOINT);
    
    FR_calf_a = (motionTransforms-> fr_FR_calf_X_fr_FR_thigh) * FR_thigh_a + FR_calf_c;
    qdd(FR_CALF_JOINT) = (FR_calf_u - FR_calf_U.dot(FR_calf_a)) / FR_calf_D;
    FR_calf_a(AZ) += qdd(FR_CALF_JOINT);
    
    FL_hip_a = (motionTransforms-> fr_FL_hip_X_fr_base) * base_a + FL_hip_c;
    qdd(FL_HIP_JOINT) = (FL_hip_u - FL_hip_U.dot(FL_hip_a)) / FL_hip_D;
    FL_hip_a(AZ) += qdd(FL_HIP_JOINT);
    
    FL_thigh_a = (motionTransforms-> fr_FL_thigh_X_fr_FL_hip) * FL_hip_a + FL_thigh_c;
    qdd(FL_THIGH_JOINT) = (FL_thigh_u - FL_thigh_U.dot(FL_thigh_a)) / FL_thigh_D;
    FL_thigh_a(AZ) += qdd(FL_THIGH_JOINT);
    
    FL_calf_a = (motionTransforms-> fr_FL_calf_X_fr_FL_thigh) * FL_thigh_a + FL_calf_c;
    qdd(FL_CALF_JOINT) = (FL_calf_u - FL_calf_U.dot(FL_calf_a)) / FL_calf_D;
    FL_calf_a(AZ) += qdd(FL_CALF_JOINT);
    
    RR_hip_a = (motionTransforms-> fr_RR_hip_X_fr_base) * base_a + RR_hip_c;
    qdd(RR_HIP_JOINT) = (RR_hip_u - RR_hip_U.dot(RR_hip_a)) / RR_hip_D;
    RR_hip_a(AZ) += qdd(RR_HIP_JOINT);
    
    RR_thigh_a = (motionTransforms-> fr_RR_thigh_X_fr_RR_hip) * RR_hip_a + RR_thigh_c;
    qdd(RR_THIGH_JOINT) = (RR_thigh_u - RR_thigh_U.dot(RR_thigh_a)) / RR_thigh_D;
    RR_thigh_a(AZ) += qdd(RR_THIGH_JOINT);
    
    RR_calf_a = (motionTransforms-> fr_RR_calf_X_fr_RR_thigh) * RR_thigh_a + RR_calf_c;
    qdd(RR_CALF_JOINT) = (RR_calf_u - RR_calf_U.dot(RR_calf_a)) / RR_calf_D;
    RR_calf_a(AZ) += qdd(RR_CALF_JOINT);
    
    RL_hip_a = (motionTransforms-> fr_RL_hip_X_fr_base) * base_a + RL_hip_c;
    qdd(RL_HIP_JOINT) = (RL_hip_u - RL_hip_U.dot(RL_hip_a)) / RL_hip_D;
    RL_hip_a(AZ) += qdd(RL_HIP_JOINT);
    
    RL_thigh_a = (motionTransforms-> fr_RL_thigh_X_fr_RL_hip) * RL_hip_a + RL_thigh_c;
    qdd(RL_THIGH_JOINT) = (RL_thigh_u - RL_thigh_U.dot(RL_thigh_a)) / RL_thigh_D;
    RL_thigh_a(AZ) += qdd(RL_THIGH_JOINT);
    
    RL_calf_a = (motionTransforms-> fr_RL_calf_X_fr_RL_thigh) * RL_thigh_a + RL_calf_c;
    qdd(RL_CALF_JOINT) = (RL_calf_u - RL_calf_U.dot(RL_calf_a)) / RL_calf_D;
    RL_calf_a(AZ) += qdd(RL_CALF_JOINT);
    
    
    // + Add gravity to the acceleration of the floating base
    base_a += g;
}
