#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace UnitreeA1::rcg;

Vector3 UnitreeA1::rcg::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    Vector3 tmpSum(Vector3::Zero());

    tmpSum += inertiaProps.getCOM_base() * inertiaProps.getMass_base();

    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    HomogeneousTransforms::MatrixType base_X_FR_hip_joint_chain;
    HomogeneousTransforms::MatrixType base_X_FL_hip_joint_chain;
    HomogeneousTransforms::MatrixType base_X_RR_hip_joint_chain;
    HomogeneousTransforms::MatrixType base_X_RL_hip_joint_chain;
    
    
    base_X_FR_hip_joint_chain = tmpX * ht.fr_base_X_fr_FR_hip;
    tmpSum += inertiaProps.getMass_FR_hip() *
            ( iit::rbd::Utils::transform(base_X_FR_hip_joint_chain, inertiaProps.getCOM_FR_hip()));
    
    base_X_FR_hip_joint_chain = base_X_FR_hip_joint_chain * ht.fr_FR_hip_X_fr_FR_thigh;
    tmpSum += inertiaProps.getMass_FR_thigh() *
            ( iit::rbd::Utils::transform(base_X_FR_hip_joint_chain, inertiaProps.getCOM_FR_thigh()));
    
    base_X_FR_hip_joint_chain = base_X_FR_hip_joint_chain * ht.fr_FR_thigh_X_fr_FR_calf;
    tmpSum += inertiaProps.getMass_FR_calf() *
            ( iit::rbd::Utils::transform(base_X_FR_hip_joint_chain, inertiaProps.getCOM_FR_calf()));
    
    base_X_FL_hip_joint_chain = tmpX * ht.fr_base_X_fr_FL_hip;
    tmpSum += inertiaProps.getMass_FL_hip() *
            ( iit::rbd::Utils::transform(base_X_FL_hip_joint_chain, inertiaProps.getCOM_FL_hip()));
    
    base_X_FL_hip_joint_chain = base_X_FL_hip_joint_chain * ht.fr_FL_hip_X_fr_FL_thigh;
    tmpSum += inertiaProps.getMass_FL_thigh() *
            ( iit::rbd::Utils::transform(base_X_FL_hip_joint_chain, inertiaProps.getCOM_FL_thigh()));
    
    base_X_FL_hip_joint_chain = base_X_FL_hip_joint_chain * ht.fr_FL_thigh_X_fr_FL_calf;
    tmpSum += inertiaProps.getMass_FL_calf() *
            ( iit::rbd::Utils::transform(base_X_FL_hip_joint_chain, inertiaProps.getCOM_FL_calf()));
    
    base_X_RR_hip_joint_chain = tmpX * ht.fr_base_X_fr_RR_hip;
    tmpSum += inertiaProps.getMass_RR_hip() *
            ( iit::rbd::Utils::transform(base_X_RR_hip_joint_chain, inertiaProps.getCOM_RR_hip()));
    
    base_X_RR_hip_joint_chain = base_X_RR_hip_joint_chain * ht.fr_RR_hip_X_fr_RR_thigh;
    tmpSum += inertiaProps.getMass_RR_thigh() *
            ( iit::rbd::Utils::transform(base_X_RR_hip_joint_chain, inertiaProps.getCOM_RR_thigh()));
    
    base_X_RR_hip_joint_chain = base_X_RR_hip_joint_chain * ht.fr_RR_thigh_X_fr_RR_calf;
    tmpSum += inertiaProps.getMass_RR_calf() *
            ( iit::rbd::Utils::transform(base_X_RR_hip_joint_chain, inertiaProps.getCOM_RR_calf()));
    
    base_X_RL_hip_joint_chain = tmpX * ht.fr_base_X_fr_RL_hip;
    tmpSum += inertiaProps.getMass_RL_hip() *
            ( iit::rbd::Utils::transform(base_X_RL_hip_joint_chain, inertiaProps.getCOM_RL_hip()));
    
    base_X_RL_hip_joint_chain = base_X_RL_hip_joint_chain * ht.fr_RL_hip_X_fr_RL_thigh;
    tmpSum += inertiaProps.getMass_RL_thigh() *
            ( iit::rbd::Utils::transform(base_X_RL_hip_joint_chain, inertiaProps.getCOM_RL_thigh()));
    
    base_X_RL_hip_joint_chain = base_X_RL_hip_joint_chain * ht.fr_RL_thigh_X_fr_RL_calf;
    tmpSum += inertiaProps.getMass_RL_calf() *
            ( iit::rbd::Utils::transform(base_X_RL_hip_joint_chain, inertiaProps.getCOM_RL_calf()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

Vector3 UnitreeA1::rcg::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_base_X_fr_FR_hip(q);
    ht.fr_base_X_fr_FL_hip(q);
    ht.fr_base_X_fr_RR_hip(q);
    ht.fr_base_X_fr_RL_hip(q);
    ht.fr_FR_hip_X_fr_FR_thigh(q);
    ht.fr_FR_thigh_X_fr_FR_calf(q);
    ht.fr_FL_hip_X_fr_FL_thigh(q);
    ht.fr_FL_thigh_X_fr_FL_calf(q);
    ht.fr_RR_hip_X_fr_RR_thigh(q);
    ht.fr_RR_thigh_X_fr_RR_calf(q);
    ht.fr_RL_hip_X_fr_RL_thigh(q);
    ht.fr_RL_thigh_X_fr_RL_calf(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
