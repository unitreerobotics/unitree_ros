#include "inertia_properties.h"

using namespace std;
using namespace iit::rbd;

UnitreeA1::rcg::InertiaProperties::InertiaProperties()
{
    com_base = Vector3(0.0,comy_base,comz_base);
    tensor_base.fill(
        m_base,
        com_base,
        Utils::buildInertiaTensor<Scalar>(ix_base,iy_base,iz_base,ixy_base,ixz_base,iyz_base) );

    com_FR_hip = Vector3(comx_FR_hip,comy_FR_hip,comz_FR_hip);
    tensor_FR_hip.fill(
        m_FR_hip,
        com_FR_hip,
        Utils::buildInertiaTensor<Scalar>(ix_FR_hip,iy_FR_hip,iz_FR_hip,ixy_FR_hip,ixz_FR_hip,iyz_FR_hip) );

    com_FR_thigh = Vector3(comx_FR_thigh,comy_FR_thigh,comz_FR_thigh);
    tensor_FR_thigh.fill(
        m_FR_thigh,
        com_FR_thigh,
        Utils::buildInertiaTensor<Scalar>(ix_FR_thigh,iy_FR_thigh,iz_FR_thigh,ixy_FR_thigh,ixz_FR_thigh,iyz_FR_thigh) );

    com_FR_calf = Vector3(comx_FR_calf,comy_FR_calf,0.0);
    tensor_FR_calf.fill(
        m_FR_calf,
        com_FR_calf,
        Utils::buildInertiaTensor<Scalar>(ix_FR_calf,iy_FR_calf,iz_FR_calf,ixy_FR_calf,ixz_FR_calf,iyz_FR_calf) );

    com_FL_hip = Vector3(comx_FL_hip,comy_FL_hip,comz_FL_hip);
    tensor_FL_hip.fill(
        m_FL_hip,
        com_FL_hip,
        Utils::buildInertiaTensor<Scalar>(ix_FL_hip,iy_FL_hip,iz_FL_hip,ixy_FL_hip,ixz_FL_hip,iyz_FL_hip) );

    com_FL_thigh = Vector3(comx_FL_thigh,comy_FL_thigh,comz_FL_thigh);
    tensor_FL_thigh.fill(
        m_FL_thigh,
        com_FL_thigh,
        Utils::buildInertiaTensor<Scalar>(ix_FL_thigh,iy_FL_thigh,iz_FL_thigh,ixy_FL_thigh,ixz_FL_thigh,iyz_FL_thigh) );

    com_FL_calf = Vector3(comx_FL_calf,comy_FL_calf,0.0);
    tensor_FL_calf.fill(
        m_FL_calf,
        com_FL_calf,
        Utils::buildInertiaTensor<Scalar>(ix_FL_calf,iy_FL_calf,iz_FL_calf,ixy_FL_calf,ixz_FL_calf,iyz_FL_calf) );

    com_RR_hip = Vector3(comx_RR_hip,comy_RR_hip,comz_RR_hip);
    tensor_RR_hip.fill(
        m_RR_hip,
        com_RR_hip,
        Utils::buildInertiaTensor<Scalar>(ix_RR_hip,iy_RR_hip,iz_RR_hip,ixy_RR_hip,ixz_RR_hip,iyz_RR_hip) );

    com_RR_thigh = Vector3(comx_RR_thigh,comy_RR_thigh,comz_RR_thigh);
    tensor_RR_thigh.fill(
        m_RR_thigh,
        com_RR_thigh,
        Utils::buildInertiaTensor<Scalar>(ix_RR_thigh,iy_RR_thigh,iz_RR_thigh,ixy_RR_thigh,ixz_RR_thigh,iyz_RR_thigh) );

    com_RR_calf = Vector3(comx_RR_calf,comy_RR_calf,0.0);
    tensor_RR_calf.fill(
        m_RR_calf,
        com_RR_calf,
        Utils::buildInertiaTensor<Scalar>(ix_RR_calf,iy_RR_calf,iz_RR_calf,ixy_RR_calf,ixz_RR_calf,iyz_RR_calf) );

    com_RL_hip = Vector3(comx_RL_hip,comy_RL_hip,comz_RL_hip);
    tensor_RL_hip.fill(
        m_RL_hip,
        com_RL_hip,
        Utils::buildInertiaTensor<Scalar>(ix_RL_hip,iy_RL_hip,iz_RL_hip,ixy_RL_hip,ixz_RL_hip,iyz_RL_hip) );

    com_RL_thigh = Vector3(comx_RL_thigh,comy_RL_thigh,comz_RL_thigh);
    tensor_RL_thigh.fill(
        m_RL_thigh,
        com_RL_thigh,
        Utils::buildInertiaTensor<Scalar>(ix_RL_thigh,iy_RL_thigh,iz_RL_thigh,ixy_RL_thigh,ixz_RL_thigh,iyz_RL_thigh) );

    com_RL_calf = Vector3(comx_RL_calf,comy_RL_calf,0.0);
    tensor_RL_calf.fill(
        m_RL_calf,
        com_RL_calf,
        Utils::buildInertiaTensor<Scalar>(ix_RL_calf,iy_RL_calf,iz_RL_calf,ixy_RL_calf,ixz_RL_calf,iyz_RL_calf) );

}


void UnitreeA1::rcg::InertiaProperties::updateParameters(const RuntimeInertiaParams& fresh)
{
    this-> params = fresh;
}
