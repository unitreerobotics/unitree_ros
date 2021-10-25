#include "transforms.h"
#include "jsim.h"

#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

//Implementation of default constructor
UnitreeA1::rcg::JSIM::JSIM(InertiaProperties& inertiaProperties, ForceTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    FR_calf_Ic(linkInertias.getTensor_FR_calf()),
    FL_calf_Ic(linkInertias.getTensor_FL_calf()),
    RR_calf_Ic(linkInertias.getTensor_RR_calf()),
    RL_calf_Ic(linkInertias.getTensor_RL_calf())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA operator()
#define Fcol(j) (block<6,1>(0,(j)+6))
#define F(i,j) DATA((i),(j)+6)
const UnitreeA1::rcg::JSIM& UnitreeA1::rcg::JSIM::update(const JointState& state) {

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_RL_thigh_X_fr_RL_calf(state);
    frcTransf -> fr_RL_hip_X_fr_RL_thigh(state);
    frcTransf -> fr_base_X_fr_RL_hip(state);
    frcTransf -> fr_RR_thigh_X_fr_RR_calf(state);
    frcTransf -> fr_RR_hip_X_fr_RR_thigh(state);
    frcTransf -> fr_base_X_fr_RR_hip(state);
    frcTransf -> fr_FL_thigh_X_fr_FL_calf(state);
    frcTransf -> fr_FL_hip_X_fr_FL_thigh(state);
    frcTransf -> fr_base_X_fr_FL_hip(state);
    frcTransf -> fr_FR_thigh_X_fr_FR_calf(state);
    frcTransf -> fr_FR_hip_X_fr_FR_thigh(state);
    frcTransf -> fr_base_X_fr_FR_hip(state);

    // Initializes the composite inertia tensors
    base_Ic = linkInertias.getTensor_base();
    FR_hip_Ic = linkInertias.getTensor_FR_hip();
    FR_thigh_Ic = linkInertias.getTensor_FR_thigh();
    FL_hip_Ic = linkInertias.getTensor_FL_hip();
    FL_thigh_Ic = linkInertias.getTensor_FL_thigh();
    RR_hip_Ic = linkInertias.getTensor_RR_hip();
    RR_thigh_Ic = linkInertias.getTensor_RR_thigh();
    RL_hip_Ic = linkInertias.getTensor_RL_hip();
    RL_thigh_Ic = linkInertias.getTensor_RL_thigh();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link RL_calf:
    iit::rbd::transformInertia<Scalar>(RL_calf_Ic, frcTransf -> fr_RL_thigh_X_fr_RL_calf, Ic_spare);
    RL_thigh_Ic += Ic_spare;

    Fcol(RL_CALF_JOINT) = RL_calf_Ic.col(AZ);
    DATA(RL_CALF_JOINT+6, RL_CALF_JOINT+6) = Fcol(RL_CALF_JOINT)(AZ);

    Fcol(RL_CALF_JOINT) = frcTransf -> fr_RL_thigh_X_fr_RL_calf * Fcol(RL_CALF_JOINT);
    DATA(RL_CALF_JOINT+6, RL_THIGH_JOINT+6) = F(AZ,RL_CALF_JOINT);
    DATA(RL_THIGH_JOINT+6, RL_CALF_JOINT+6) = DATA(RL_CALF_JOINT+6, RL_THIGH_JOINT+6);
    Fcol(RL_CALF_JOINT) = frcTransf -> fr_RL_hip_X_fr_RL_thigh * Fcol(RL_CALF_JOINT);
    DATA(RL_CALF_JOINT+6, RL_HIP_JOINT+6) = F(AZ,RL_CALF_JOINT);
    DATA(RL_HIP_JOINT+6, RL_CALF_JOINT+6) = DATA(RL_CALF_JOINT+6, RL_HIP_JOINT+6);
    Fcol(RL_CALF_JOINT) = frcTransf -> fr_base_X_fr_RL_hip * Fcol(RL_CALF_JOINT);

    // Link RL_thigh:
    iit::rbd::transformInertia<Scalar>(RL_thigh_Ic, frcTransf -> fr_RL_hip_X_fr_RL_thigh, Ic_spare);
    RL_hip_Ic += Ic_spare;

    Fcol(RL_THIGH_JOINT) = RL_thigh_Ic.col(AZ);
    DATA(RL_THIGH_JOINT+6, RL_THIGH_JOINT+6) = Fcol(RL_THIGH_JOINT)(AZ);

    Fcol(RL_THIGH_JOINT) = frcTransf -> fr_RL_hip_X_fr_RL_thigh * Fcol(RL_THIGH_JOINT);
    DATA(RL_THIGH_JOINT+6, RL_HIP_JOINT+6) = F(AZ,RL_THIGH_JOINT);
    DATA(RL_HIP_JOINT+6, RL_THIGH_JOINT+6) = DATA(RL_THIGH_JOINT+6, RL_HIP_JOINT+6);
    Fcol(RL_THIGH_JOINT) = frcTransf -> fr_base_X_fr_RL_hip * Fcol(RL_THIGH_JOINT);

    // Link RL_hip:
    iit::rbd::transformInertia<Scalar>(RL_hip_Ic, frcTransf -> fr_base_X_fr_RL_hip, Ic_spare);
    base_Ic += Ic_spare;

    Fcol(RL_HIP_JOINT) = RL_hip_Ic.col(AZ);
    DATA(RL_HIP_JOINT+6, RL_HIP_JOINT+6) = Fcol(RL_HIP_JOINT)(AZ);

    Fcol(RL_HIP_JOINT) = frcTransf -> fr_base_X_fr_RL_hip * Fcol(RL_HIP_JOINT);

    // Link RR_calf:
    iit::rbd::transformInertia<Scalar>(RR_calf_Ic, frcTransf -> fr_RR_thigh_X_fr_RR_calf, Ic_spare);
    RR_thigh_Ic += Ic_spare;

    Fcol(RR_CALF_JOINT) = RR_calf_Ic.col(AZ);
    DATA(RR_CALF_JOINT+6, RR_CALF_JOINT+6) = Fcol(RR_CALF_JOINT)(AZ);

    Fcol(RR_CALF_JOINT) = frcTransf -> fr_RR_thigh_X_fr_RR_calf * Fcol(RR_CALF_JOINT);
    DATA(RR_CALF_JOINT+6, RR_THIGH_JOINT+6) = F(AZ,RR_CALF_JOINT);
    DATA(RR_THIGH_JOINT+6, RR_CALF_JOINT+6) = DATA(RR_CALF_JOINT+6, RR_THIGH_JOINT+6);
    Fcol(RR_CALF_JOINT) = frcTransf -> fr_RR_hip_X_fr_RR_thigh * Fcol(RR_CALF_JOINT);
    DATA(RR_CALF_JOINT+6, RR_HIP_JOINT+6) = F(AZ,RR_CALF_JOINT);
    DATA(RR_HIP_JOINT+6, RR_CALF_JOINT+6) = DATA(RR_CALF_JOINT+6, RR_HIP_JOINT+6);
    Fcol(RR_CALF_JOINT) = frcTransf -> fr_base_X_fr_RR_hip * Fcol(RR_CALF_JOINT);

    // Link RR_thigh:
    iit::rbd::transformInertia<Scalar>(RR_thigh_Ic, frcTransf -> fr_RR_hip_X_fr_RR_thigh, Ic_spare);
    RR_hip_Ic += Ic_spare;

    Fcol(RR_THIGH_JOINT) = RR_thigh_Ic.col(AZ);
    DATA(RR_THIGH_JOINT+6, RR_THIGH_JOINT+6) = Fcol(RR_THIGH_JOINT)(AZ);

    Fcol(RR_THIGH_JOINT) = frcTransf -> fr_RR_hip_X_fr_RR_thigh * Fcol(RR_THIGH_JOINT);
    DATA(RR_THIGH_JOINT+6, RR_HIP_JOINT+6) = F(AZ,RR_THIGH_JOINT);
    DATA(RR_HIP_JOINT+6, RR_THIGH_JOINT+6) = DATA(RR_THIGH_JOINT+6, RR_HIP_JOINT+6);
    Fcol(RR_THIGH_JOINT) = frcTransf -> fr_base_X_fr_RR_hip * Fcol(RR_THIGH_JOINT);

    // Link RR_hip:
    iit::rbd::transformInertia<Scalar>(RR_hip_Ic, frcTransf -> fr_base_X_fr_RR_hip, Ic_spare);
    base_Ic += Ic_spare;

    Fcol(RR_HIP_JOINT) = RR_hip_Ic.col(AZ);
    DATA(RR_HIP_JOINT+6, RR_HIP_JOINT+6) = Fcol(RR_HIP_JOINT)(AZ);

    Fcol(RR_HIP_JOINT) = frcTransf -> fr_base_X_fr_RR_hip * Fcol(RR_HIP_JOINT);

    // Link FL_calf:
    iit::rbd::transformInertia<Scalar>(FL_calf_Ic, frcTransf -> fr_FL_thigh_X_fr_FL_calf, Ic_spare);
    FL_thigh_Ic += Ic_spare;

    Fcol(FL_CALF_JOINT) = FL_calf_Ic.col(AZ);
    DATA(FL_CALF_JOINT+6, FL_CALF_JOINT+6) = Fcol(FL_CALF_JOINT)(AZ);

    Fcol(FL_CALF_JOINT) = frcTransf -> fr_FL_thigh_X_fr_FL_calf * Fcol(FL_CALF_JOINT);
    DATA(FL_CALF_JOINT+6, FL_THIGH_JOINT+6) = F(AZ,FL_CALF_JOINT);
    DATA(FL_THIGH_JOINT+6, FL_CALF_JOINT+6) = DATA(FL_CALF_JOINT+6, FL_THIGH_JOINT+6);
    Fcol(FL_CALF_JOINT) = frcTransf -> fr_FL_hip_X_fr_FL_thigh * Fcol(FL_CALF_JOINT);
    DATA(FL_CALF_JOINT+6, FL_HIP_JOINT+6) = F(AZ,FL_CALF_JOINT);
    DATA(FL_HIP_JOINT+6, FL_CALF_JOINT+6) = DATA(FL_CALF_JOINT+6, FL_HIP_JOINT+6);
    Fcol(FL_CALF_JOINT) = frcTransf -> fr_base_X_fr_FL_hip * Fcol(FL_CALF_JOINT);

    // Link FL_thigh:
    iit::rbd::transformInertia<Scalar>(FL_thigh_Ic, frcTransf -> fr_FL_hip_X_fr_FL_thigh, Ic_spare);
    FL_hip_Ic += Ic_spare;

    Fcol(FL_THIGH_JOINT) = FL_thigh_Ic.col(AZ);
    DATA(FL_THIGH_JOINT+6, FL_THIGH_JOINT+6) = Fcol(FL_THIGH_JOINT)(AZ);

    Fcol(FL_THIGH_JOINT) = frcTransf -> fr_FL_hip_X_fr_FL_thigh * Fcol(FL_THIGH_JOINT);
    DATA(FL_THIGH_JOINT+6, FL_HIP_JOINT+6) = F(AZ,FL_THIGH_JOINT);
    DATA(FL_HIP_JOINT+6, FL_THIGH_JOINT+6) = DATA(FL_THIGH_JOINT+6, FL_HIP_JOINT+6);
    Fcol(FL_THIGH_JOINT) = frcTransf -> fr_base_X_fr_FL_hip * Fcol(FL_THIGH_JOINT);

    // Link FL_hip:
    iit::rbd::transformInertia<Scalar>(FL_hip_Ic, frcTransf -> fr_base_X_fr_FL_hip, Ic_spare);
    base_Ic += Ic_spare;

    Fcol(FL_HIP_JOINT) = FL_hip_Ic.col(AZ);
    DATA(FL_HIP_JOINT+6, FL_HIP_JOINT+6) = Fcol(FL_HIP_JOINT)(AZ);

    Fcol(FL_HIP_JOINT) = frcTransf -> fr_base_X_fr_FL_hip * Fcol(FL_HIP_JOINT);

    // Link FR_calf:
    iit::rbd::transformInertia<Scalar>(FR_calf_Ic, frcTransf -> fr_FR_thigh_X_fr_FR_calf, Ic_spare);
    FR_thigh_Ic += Ic_spare;

    Fcol(FR_CALF_JOINT) = FR_calf_Ic.col(AZ);
    DATA(FR_CALF_JOINT+6, FR_CALF_JOINT+6) = Fcol(FR_CALF_JOINT)(AZ);

    Fcol(FR_CALF_JOINT) = frcTransf -> fr_FR_thigh_X_fr_FR_calf * Fcol(FR_CALF_JOINT);
    DATA(FR_CALF_JOINT+6, FR_THIGH_JOINT+6) = F(AZ,FR_CALF_JOINT);
    DATA(FR_THIGH_JOINT+6, FR_CALF_JOINT+6) = DATA(FR_CALF_JOINT+6, FR_THIGH_JOINT+6);
    Fcol(FR_CALF_JOINT) = frcTransf -> fr_FR_hip_X_fr_FR_thigh * Fcol(FR_CALF_JOINT);
    DATA(FR_CALF_JOINT+6, FR_HIP_JOINT+6) = F(AZ,FR_CALF_JOINT);
    DATA(FR_HIP_JOINT+6, FR_CALF_JOINT+6) = DATA(FR_CALF_JOINT+6, FR_HIP_JOINT+6);
    Fcol(FR_CALF_JOINT) = frcTransf -> fr_base_X_fr_FR_hip * Fcol(FR_CALF_JOINT);

    // Link FR_thigh:
    iit::rbd::transformInertia<Scalar>(FR_thigh_Ic, frcTransf -> fr_FR_hip_X_fr_FR_thigh, Ic_spare);
    FR_hip_Ic += Ic_spare;

    Fcol(FR_THIGH_JOINT) = FR_thigh_Ic.col(AZ);
    DATA(FR_THIGH_JOINT+6, FR_THIGH_JOINT+6) = Fcol(FR_THIGH_JOINT)(AZ);

    Fcol(FR_THIGH_JOINT) = frcTransf -> fr_FR_hip_X_fr_FR_thigh * Fcol(FR_THIGH_JOINT);
    DATA(FR_THIGH_JOINT+6, FR_HIP_JOINT+6) = F(AZ,FR_THIGH_JOINT);
    DATA(FR_HIP_JOINT+6, FR_THIGH_JOINT+6) = DATA(FR_THIGH_JOINT+6, FR_HIP_JOINT+6);
    Fcol(FR_THIGH_JOINT) = frcTransf -> fr_base_X_fr_FR_hip * Fcol(FR_THIGH_JOINT);

    // Link FR_hip:
    iit::rbd::transformInertia<Scalar>(FR_hip_Ic, frcTransf -> fr_base_X_fr_FR_hip, Ic_spare);
    base_Ic += Ic_spare;

    Fcol(FR_HIP_JOINT) = FR_hip_Ic.col(AZ);
    DATA(FR_HIP_JOINT+6, FR_HIP_JOINT+6) = Fcol(FR_HIP_JOINT)(AZ);

    Fcol(FR_HIP_JOINT) = frcTransf -> fr_base_X_fr_FR_hip * Fcol(FR_HIP_JOINT);

    // Copies the upper-right block into the lower-left block, after transposing
    block<12, 6>(6,0) = (block<6, 12>(0,6)).transpose();
    // The composite-inertia of the whole robot is the upper-left quadrant of the JSIM
    block<6,6>(0,0) = base_Ic;
    return *this;
}

#undef DATA
#undef F

void UnitreeA1::rcg::JSIM::computeL() {
    L = this -> triangularView<Eigen::Lower>();
    // Joint RL_calf_joint, index 11 :
    L(11, 11) = ScalarTraits::sqrt(L(11, 11));
    L(11, 10) = L(11, 10) / L(11, 11);
    L(11, 9) = L(11, 9) / L(11, 11);
    L(10, 10) = L(10, 10) - L(11, 10) * L(11, 10);
    L(10, 9) = L(10, 9) - L(11, 10) * L(11, 9);
    L(9, 9) = L(9, 9) - L(11, 9) * L(11, 9);
    
    // Joint RL_thigh_joint, index 10 :
    L(10, 10) = ScalarTraits::sqrt(L(10, 10));
    L(10, 9) = L(10, 9) / L(10, 10);
    L(9, 9) = L(9, 9) - L(10, 9) * L(10, 9);
    
    // Joint RL_hip_joint, index 9 :
    L(9, 9) = ScalarTraits::sqrt(L(9, 9));
    
    // Joint RR_calf_joint, index 8 :
    L(8, 8) = ScalarTraits::sqrt(L(8, 8));
    L(8, 7) = L(8, 7) / L(8, 8);
    L(8, 6) = L(8, 6) / L(8, 8);
    L(7, 7) = L(7, 7) - L(8, 7) * L(8, 7);
    L(7, 6) = L(7, 6) - L(8, 7) * L(8, 6);
    L(6, 6) = L(6, 6) - L(8, 6) * L(8, 6);
    
    // Joint RR_thigh_joint, index 7 :
    L(7, 7) = ScalarTraits::sqrt(L(7, 7));
    L(7, 6) = L(7, 6) / L(7, 7);
    L(6, 6) = L(6, 6) - L(7, 6) * L(7, 6);
    
    // Joint RR_hip_joint, index 6 :
    L(6, 6) = ScalarTraits::sqrt(L(6, 6));
    
    // Joint FL_calf_joint, index 5 :
    L(5, 5) = ScalarTraits::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(5, 3) = L(5, 3) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    L(4, 3) = L(4, 3) - L(5, 4) * L(5, 3);
    L(3, 3) = L(3, 3) - L(5, 3) * L(5, 3);
    
    // Joint FL_thigh_joint, index 4 :
    L(4, 4) = ScalarTraits::sqrt(L(4, 4));
    L(4, 3) = L(4, 3) / L(4, 4);
    L(3, 3) = L(3, 3) - L(4, 3) * L(4, 3);
    
    // Joint FL_hip_joint, index 3 :
    L(3, 3) = ScalarTraits::sqrt(L(3, 3));
    
    // Joint FR_calf_joint, index 2 :
    L(2, 2) = ScalarTraits::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(2, 0) = L(2, 0) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    L(1, 0) = L(1, 0) - L(2, 1) * L(2, 0);
    L(0, 0) = L(0, 0) - L(2, 0) * L(2, 0);
    
    // Joint FR_thigh_joint, index 1 :
    L(1, 1) = ScalarTraits::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
    
    // Joint FR_hip_joint, index 0 :
    L(0, 0) = ScalarTraits::sqrt(L(0, 0));
    
}

void UnitreeA1::rcg::JSIM::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
    inverse(1, 0) =  + (Linv(1, 0) * Linv(0, 0));
    inverse(0, 1) = inverse(1, 0);
    inverse(2, 2) =  + (Linv(2, 0) * Linv(2, 0)) + (Linv(2, 1) * Linv(2, 1)) + (Linv(2, 2) * Linv(2, 2));
    inverse(2, 1) =  + (Linv(2, 0) * Linv(1, 0)) + (Linv(2, 1) * Linv(1, 1));
    inverse(1, 2) = inverse(2, 1);
    inverse(2, 0) =  + (Linv(2, 0) * Linv(0, 0));
    inverse(0, 2) = inverse(2, 0);
    inverse(3, 3) =  + (Linv(3, 3) * Linv(3, 3));
    inverse(4, 4) =  + (Linv(4, 3) * Linv(4, 3)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 3) =  + (Linv(4, 3) * Linv(3, 3));
    inverse(3, 4) = inverse(4, 3);
    inverse(5, 5) =  + (Linv(5, 3) * Linv(5, 3)) + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) =  + (Linv(5, 3) * Linv(4, 3)) + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(5, 3) =  + (Linv(5, 3) * Linv(3, 3));
    inverse(3, 5) = inverse(5, 3);
    inverse(6, 6) =  + (Linv(6, 6) * Linv(6, 6));
    inverse(7, 7) =  + (Linv(7, 6) * Linv(7, 6)) + (Linv(7, 7) * Linv(7, 7));
    inverse(7, 6) =  + (Linv(7, 6) * Linv(6, 6));
    inverse(6, 7) = inverse(7, 6);
    inverse(8, 8) =  + (Linv(8, 6) * Linv(8, 6)) + (Linv(8, 7) * Linv(8, 7)) + (Linv(8, 8) * Linv(8, 8));
    inverse(8, 7) =  + (Linv(8, 6) * Linv(7, 6)) + (Linv(8, 7) * Linv(7, 7));
    inverse(7, 8) = inverse(8, 7);
    inverse(8, 6) =  + (Linv(8, 6) * Linv(6, 6));
    inverse(6, 8) = inverse(8, 6);
    inverse(9, 9) =  + (Linv(9, 9) * Linv(9, 9));
    inverse(10, 10) =  + (Linv(10, 9) * Linv(10, 9)) + (Linv(10, 10) * Linv(10, 10));
    inverse(10, 9) =  + (Linv(10, 9) * Linv(9, 9));
    inverse(9, 10) = inverse(10, 9);
    inverse(11, 11) =  + (Linv(11, 9) * Linv(11, 9)) + (Linv(11, 10) * Linv(11, 10)) + (Linv(11, 11) * Linv(11, 11));
    inverse(11, 10) =  + (Linv(11, 9) * Linv(10, 9)) + (Linv(11, 10) * Linv(10, 10));
    inverse(10, 11) = inverse(11, 10);
    inverse(11, 9) =  + (Linv(11, 9) * Linv(9, 9));
    inverse(9, 11) = inverse(11, 9);
}

void UnitreeA1::rcg::JSIM::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(6, 6) = 1 / L(6, 6);
    Linv(7, 7) = 1 / L(7, 7);
    Linv(8, 8) = 1 / L(8, 8);
    Linv(9, 9) = 1 / L(9, 9);
    Linv(10, 10) = 1 / L(10, 10);
    Linv(11, 11) = 1 / L(11, 11);
    Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
    Linv(2, 1) = - Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(2, 0) = - Linv(0, 0) * ((Linv(2, 1) * L(1, 0)) + (Linv(2, 2) * L(2, 0)) + 0);
    Linv(4, 3) = - Linv(3, 3) * ((Linv(4, 4) * L(4, 3)) + 0);
    Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(5, 3) = - Linv(3, 3) * ((Linv(5, 4) * L(4, 3)) + (Linv(5, 5) * L(5, 3)) + 0);
    Linv(7, 6) = - Linv(6, 6) * ((Linv(7, 7) * L(7, 6)) + 0);
    Linv(8, 7) = - Linv(7, 7) * ((Linv(8, 8) * L(8, 7)) + 0);
    Linv(8, 6) = - Linv(6, 6) * ((Linv(8, 7) * L(7, 6)) + (Linv(8, 8) * L(8, 6)) + 0);
    Linv(10, 9) = - Linv(9, 9) * ((Linv(10, 10) * L(10, 9)) + 0);
    Linv(11, 10) = - Linv(10, 10) * ((Linv(11, 11) * L(11, 10)) + 0);
    Linv(11, 9) = - Linv(9, 9) * ((Linv(11, 10) * L(10, 9)) + (Linv(11, 11) * L(11, 9)) + 0);
}
