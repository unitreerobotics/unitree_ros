cmake_minimum_required(VERSION 3.10.2)
project(g1_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes inspire_hand images
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  g1_23dof.urdf
  g1_23dof.xml
  g1_23dof_mode_10.urdf
  g1_23dof_rev_1_0.urdf
  g1_23dof_rev_1_0.xml
  g1_29dof.urdf
  g1_29dof.xml
  g1_29dof_lock_waist.urdf
  g1_29dof_lock_waist.xml
  g1_29dof_lock_waist_rev_1_0.urdf
  g1_29dof_lock_waist_rev_1_0.xml
  g1_29dof_lock_waist_with_hand_rev_1_0.urdf
  g1_29dof_lock_waist_with_hand_rev_1_0.xml
  g1_29dof_mode_11.urdf
  g1_29dof_mode_12.urdf
  g1_29dof_mode_13.urdf
  g1_29dof_mode_14.urdf
  g1_29dof_mode_15.urdf
  g1_29dof_mode_15_with_dex1_1.urdf
  g1_29dof_mode_16.urdf
  g1_29dof_rev_1_0.urdf
  g1_29dof_rev_1_0.xml
  g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf
  g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf
  g1_29dof_with_hand.urdf
  g1_29dof_with_hand.xml
  g1_29dof_with_hand_rev_1_0.urdf
  g1_29dof_with_hand_rev_1_0.xml
  g1_comp.urdf
  g1_dual_arm.urdf
  g1_dual_arm.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
