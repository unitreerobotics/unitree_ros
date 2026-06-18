# Unitree G1 Description (URDF & MJCF)

## Overview

This package includes a universal humanoid robot description (URDF & MJCF) for the [Unitree G1](https://www.unitree.com/g1/), developed by [Unitree Robotics](https://www.unitree.com/).

MJCF/URDF for the G1 robot:

| MJCF/URDF file name                      | `mode_machine` | hip.{pitch, roll} gear ratio | wrist motor | lock waist | Update status | dof#leg | dof#waist | dof#arm | dof#hand |
|------------------------------------------|:--------------:|------------------------------|-------------|------------|---------------|:-------:|:---------:|:-------:|:--------:|
| `g1_23dof_mode_10`                       |       10       | {22.5, 22.5}                 | null        | no         | Up-to-date    |   6*2   |     1     |   5*2   |   null   |
| `g1_29dof_mode_11`                       |       11       | {22.5, 22.5}                 | 4010        | no         | Up-to-date    |   6*2   |     3     |   7*2   |   null   |
| `g1_29dof_mode_12`                       |       12       | {22.5, 22.5}                 | 4010        | yes        | Up-to-date    |   6*2   |     3     |   7*2   |   null   |
| `g1_29dof_mode_13`                       |       13       | {14.3, 22.5}                 | 5010(new)   | no         | Up-to-date    |   6*2   |     3     |   7*2   |   null   |
| `g1_29dof_mode_14`                       |       14       | {14.3, 22.5}                 | 5010(new)   | yes        | Up-to-date    |   6*2   |     3     |   7*2   |   null   |
| `g1_29dof_mode_15`                       |       15       | {22.5, 22.5}                 | 5010(new)   | no         | Up-to-date    |   6*2   |     3     |   7*2   |   null   |
| `g1_29dof_mode_16`                       |       16       | {22.5, 22.5}                 | 5010(new)   | yes        | Up-to-date    |   6*2   |     3     |   7*2   |   null   |
| `g1_23dof_rev_1_0`                       |       4        | {14.3, 22.5}                 | null        |            | Up-to-date    |   6*2   |     1     |   5*2   |   null   |
| `g1_29dof_rev_1_0`                       |       5        | {14.3, 22.5}                 | 4010        |            | Up-to-date    |   6*2   |     3     |   7*2   |   null   |
| `g1_29dof_with_hand_rev_1_0`             |       5        | {14.3, 22.5}                 | 4010        |            | Up-to-date    |   6*2   |     3     |   7*2   |   7*2    |
| `g1_29dof_rev_1_0_with_inspire_hand_DFQ` |       5        | {14.3, 22.5}                 | 4010        |            | Up-to-date    |   6*2   |     3     |   7*2   |   12*2   |
| `g1_29dof_rev_1_0_with_inspire_hand_FTP` |       5        | {14.3, 22.5}                 | 4010        |            | Up-to-date    |   6*2   |     3     |   7*2   |   12*2   |
| `g1_29dof_lock_waist_rev_1_0`            |       6        | {14.3, 22.5}                 | 4010        |            | Up-to-date    |   6*2   |     3     |   7*2   |   null   |
| `g1_29dof_lock_waist_with_hand_rev_1_0`  |       6        | {14.3, 22.5}                 | 4010        |            | Up-to-date    |   6*2   |     3     |   7*2   |   7*2    |
| `g1_dual_arm`                            |       9        | null                         | 4010        |            | Up-to-date    |  null   |   null    |   7*2   |   null   |
| ~~`g1_23dof`~~                           |       1        | {14.3, 14.5}                 | null        |            | Deprecated    |   6*2   |     1     |   5*2   |   null   |
| ~~`g1_29dof`~~                           |       2        | {14.3, 14.5}                 | 4010        |            | Deprecated    |   6*2   |     3     |   7*2   |   null   |
| ~~`g1_29dof_with_hand`~~                 |       2        | {14.3, 14.5}                 | 4010        |            | Deprecated    |   6*2   |     3     |   7*2   |   7*2    |
| ~~`g1_29dof_lock_waist`~~                |       3        | {14.3, 14.5}                 | 4010        |            | Deprecated    |   6*2   |     3     |   7*2   |   null   |

💡**Note:** The robot's `mode_machine` ID can be viewed in the app by navigating to **Device** → **Data** → **Robot** → **Machine Type**.

## Visulization with [MuJoCo](https://github.com/google-deepmind/mujoco)

1. Open MuJoCo Viewer

   ```bash
   pip install mujoco
   python -m mujoco.viewer
   ```

2. Drag and drop the MJCF/URDF model file (`g1_XXX.xml`/`g1_XXX.urdf`) to the MuJoCo Viewer.

## Visulization with VSCode + [Urdf Visualizer](https://github.com/MorningFrog/urdf-visualizer)

Install VSCode and [Urdf Visualizer](https://github.com/MorningFrog/urdf-visualizer)

## Q&A: How to Add AGX Backpack?

Add the following code snippet to the URDF file.

```xml
<!-- ADD AGX Backpack -->
<link name="backpack_link">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.9"/>
    <inertia ixx="0.015510" ixy="0.000017" ixz="0.000563" iyy="0.010909" iyz="0.000059" izz="0.007288"/>
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="meshes/backpack_link.STL"/>
    </geometry>
    <material name="white"/>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="meshes/backpack_link.STL"/>
    </geometry>
  </collision>
</link>
<joint name="backpack_joint" type="fixed">
  <origin xyz="-0.094605 -0.0005 0.165324" rpy="0 0 0"/>
  <parent link="torso_link"/>
  <child link="backpack_link"/>
</joint>
```