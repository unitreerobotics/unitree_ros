# Unitree G1 Description (URDF & MJCF)

## Overview

This package includes a universal humanoid robot description (URDF & MJCF) for the [Unitree G1](https://www.unitree.com/g1/), developed by [Unitree Robotics](https://www.unitree.com/).

MJCF/URDF for the G1 robot:

| MJCF/URDF file name                             | `mode_machine` |
| ----------------------------------------------- | :------------: |
| `g1_23dof`                                      |       1        |
| `g1_29dof`/`g1_29dof_with_hand`                 |       2        |
| `g1_29dof_lock_waist`                           |       3        |
| `g1_23dof_rev_1_0`                              |       4        |
| `g1_29dof_rev_1_0`/`g1_29dof_with_hand_rev_1_0` |       5        |
| `g1_29dof_lock_waist_rev_1_0`                   |       6        |
| `g1_dual_arm`                                   |       9        |

## Visulization with [MuJoCo](https://github.com/google-deepmind/mujoco)

1. Open MuJoCo Viewer

   ```bash
   pip install mujoco
   python -m mujoco.viewer
   ```

2. Drag and drop the MJCF/URDF model file (`g1_XXX.xml`/`g1_XXX.urdf`) to the MuJoCo Viewer.
