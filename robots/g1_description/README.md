# Unitree G1 Description (URDF & MJCF)

## Overview

This package includes a streamlined robot description (URDF & MJCF) for the [Unitree G1](https://www.unitree.com/g1/), developed by [Unitree Robotics](https://www.unitree.com/).

<p align="center">
  <img src="g1.png" width="500"/>
</p>

Current G1 have 37 DOFs:

```text
root [⚓] => /pelvis/
    left_hip_pitch_joint [⚙+Y] => /left_hip_pitch_link/
        left_hip_roll_joint [⚙+X] => /left_hip_roll_link/
            left_hip_yaw_joint [⚙+Z] => /left_hip_yaw_link/
                left_knee_joint [⚙+Y] => /left_knee_link/
                    left_ankle_pitch_joint [⚙+Y] => /left_ankle_pitch_link/
                        left_ankle_roll_joint [⚙+X] => /left_ankle_roll_link/
    right_hip_pitch_joint [⚙+Y] => /right_hip_pitch_link/
        right_hip_roll_joint [⚙+X] => /right_hip_roll_link/
            right_hip_yaw_joint [⚙+Z] => /right_hip_yaw_link/
                right_knee_joint [⚙+Y] => /right_knee_link/
                    right_ankle_pitch_joint [⚙+Y] => /right_ankle_pitch_link/
                        right_ankle_roll_joint [⚙+X] => /right_ankle_roll_link/
    torso_joint [⚙+Z] => /torso_link/
        left_shoulder_pitch_joint [⚙+Y] => /left_shoulder_pitch_link/
            left_shoulder_roll_joint [⚙+X] => /left_shoulder_roll_link/
                left_shoulder_yaw_joint [⚙+Z] => /left_shoulder_yaw_link/
                    left_elbow_pitch_joint [⚙+Y] => /left_elbow_pitch_link/
                        left_elbow_roll_joint [⚙+X] => /left_elbow_roll_link/
                            left_palm_joint [⚓] => /left_palm_link/
                                left_zero_joint [⚙+Y] => /left_zero_link/
                                    left_one_joint [⚙+Z] => /left_one_link/
                                        left_two_joint [⚙+Z] => /left_two_link/
                                left_three_joint [⚙+Z] => /left_three_link/
                                    left_four_joint [⚙+Z] => /left_four_link/
                                left_five_joint [⚙+Z] => /left_five_link/
                                    left_six_joint [⚙+Z] => /left_six_link/
        right_shoulder_pitch_joint [⚙+Y] => /right_shoulder_pitch_link/
            right_shoulder_roll_joint [⚙+X] => /right_shoulder_roll_link/
                right_shoulder_yaw_joint [⚙+Z] => /right_shoulder_yaw_link/
                    right_elbow_pitch_joint [⚙+Y] => /right_elbow_pitch_link/
                        right_elbow_roll_joint [⚙+X] => /right_elbow_roll_link/
                            right_palm_joint [⚓] => /right_palm_link/
                                right_zero_joint [⚙+Y] => /right_zero_link/
                                    right_one_joint [⚙+Z] => /right_one_link/
                                        right_two_joint [⚙+Z] => /right_two_link/
                                right_three_joint [⚙+Z] => /right_three_link/
                                    right_four_joint [⚙+Z] => /right_four_link/
                                right_five_joint [⚙+Z] => /right_five_link/
                                    right_six_joint [⚙+Z] => /right_six_link/
        imu_joint [⚓] => /imu_link/
```

## Visulization with [MuJoCo](https://github.com/google-deepmind/mujoco)

1. Open MuJoCo Viewer

   ```bash
   pip install mujoco
   python -m mujoco.viewer
   ```

2. Drag and drop the MJCF model file (`scene.xml`) to the MuJoCo Viewer.
