# 电机顺序

## G1 全身关节电机顺序

`unitree_hg::msg::dds_::LowCmd_.motor_cmd` 与 `unitree_hg::msg::dds_::LowState_.motor_state` 包含 G1 全身电机（不含手）的信息，其电机顺序如下：

### 23DOF 版本

| Joint Index in IDL | Joint Name (LowCmd_.mode or LowState_.mode == 0) | Joint Name (LowCmd_.mode or LowState_.mode == 1) |
| ------------------ | ------------------------------------------------ | ------------------------------------------------ |
| 0                  | L_LEG_HIP_PITCH                                  | L_LEG_HIP_PITCH                                  |
| 1                  | L_LEG_HIP_ROLL                                   | L_LEG_HIP_ROLL                                   |
| 2                  | L_LEG_HIP_YAW                                    | L_LEG_HIP_YAW                                    |
| 3                  | L_LEG_KNEE                                       | L_LEG_KNEE                                       |
| 4                  | **L_LEG_ANKLE_PITCH**                            | **L_LEG_ANKLE_B**                                |
| 5                  | **L_LEG_ANKLE_ROLL**                             | **L_LEG_ANKLE_A**                                |
| 6                  | R_LEG_HIP_PITCH                                  | R_LEG_HIP_PITCH                                  |
| 7                  | R_LEG_HIP_ROLL                                   | R_LEG_HIP_ROLL                                   |
| 8                  | R_LEG_HIP_YAW                                    | R_LEG_HIP_YAW                                    |
| 9                  | R_LEG_KNEE                                       | R_LEG_KNEE                                       |
| 10                 | **R_LEG_ANKLE_PITCH**                            | **R_LEG_ANKLE_B**                                |
| 11                 | **R_LEG_ANKLE_ROLL**                             | **R_LEG_ANKLE_A**                                |
| 12                 | TORSO                                            | TORSO                                            |
| 13                 | L_SHOULDER_PITCH                                 | L_SHOULDER_PITCH                                 |
| 14                 | L_SHOULDER_ROLL                                  | L_SHOULDER_ROLL                                  |
| 15                 | L_SHOULDER_YAW                                   | L_SHOULDER_YAW                                   |
| 16                 | L_ELBOW_PITCH                                    | L_ELBOW_PITCH                                    |
| 17                 | L_ELBOW_ROLL                                     | L_ELBOW_ROLL                                     |
| 18                 | R_SHOULDER_PITCH                                 | R_SHOULDER_PITCH                                 |
| 19                 | R_SHOULDER_ROLL                                  | R_SHOULDER_ROLL                                  |
| 20                 | R_SHOULDER_YAW                                   | R_SHOULDER_YAW                                   |
| 21                 | R_ELBOW_PITCH                                    | R_ELBOW_PITCH                                    |
| 22                 | R_ELBOW_ROLL                                     | R_ELBOW_ROLL                                     |

### 29DOF 版本

| Joint Index in IDL | Joint Name (LowCmd_.mode or LowState_.mode == 0) | Joint Name (LowCmd_.mode or LowState_.mode == 1) |
| ------------------ | ------------------------------------------------ | ------------------------------------------------ |
| 0                  | L_LEG_HIP_PITCH                                  | L_LEG_HIP_PITCH                                  |
| 1                  | L_LEG_HIP_ROLL                                   | L_LEG_HIP_ROLL                                   |
| 2                  | L_LEG_HIP_YAW                                    | L_LEG_HIP_YAW                                    |
| 3                  | L_LEG_KNEE                                       | L_LEG_KNEE                                       |
| 4                  | **L_LEG_ANKLE_PITCH**                            | **L_LEG_ANKLE_B**                                |
| 5                  | **L_LEG_ANKLE_ROLL**                             | **L_LEG_ANKLE_A**                                |
| 6                  | R_LEG_HIP_PITCH                                  | R_LEG_HIP_PITCH                                  |
| 7                  | R_LEG_HIP_ROLL                                   | R_LEG_HIP_ROLL                                   |
| 8                  | R_LEG_HIP_YAW                                    | R_LEG_HIP_YAW                                    |
| 9                  | R_LEG_KNEE                                       | R_LEG_KNEE                                       |
| 10                 | **R_LEG_ANKLE_PITCH**                            | **R_LEG_ANKLE_B**                                |
| 11                 | **R_LEG_ANKLE_ROLL**                             | **R_LEG_ANKLE_A**                                |
| 12                 | WAIST_YAW                                        | WAIST_YAW                                        |
| 13                 | **WAIST_ROLL**                                   | **WAIST_A**                                      |
| 14                 | **WAIST_PITCH**                                  | **WAIST_B**                                      |
| 15                 | L_SHOULDER_PITCH                                 | L_SHOULDER_PITCH                                 |
| 16                 | L_SHOULDER_ROLL                                  | L_SHOULDER_ROLL                                  |
| 17                 | L_SHOULDER_YAW                                   | L_SHOULDER_YAW                                   |
| 18                 | L_ELBOW                                          | L_ELBOW                                          |
| 19                 | L_WRIST_ROLL                                     | L_WRIST_ROLL                                     |
| 20                 | L_WRIST_PITCH                                    | L_WRIST_PITCH                                    |
| 21                 | L_WRIST_YAW                                      | L_WRIST_YAW                                      |
| 22                 | R_SHOULDER_PITCH                                 | R_SHOULDER_PITCH                                 |
| 23                 | R_SHOULDER_ROLL                                  | R_SHOULDER_ROLL                                  |
| 24                 | R_SHOULDER_YAW                                   | R_SHOULDER_YAW                                   |
| 25                 | R_ELBOW                                          | R_ELBOW                                          |
| 26                 | R_WRIST_ROLL                                     | R_WRIST_ROLL                                     |
| 27                 | R_WRIST_PITCH                                    | R_WRIST_PITCH                                    |
| 28                 | R_WRIST_YAW                                      | R_WRIST_YAW                                      |

### 14DOF 双臂版本

| Joint Index in IDL | Joint Name       |
| ------------------ | ---------------- |
| 0                  | (empty)          |
| 1                  | (empty)          |
| 2                  | (empty)          |
| 3                  | (empty)          |
| 4                  | (empty)          |
| 5                  | (empty)          |
| 6                  | (empty)          |
| 7                  | (empty)          |
| 8                  | (empty)          |
| 9                  | (empty)          |
| 10                 | (empty)          |
| 11                 | (empty)          |
| 12                 | (empty)          |
| 13                 | (empty)          |
| 14                 | (empty)          |
| 15                 | L_SHOULDER_PITCH |
| 16                 | L_SHOULDER_ROLL  |
| 17                 | L_SHOULDER_YAW   |
| 18                 | L_ELBOW          |
| 19                 | L_WRIST_ROLL     |
| 20                 | L_WRIST_PITCH    |
| 21                 | L_WRIST_YAW      |
| 22                 | R_SHOULDER_PITCH |
| 23                 | R_SHOULDER_ROLL  |
| 24                 | R_SHOULDER_YAW   |
| 25                 | R_ELBOW          |
| 26                 | R_WRIST_ROLL     |
| 27                 | R_WRIST_PITCH    |
| 28                 | R_WRIST_YAW      |

## Dex3-1 关节电机顺序

`unitree_hg::msg::dds_::HandCmd_.motor_cmd` 与 `unitree_hg::msg::dds_::HandState_.motor_state` 包含所有的灵巧手电机的信息，其电机顺序如下：

| Hand Joint Index in IDL | Hand Joint Name |
| ----------------------- | --------------- |
| 0                       | thumb_0         |
| 1                       | thumb_1         |
| 2                       | thumb_2         |
| 3                       | index_0         |
| 4                       | index_1         |
| 5                       | middle_0        |
| 6                       | middle_1        |
