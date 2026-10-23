# Robot Descriptions

URDF, Xacro and MJCF models for the Unitree robots supported by this workspace.
Directories without a `package.xml` are plain model assets: they can be loaded directly in RViz or MuJoCo, but are not built by `catkin_make`.

`DOF` counts the actuated joints in the model files. `n + m` means `n` body joints plus `m` wheel joints, and values in parentheses include optional dexterous hands or extra transmission joints.

| Family | Packages | Robots |
| :--- | ---: | :--- |
| Quadrupeds | 9 | A1, A2, AlienGo, As2, B1, B2, Go1, Go2, Laikago |
| Wheeled quadrupeds | 3 | As2-W, B2-W, Go2-W |
| Humanoids | 9 | G1, G1+, G1 with BrainCo Hand, H1, H1-2, H2, H2 Plus, H2-X, R1, R1 Air |
| Wheeled humanoids | 2 | G1-D, H2-D |
| Mobile manipulators | 2 | AlienGo with Z1, R1-A7-D |
| Robotic arms | 4 | Z1, R1-7a, R1-A5, R1-A7 |
| Dexterous hands | 1 | Dex1-1, Dex2/5, Dex3-1, Dex5-1 |
| MuJoCo-only | 1 | B2 |

`G1+` is a model file (`g1_pl.urdf`) inside `g1_description`, so the humanoid count lists one more robot than package.

## Quadrupeds

| Robot | Package | Models | DOF | Notes |
| :--- | :--- | :--- | ---: | :--- |
| A1 | [`a1_description`](a1_description) | Xacro + URDF | 12 | `a1_rviz.launch`; Gazebo controller config |
| A2 | [`a2_description`](a2_description) | URDF + MJCF | 12 | `a2.xml` for MuJoCo; no catkin package |
| AlienGo | [`aliengo_description`](aliengo_description) | Xacro + URDF | 12 | `aliengo_rviz.launch`; Gazebo controller config |
| As2 | [`as2_description`](as2_description) | URDF + MJCF | 12 | `as2.xml` for MuJoCo; no catkin package |
| B1 | [`b1_description`](b1_description) | Xacro | 12 | `b1_rviz.launch`; Gazebo controller config |
| B2 | [`b2_description`](b2_description) | Xacro + URDF | 12 | `display.launch`, `gazebo.launch`; Gazebo controller config |
| Go1 | [`go1_description`](go1_description) | Xacro + URDF | 12 | `go1_rviz.launch`; Gazebo controller config |
| Go2 | [`go2_description`](go2_description) | Xacro + URDF | 12 | `go2_rviz.launch`, `gazebo.launch`; DAE meshes |
| Laikago | [`laikago_description`](laikago_description) | Xacro + URDF | 12 | `laikago_rviz.launch`; Gazebo controller config |

## Wheeled quadrupeds

| Robot | Package | Models | DOF | Notes |
| :--- | :--- | :--- | ---: | :--- |
| As2-W | [`as2w_description`](as2w_description) | URDF | 12 + 4 | Wheeled As2; no catkin package |
| B2-W | [`b2w_description`](b2w_description) | Xacro + URDF | 12 + 4 | `display.launch`, `gazebo.launch`; no controller config |
| Go2-W | [`go2w_description`](go2w_description) | URDF | 12 + 4 | `go2w_rviz.launch`, `gazebo.launch`; DAE meshes |

## Humanoids

| Robot | Package | Models | DOF | Notes |
| :--- | :--- | :--- | ---: | :--- |
| G1 | [`g1_description`](g1_description) | URDF + MJCF | 23 / 29 | 33 model files: 23/29-DOF, lock-waist, with-hand (Dex3, Inspire), dual-arm and G1-Comp variants; see its [README](g1_description/README.md) |
| G1+ | [`g1_description`](g1_description) | URDF (`g1_pl.urdf`) | 31 | 29-DOF G1 plus a 2-DOF neck; no hands |
| G1 with BrainCo Hand | [`g1_with_brainco_hand`](g1_with_brainco_hand) | URDF | 61 | Single 29-DOF URDF with BrainCo hands; no catkin package |
| H1 | [`h1_description`](h1_description) | URDF + MJCF | 19 (45) | `display.launch`, `gazebo.launch`; `h1_with_hand.urdf` (45); `mjcf/` |
| H1-2 | [`h1_2_description`](h1_2_description) | URDF + MJCF | 27 (51) | `h1_2_handless.urdf` (27), `h1_2_with_FTP_hand.urdf` (51); MJCF |
| H2 | [`h2_description`](h2_description) | URDF + MJCF | 31 | `H2.urdf` (31), `H2-A.urdf` (19, fixed base), `H2_loop` (closed-loop ankle, 37 motor joints), DAE variant; MJCF |
| H2 Plus | [`h2_plus`](h2_plus) | URDF | 31 (75) | 5-finger hands in the same URDF; no catkin package |
| H2-X | [`h2x_description`](h2x_description) | URDF | 31 (37) | Adds 6 ankle motor joints; no catkin package |
| R1 | [`r1_description`](r1_description) | URDF | 26 | Closed-loop ankle linkage; no catkin package |
| R1 Air | [`r1_air_description`](r1_air_description) | URDF | 20 | No waist or head joints; no catkin package |

## Wheeled humanoids

| Robot | Package | Models | DOF | Notes |
| :--- | :--- | :--- | ---: | :--- |
| G1-D | [`g1_d_description`](g1_d_description) | URDF | 17 / 19 | 2-DOF lift, 7-DOF arms, optional hands; 19 DOF with the 2-wheel chassis; no catkin package |
| H2-D | [`h2d_description`](h2d_description) | URDF | 19 | 7-DOF arms, 1-DOF waist, 2-DOF head, 2 wheels; lift and Dex1 hands in the URDF |

## Mobile manipulators

| Robot | Package | Models | DOF | Notes |
| :--- | :--- | :--- | ---: | :--- |
| AlienGo with Z1 | [`aliengoZ1_description`](aliengoZ1_description) | Xacro | 12 + 6 | Quadruped plus Z1 arm; `aliengoZ1_rviz.launch`, `aliengoZ1_gazebo.launch`; `worlds/` |
| R1-A7-D | [`r1d_description`](r1d_description) | URDF | 21 | Wheeled R1 (`R1D` model): 2 wheels + 2-DOF lift + 17-DOF upper body; 8 passive caster joints |

## Robotic arms

| Robot | Package | Models | DOF | Notes |
| :--- | :--- | :--- | ---: | :--- |
| Z1 | [`z1_description`](z1_description) | Xacro | 6 | 6-DOF arm plus gripper; launch with `unitree_gazebo/launch/z1.launch` |
| R1-7a | [`r1_7a_description`](r1_7a_description) | URDF | 7 | 7-DOF arm (`R1_Robotic_Arm`); no catkin package |
| R1-A5 | [`r1_a5_description`](r1_a5_description) | URDF | 13 | Waist yaw + 5-DOF arms + 2-DOF head |
| R1-A7 | [`r1_a7_description`](r1_a7_description) | URDF | 17 | Waist yaw + 7-DOF arms + 2-DOF head |

## Dexterous hands

| Hand | Package | Models | Joints | Notes |
| :--- | :--- | :--- | ---: | :--- |
| Dex1-1 | [`dex1_1`](dexterous_hand_description/dex1_1) | URDF | 2 | Parallel gripper with two prismatic fingers |
| Dex2/5 | [`dex2_5`](dexterous_hand_description/dex2_5) | URDF | 10 | Left and right hands, plus G1 5010 wrist variants |
| Dex3-1 | [`dex3_1`](dexterous_hand_description/dex3_1) | URDF | 7 | Three fingers; left and right |
| Dex5-1 | [`dex5_1`](dexterous_hand_description/dex5_1) | URDF | 20 | Five fingers; `Dex5-URDF-L` and `Dex5-URDF-R` |

## MuJoCo-only

| Robot | Package | Models | DOF | Notes |
| :--- | :--- | :--- | ---: | :--- |
| B2 | [`b2_description_mujoco`](b2_description_mujoco) | MJCF | 12 | `xml/scene.xml` and `xml/b2.xml` |

MJCF models are also bundled with `a2_description`, `as2_description`, `g1_description`, `h1_description`, `h1_2_description` and `h2_description`.
