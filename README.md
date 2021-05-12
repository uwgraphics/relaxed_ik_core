# Relaxed IK Core

## Introduction

Relaxed IK is an inverse kinematics (IK) solver designed for robot platforms such that the conversion between Cartesian end-effector pose goals (such as “move the robot’s right arm end-effector to position X, while maintaining an end-effector orientation Y”) to joint space (i.e., the robot’s rotation values for each joint degree-of-freedom at a particular time-point) is done both accurately and feasibly. Relaxed IK attempts to find the closest possible solution to the desired end-effector pose goals without exhibiting negative effects such as self-collisions, environment collisions, kinematic-singularities, or joint-space discontinuities.

The Rust version of [Relaxed IK (Deprecated)](https://github.com/uwgraphics/relaxed_ik/tree/dev) consists of three parts: a Rust library of Relaxed IK, a ROS1 wrapper around it, and a preprocessing toolkit to generate robot config files needed for Relaxed IK. To make it more convenient to iterate and build other wrappers around it, we extracted the Rust library from it and refractored the library into this repo. This repo contains a Rust implemntation of Relaxed IK in the form of both a library crate and a binary crate. The library crate simply includes the Rust code base of Relaxed IK, and the binary crate is desgined to be executed as a binary which takes a Cartesian-space goal as input and returns a robot joint configuration. Although this repo is runnable when compiled as a binary crate, it is not designed to run independently. In most cases, one of the Relaxed IK wrappers that connects Relaxed IK Core to various interfaces should be more useful.

The preprocessing toolkit in the origianl version of Relaxed IK was excluded from this repo to keep the modularity. Instead, we provide all of the pre-generated config files for some mostly used robot manipulators, including a Universal Robots UR5 (6-DOF), a Rethink Robotics Sawyer (7-DOF), a Kinova Jaco (7-DOF), a Kuka IIWA (7-DOF), a Franka Panda (7-DOF), an ABB Yumi (14-DOF), a Rethink Robotics Baxter (14-DOF), and the Rainbow Robotics Hubo+ (15-DOF). If your robot is not included in this list, please follow the instructions given below in the "Getting Started" section.

## Relaxed IK Family

More information about Relaxed IK, Collision IK, and all the wrappers could be found in this [documentation](https://uwgraphics.github.io/relaxed_ik_core/).

- [Relaxed IK (Deprecated)](https://github.com/uwgraphics/relaxed_ik/tree/dev)
- [Relaxed IK Core](https://github.com/uwgraphics/relaxed_ik_core)
- [Relaxed IK ROS1](https://github.com/uwgraphics/relaxed_ik_ros1)
- [Relaxed IK Unity](https://github.com/uwgraphics/relaxed_ik_unity)
- [Relaxed IK CoppeliaSim](https://github.com/uwgraphics/relaxed_ik_coppeliasim)
- [Relaxed IK Mujoco](https://github.com/uwgraphics/relaxed_ik_mujoco)

||**Relaxed IK (Deprecated)**|**Relaxed IK ROS1**|**Relaxed IK Unity**|**Relaxed IK Coppeliasim**|**Relaxed IK Mujoco**|  
|:------|:-----|:-----|:-----|:-----|:-----| 
|**Relaxed IK**|:o:|:o:|:o:|:o:|:o:|  
|**Collision IK**|:x:|:o:|:x:|:x:|:x:|  

## Dependencies

1. This repo is runnable on all the operating systems that support Rust.
1. To use relaxed_ik_core and all the wrappers that include relaxed_ik_core, you will need to install Rust. Please go to https://www.rust-lang.org/learn/get-started for more infomation.

### Getting Started

1. Make sure that you have installed all the dependencies.
1. If you intend to investigate or run this repo independently, simply clone this repo into your destination folder. If you are going to use one of the wrappers of Relaxed IK, take a look at the README in their corresponding repo before proceeding. 
2. If your robot is in this list: [baxter, hubo, iiwa7, jaco7, panda, sawyer, ur5, yumi], ignore this step. Else, you will need to clone [this repo](https://github.com/uwgraphics/relaxed_ik) and follow the step-by-step guide [there](https://github.com/uwgraphics/relaxed_ik/blob/dev/src/start_here.py) to get the required robot config files into corresponding folders in the *config* folder. To specify, there should be (replace "sawyer" with your robot name or your urdf name in some cases):
    - 1 self-collision file <collision_sawyer.yaml> in the *collision_files* folder
    - 4 Rust neural network files <sawyer_nn, sawyer_nn.yaml, sawyer_nn_jointpoint, sawyer_nn_jointpoint.yaml> in the *collision_nn_rust* folder
    - 1 info file <sawyer_info.yaml> in the *info_files* folder
    - 1 joint state function file <sawyer_joint_state_define> in the *joint_state_define_functions* folder
    - 1 urdf file <sawyer.urdf> in the *urdfs* folder.
3. Look at <settings.yaml> in the *config* folder and follow the information there to customize the parameters.
4. Compile this repo:
    ```bash
    cargo build
    ```
5. If you don't intend to execute the binary compiled from this repo, ignore this step. Otherwise, type this command and follow the prompts in your console to try it out:
    ```bash
    cargo run --bin relaxed_ik_bin
    ```
6. Enjoy working with Relaxed IK!