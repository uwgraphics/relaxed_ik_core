# RelaxedIK Core

## Introduction

Welcome to RelaxedIK! This solver implements the methods discussed in our paper: [*RelaxedIK: Real-time Synthesis of Accurate and Feasible Robot Arm Motion*](http://www.roboticsproceedings.org/rss14/p43.html)

- Video of presentation at RSS 2018 (RelaxedIK part starts around 12:00): https://youtu.be/bih5e9MHc88?t=737
- Video explaining RelaxedIK: https://youtu.be/AhsQFJzB8WQ

RelaxedIK is an inverse kinematics (IK) solver designed for robot platforms such that the conversion between Cartesian end-effector pose goals (such as “move the robot’s right arm end-effector to position X, while maintaining an end-effector orientation Y”) to joint space (i.e., the robot’s rotation values for each joint degree-of-freedom at a particular time-point) is done both accurately and feasibly. RelaxedIK attempts to find the closest possible solution to the desired end-effector pose goals without exhibiting negative effects such as self-collisions, environment collisions, kinematic-singularities, or joint-space discontinuities.

The Rust version of [RelaxedIK (Deprecated)](https://github.com/uwgraphics/relaxed_ik/tree/dev) consists of three parts: a Rust library of RelaxedIK, a ROS1 wrapper around it, and a preprocessing toolkit to generate robot config files needed for RelaxedIK. To make it more convenient to iterate and build other wrappers around it, we extracted the Rust library from it and refractored the library into this repo. This repo contains a Rust implemntation of RelaxedIK in the form of both a library crate and a binary crate. The library crate simply includes the Rust code base of RelaxedIK, and the binary crate is desgined to be executed as a binary which takes a Cartesian-space goal as input and returns a robot joint configuration. Although this repo is runnable when compiled as a binary crate, it is not designed to run independently. In most cases, one of the RelaxedIK wrappers that connects RelaxedIK Core to various interfaces should be more useful.

The preprocessing toolkit in the origianl version of RelaxedIK was excluded from this repo to keep the modularity. Instead, we provide all of the pre-generated config files for some mostly used robot manipulators, including a Universal Robots UR5 (6-DOF), a Rethink Robotics Sawyer (7-DOF), a Kinova Jaco (7-DOF), a Kuka IIWA (7-DOF), a Franka Panda (7-DOF), an ABB Yumi (14-DOF), a Rethink Robotics Baxter (14-DOF), and the Rainbow Robotics Hubo+ (15-DOF). If your robot is not included in this list, please follow the specific instructions in the "Getting Started" section.

If anything with the solver is not working as expected, or if you have any feedback, feel free to let us know! (email: rakita@cs.wisc.edu, website: http://pages.cs.wisc.edu/~rakita) We are actively supporting and extending this code, so we are interested to hear about how the solver is being used and any positive or negative experiences in using it.

## Citation

If you use RelaxedIK, please cite our RSS paper: [*RelaxedIK: Real-time Synthesis of Accurate and Feasible Robot Arm Motion*](http://www.roboticsproceedings.org/rss14/p43.html)
```
@INPROCEEDINGS{Rakita-RSS-18, 
    AUTHOR    = {Daniel Rakita AND Bilge Mutlu AND Michael Gleicher}, 
    TITLE     = {{RelaxedIK: Real-time Synthesis of Accurate and Feasible Robot Arm Motion}}, 
    BOOKTITLE = {Proceedings of Robotics: Science and Systems}, 
    YEAR      = {2018}, 
    ADDRESS   = {Pittsburgh, Pennsylvania}, 
    MONTH     = {June}, 
    DOI       = {10.15607/RSS.2018.XIV.043} 
}
```

If you use CollisionIK (RelaxedIK with environment collision avoidance), please cite our ICRA paper: [*CollisionIK: A Per-Instant Pose Optimization Method for Generating Robot Motions with Environment Collision Avoidance*](https://arxiv.org/abs/2102.13187)
```
@article{rakita2021collisionik,
  title={CollisionIK: A Per-Instant Pose Optimization Method for Generating Robot Motions with Environment Collision Avoidance},
  author={Rakita, Daniel and Shi, Haochen and Mutlu, Bilge and Gleicher, Michael},
  journal={arXiv preprint arXiv:2102.13187},
  year={2021}
}
```

If you use our solver for a robot teleoperation interface, also consider citing our prior work that shows the effectiveness of RelaxedIK in this setting: [*A Motion Retargeting Method for Effective Mimicry-based Teleoperation of Robot Arms*](https://dl.acm.org/citation.cfm?id=3020254)
```
@inproceedings{rakita2017motion,
  title={A motion retargeting method for effective mimicry-based teleoperation of robot arms},
  author={Rakita, Daniel and Mutlu, Bilge and Gleicher, Michael},
  booktitle={Proceedings of the 2017 ACM/IEEE International Conference on Human-Robot Interaction},
  pages={361--370},
  year={2017},
  organization={ACM}
}
```

or [*An Autonomous Dynamic Camera Method for Effective Remote Teleoperation*](https://dl.acm.org/citation.cfm?id=3171221.3171279)
```
@inproceedings{rakita2018autonomous,
  title={An autonomous dynamic camera method for effective remote teleoperation},
  author={Rakita, Daniel and Mutlu, Bilge and Gleicher, Michael},
  booktitle={Proceedings of the 2018 ACM/IEEE International Conference on Human-Robot Interaction},
  pages={325--333},
  year={2018},
  organization={ACM}
}
```

## RelaxedIK Family

More information about RelaxedIK, Collision IK, and all the wrappers could be found in this [documentation](https://uwgraphics.github.io/relaxed_ik_core/).

- [RelaxedIK (Deprecated)](https://github.com/uwgraphics/relaxed_ik/tree/dev)
- [RelaxedIK Core](https://github.com/uwgraphics/relaxed_ik_core)
- [RelaxedIK ROS1](https://github.com/uwgraphics/relaxed_ik_ros1)
- [RelaxedIK Unity](https://github.com/uwgraphics/relaxed_ik_unity)
- [RelaxedIK CoppeliaSim](https://github.com/uwgraphics/relaxed_ik_coppeliasim)
- [RelaxedIK Mujoco](https://github.com/uwgraphics/relaxed_ik_mujoco)

||**RelaxedIK (Deprecated)**|**RelaxedIK ROS1**|**RelaxedIK Unity**|**RelaxedIK Coppeliasim**|**RelaxedIK Mujoco**|  
|:------|:-----|:-----|:-----|:-----|:-----| 
|**RelaxedIK**|:o:|:o:|:o:|:o:|:o:|  
|**Collision IK**|:x:|:o:|:x:|:x:|:x:|  

## Dependencies

1. This repo is runnable on all the operating systems that support Rust.
1. To use relaxed_ik_core and all the wrappers that include relaxed_ik_core, you will need to install Rust. Please go to https://www.rust-lang.org/learn/get-started for more infomation.

### Getting Started

1. Make sure that you have installed all the dependencies.
1. If you intend to investigate or run this repo independently, simply clone this repo into your destination folder. If you are going to use one of the wrappers of RelaxedIK, take a look at the README in their corresponding repo before proceeding. 
1. If your robot is in this list: [baxter, hubo, iiwa7, jaco7, panda, sawyer, ur5, yumi], ignore this step. Else, you will need to clone [this repo](https://github.com/uwgraphics/relaxed_ik) and follow the step-by-step guide [there](https://github.com/uwgraphics/relaxed_ik/blob/dev/src/start_here.py) to get the required robot config files into corresponding folders in the *config* folder. To specify, there should be (replace "sawyer" with your robot name or your urdf name in some cases):
    - 1 self-collision file <collision_sawyer.yaml> in the *collision_files* folder
    - 4 Rust neural network files <sawyer_nn, sawyer_nn.yaml, sawyer_nn_jointpoint, sawyer_nn_jointpoint.yaml> in the *collision_nn_rust* folder
    - 1 info file <sawyer_info.yaml> in the *info_files* folder
    - 1 joint state function file <sawyer_joint_state_define> in the *joint_state_define_functions* folder
    - 1 urdf file <sawyer.urdf> in the *urdfs* folder.
1. Select the robot you want to work with by typing the corresponding info file name of the robot (e.g. "sawyer_info.yaml") into <config/loaded_robot>. Note that you don't need to recompile *relaxed_ik_core* every time you change the robot name.
1. Compile this repo:
    ```bash
    cargo build
    ```
1. If you don't intend to execute the binary compiled from this repo, ignore this step. Otherwise, type this command and follow the prompts in your console to try it out:
    ```bash
    cargo run --bin relaxed_ik_bin
    ```
1. Enjoy working with RelaxedIK!
