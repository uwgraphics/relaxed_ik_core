# CollisionIK Core

Note: if you encounter a compilation error, please refer to [this issue](https://github.com/uwgraphics/relaxed_ik_ros1/issues/7).

Meanwhile, the [ranged-ik](https://github.com/uwgraphics/relaxed_ik_core/tree/ranged-ik) branch is recently maintained and works with more recent rust versions.

## Introduction

[documentation](https://uwgraphics.github.io/relaxed_ik_core/)

Welcome to CollisionIK! This solver implements the methods discussed in our paper: [*CollisionIK: A Per-Instant Pose Optimization Method for Generating Robot Motions with Environment Collision Avoidance*](https://arxiv.org/abs/2102.13187)

- [Video explaining CollisionIK](https://youtu.be/rdMl1gOPNoM)

CollisionIK is a per-instant pose optimization method that can generate configurations that achieve specified pose or motion objectives as best as possible over a sequence of solutions, while also simultaneously avoiding collisions with static or dynamic obstacles in the environment. CollisionIK builds on our prior work on [*RelaxedIK: Real-time Synthesis of Accurate and Feasible Robot Arm Motion*](http://www.roboticsproceedings.org/rss14/p43.html), and that's why we put it as one branch of this RelaxedIK Core repo.

This repo contains a Rust implemntation of CollisionIK in the form of both a library crate and a binary crate. The library crate simply includes the Rust code base of CollisionIK, and the binary crate is desgined to be executed as a binary which takes a Cartesian-space goal as input and returns a robot joint configuration. Although this repo is runnable when compiled as a binary crate, it is not designed to run independently. In most cases, you would like to use [this Collision IK ROS wrapper](https://github.com/uwgraphics/relaxed_ik_ros1) that connects the Rust library to ROS.

We provide all of the pre-generated config files for some mostly used robot manipulators, including a Universal Robots UR5 (6-DOF), a Rethink Robotics Sawyer (7-DOF), a Kinova Jaco (7-DOF), a Kuka IIWA (7-DOF), a Franka Panda (7-DOF), an ABB Yumi (14-DOF), a Rethink Robotics Baxter (14-DOF), and the Rainbow Robotics Hubo+ (15-DOF). If your robot is not included in this list, please follow the specifc instructions in the "Getting Started" section.

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
2. If you intend to investigate or run this repo independently, simply clone this repo into your destination folder. If you are going to use one of the wrappers, take a look at the README in their corresponding repo before proceeding. 
3. If your robot is in this list: [baxter, hubo, iiwa7, jaco7, panda, sawyer, ur5, yumi], ignore this step. Else, you will need to clone [this repo](https://github.com/uwgraphics/relaxed_ik) and follow the step-by-step guide [there](https://github.com/uwgraphics/relaxed_ik/blob/dev/src/start_here.py) to get the required robot config files into corresponding folders in the *config* folder. To specify, there should be (replace "sawyer" with your robot name or your urdf name in some cases):
    - 1 self-collision file <collision_sawyer.yaml> in the *collision_files* folder
    - 4 Rust neural network files <sawyer_nn, sawyer_nn.yaml, sawyer_nn_jointpoint, sawyer_nn_jointpoint.yaml> in the *collision_nn_rust* folder
    - 1 info file <sawyer_info.yaml> in the *info_files* folder
    - 1 joint state function file <sawyer_joint_state_define> in the *joint_state_define_functions* folder
    - 1 urdf file <sawyer.urdf> in the *urdfs* folder.
4. Look at <settings.yaml> in the *config* folder and follow the information there to customize the parameters. Note that you don't need to recompile *relaxed_ik_core* every time you change the parameters in <settings.yaml>.
5. Compile this repo:
    ```bash
    cargo build
    ```
6. If you don't intend to execute the binary compiled from this repo, ignore this step. Otherwise, type this command and follow the prompts in your console to try it out:
    ```bash
    cargo run --bin relaxed_ik_bin
    ```
7. Enjoy working with CollisionIK!
