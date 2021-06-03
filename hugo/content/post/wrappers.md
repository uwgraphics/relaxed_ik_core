+++
title = "Wrappers"
description = "Wrappers"
date = "2019-08-07"
author = ""
sec = 3
+++

Although the core is runnable, it is not designed to be run alone. Wrappers are recommended to connect RelaxedIK to different interfaces. Here are 4 wrappers that we implemented for ROS, CoppeliaSim, Mujoco, and Unity respectively. Each wrapper has detailed instructions on how to work with them in the READMEs in their repo. If you are interested, it is possible to wrap up the core in other interfaces as well. 

The links to these github repos are here:

- [RelaxedIK ROS1](https://github.com/uwgraphics/relaxed_ik_ros1)
- [RelaxedIK Unity](https://github.com/uwgraphics/relaxed_ik_unity)
- [RelaxedIK CoppeliaSim](https://github.com/uwgraphics/relaxed_ik_coppeliasim)
- [RelaxedIK Mujoco](https://github.com/uwgraphics/relaxed_ik_mujoco)

### [RelaxedIK ROS1](https://github.com/uwgraphics/relaxed_ik_ros1)
- This wrapper has the complete set of features available in the RelaxedIK package and it is also where CollisionIK resides. If you doesn’t have strong preferences over any specific wrapper, you probably should consider this ROS wrapper as the first choice. A keyboard pose goal driver and an rviz viewer are provided for testing purpose.

### [RelaxedIK CoppeliaSim](https://github.com/uwgraphics/relaxed_ik_coppeliasim)
- This wrapper is a RelaxedIK plugin for CoppeliaSim. Although it’s possible to access the ROS1 wrapper of RelaxedIK in CoppeliaSim through ROS topics and params, it might be more convenient to directly access RelaxedIK in the form of a CoppeliaSim Pluggin. That’s where the inspiration of this wrapper comes from. If you don't need ROS in your project, this wrapper is designed for you; however, if you need ROS for other parts of your project, it is still recommended to use the ROS wrapper since that one has the complete set of features available.

### [RelaxedIK Mujoco](https://github.com/uwgraphics/relaxed_ik_mujoco)
- This wrapper is a RelaxedIK Plugin for MuJoCo. MuJoCo is an advanced physics simulation engine that may have some extra features unavailable in CoppeliaSim.

### [RelaxedIK Unity](https://github.com/uwgraphics/relaxed_ik_unity)
- This wrapper allows users to use RelaxedIK in Unity. This is designed to work on Windows and it probably won’t work on a Linux machine. A few commonly used simulated robot arms have already been set up for you to play with. In the simulation, you will be able to disable or enable RelaxedIK as you like. When RelaxedIK is disabled, a joint angle writer panel will show up for you to adjust and visualize the joint angle configuration. When RelaxedIK is enabled, you will be able to have real-time interactions with RelaxedIK by dragging the transform gizmo associated with the gripper of the robot.

    <img src="images/unity2.png" alt="Unity screenshot" width="100%"/>
    <p style="text-align:center; padding-bottom:10px">RelaxedIK Unity</p>