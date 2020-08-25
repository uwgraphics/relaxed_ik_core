+++
title = "Wrappers"
description = "Wrappers"
date = "2019-08-07"
author = ""
sec = 3
+++

Although the core is runnable, it is usually not intended to be run alone. Wrappers are required to connect RelaxedIK to different interfaces. Here are 5 wrappers that we implemented for ROS, ROS2, CoppeliaSim, Mujoco, and Unity respectively. Each wrapper has detailed instructions on how to work with them in the README in their repo. If you are interested, it is possible to wrap up the core in other interfaces as well. 

### Relaxed IK ROS
+ Available at [relaxed_ik_ros1](https://github.com/uwgraphics/relaxed_ik_ros1).
+ This is a lighter-weight version of RelaxedIK wrapped up in ROS1. A keyboard pose goal driver and a rviz viewer are included in this wrapper for you to play with. You can treat this wrapper as a new iteration over [relaxed_ik](https://github.com/uwgraphics/relaxed_ik/tree/dev).
+ Please refer to the README in the repo for more infomation.

### Relaxed IK ROS2 (Under developed)
+ This is RelaxedIK wrapped up in ROS2.

### Relaxed IK CoppeliaSim
+ Available at [relaxed_ik_coppeliasim](https://github.com/uwgraphics/relaxed_ik_coppeliasim).
+ This is a RelaxedIK plugin for CoppeliaSim. Although it's possible to use RelaxedIK in CoppeliaSim via ROS and the ROS wrapper of RelaxedIK at [relaxed_ik_ros1](https://github.com/uwgraphics/relaxed_ik_ros1), it might be more convenient to directly access RelaxedIK in the form of a CoppeliaSim Pluggin. That's why we developed this wrapper.
+ Please refer to the README in the repo for more infomation.

### Relaxed IK Mujoco
+ Available at [relaxed_ik_mujoco](https://github.com/uwgraphics/relaxed_ik_mujoco).
+ This is a RelaxedIK wrapper for Mujoco.
+ Please refer to the README in the repo for more infomation.

### Relaxed IK Unity
+ Available at [relaxed_ik_unity](https://github.com/uwgraphics/relaxed_ik_unity).
+ This is the Unity wrapper of RelaxedIK designed to be run on Windows. Note that it probably won't work on a linux machine. The simulation of a few commonly used robot arms have already been set up for you to play with. You will be able to disable and enable RelaxedIK as you like. When RelaxedIK is disabled, a joint angle writer panel will show up for you to modify the joint angle configuration. When RelaxedIK is enabled, you will be able to have real-time interactions with the robot and RelaxedIK by dragging the transform gizmo associated with the gripper of the robot. Here is a screenshot of what it looks like currently:

<img src="images/unity1.png" alt="Unity screenshot1" width="100%"/>

<p style="text-align:center; padding-bottom:10px">(1) Unity RelaxedIK disabled</p>

<img src="images/unity2.png" alt="Unity screenshot2" width="100%"/>

<p style="text-align:center; padding-bottom:10px">(2) Unity RelaxedIK enabled</p>

+ Please refer to the README in the repo for more infomation.