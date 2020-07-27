+++
title = "Wrappers"
description = "Wrappers"
date = "2019-08-07"
author = ""
sec = 3
+++

Although the core is runnable, it is usually not intended to be run alone. Wrappers are required to connect it to different interfaces. Here are three wrappers that we implemented for ROS, ROS2, and Unity respectively. If you are interested, it is possible to wrap up the core to other interfaces as well. Each of them have detailed instructions on how to work with them in their README.

### Relaxed IK ROS
Available at [relaxed_ik_ros1](https://github.com/uwgraphics/relaxed_ik_ros1).

This is a cleaner version of RelaxedIK wrapped up in ROS1 with pre-generated config files of some mostly used robot arms. A keyboard pose goal driver and a rviz viewer are included in this wrapper for you to play with. However, you will not be able to customize your own config files in this wrapper. You can go to this link instead [relaxed_ik](https://github.com/uwgraphics/relaxed_ik) for the complete version. In the complete version, you will have the opportunity to (and have to) set up everything from scratch, including importing a urdf, configuring the collision files, and training the neural network, etc. The complete version is recommended if you would like to work on the configuration process yourself or a robot that we didn't include in the available options; otherwise, this wrapper is likely to make your life much easier since everything you need to do will be setting the name of the robot you would like to work with.

Please refer to the README in the repo for more infomation.

### Relaxed IK ROS2 (Under developed)
Under developed at [relaxed_ik_ros2](https://github.com/uwgraphics/relaxed_ik_ros2).

This is RelaxedIK wrapped up in ROS2.

### Relaxed IK Unity
Available at [relaxed_ik_unity](https://github.com/uwgraphics/relaxed_ik_unity).

This is the Unity wrapper of RelaxedIK designed to be run on Windows. Note that it probably won't work on a linux machine. The simulation of a few commonly used robot arms have already been set up for you to play with. You will be able to disable and enable RelaxedIK as you like. When RelaxedIK is disabled, a joint angle writer panel will show up for you to modify the joint angle configuration. When RelaxedIK is enabled, you will be able to have real-time interactions with the robot and RelaxedIK by dragging the transform gizmo associated with the gripper of the robot. Here is a screenshot of what it looks like currently:

<img src="images/unity1.png" alt="Unity screenshot1" width="100%"/>

<p style="text-align:center; padding-bottom:10px">(1) Unity RelaxedIK disabled</p>

<img src="images/unity2.png" alt="Unity screenshot2" width="100%"/>

<p style="text-align:center; padding-bottom:10px">(2) Unity RelaxedIK enabled</p>

Please refer to the README in the repo for more infomation.