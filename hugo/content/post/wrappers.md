+++
title = "Wrappers"
description = "Wrappers"
date = "2019-08-07"
author = ""
sec = 3
+++

Although the core is runnable, it is usually not intended to be run alone. Here are three wrappers of RelaxedIK that we implemented for ROS, ROS2, and Unity respectively. If you are interested, it is possible to wrap up the core to other interfaces as well. Each of them have detailed instructions on how to work with them in README of their repos.

### Relaxed IK ROS
Available at [relaxed_ik_ros1](https://github.com/uwgraphics/relaxed_ik_ros1).

This is a cleaner version of RelaxedIK wrapped up in ROS1 with pre-generated config files of some mostly used robot arms. A keyboard pose goal driver and a rviz viewer are included in this wrapper to play with. However, you will not be able to customize your own config files in this wrapper. You can go to this link [relaxed_ik](https://github.com/uwgraphics/relaxed_ik) for the complete version instead. In the complete version, you will have the opportunity to set up everything from scratch, including importing a urdf, configuring the collision files, and training the neural network, etc.

### Relaxed IK ROS2 (Under developed)
Under developed at [relaxed_ik_ros2](https://github.com/uwgraphics/relaxed_ik_ros2).

This is RelaxedIK wrapped up in ROS2.

### Relaxed IK Unity
Available at [relaxed_ik_unity](https://github.com/uwgraphics/relaxed_ik_unity).

This is the Unity wrapper of RelaxedIK designed to be run on Windows. Note that it probably won't work on a Linux machine. The simulation of a few commonly used robot arms have already been set up to play with. 
