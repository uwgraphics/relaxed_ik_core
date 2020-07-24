+++
title = "About"
description = "Project description"
date = "2019-08-07"
author = ""
sec = 1
+++

Welcome to RelaxedIK! This solver implements the methods discussed in our paper <i> RelaxedIK: Real-time Synthesis of Accurate and Feasible Robot Arm Motion </i> (http://www.roboticsproceedings.org/rss14/p43.html)

+ Video of presentation at RSS 2018 (RelaxedIK part starts around 12:00): 
https://youtu.be/bih5e9MHc88?t=737
+ Video explaining RelaxedIK: 
https://youtu.be/AhsQFJzB8WQ

RelaxedIK is an inverse kinematics (IK) solver designed for robot platforms such that the conversion between Cartesian end-effector pose goals (such as "move the robot's right arm end-effector to position X, while maintaining an end-effector orientation Y") to Joint-Space (i.e., the robot's rotation values for each joint degree-of-freedom at a particular time-point) is done both ACCURATELY and FEASIBLY.  By this, we mean that RelaxedIK attempts to find the closest possible solution to the desired end-effector pose goals without exhibiting negative effects such as self-collisions, environment collisions, kinematic-singularities, or joint-space discontinuities.
