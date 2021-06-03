+++
title = "Core"
description = "Core"
date = "2019-08-07"
author = ""
sec = 2
+++

The core part of RelaxedIK and CollisionIK is at [RelaxedIK Core](https://github.com/uwgraphics/relaxed_ik_core). The core contains a Rust implementation of RelaxedIK and CollisionIK with both a library crate and a binary crate. The library crate includes all the kinematics libraries of RelaxedIK, while the binary crate is desgined for the purpose of testing. Since the core is not designed to be run independently in general (although it is runnable), please refer to the Wrappers section and READMEs in those wrapper repos for more information on how to run RelaxedIK or CollisionIK.

To introduce relaxed_ik_core, let's first talk about the previous iteration of RelaxedIK at [relaxed_ik](https://github.com/uwgraphics/relaxed_ik/tree/dev). There are three parts in this repo: the Rust library of RelaxedIK, a ROS wrapper around it, and a preprocessing toolkit for robot config files needed for RelaxedIK. To make it more convenient to extend RelaxedIK and build all kinds of wrappers around it, we extract the Rust library from that repo to be relaxed_ik_core. Also, to ease the process of setting up a robot arm with RelaxedIK, we decide to exclude the preprocessing toolkit (which is accessible in [relaxed_ik](https://github.com/uwgraphics/relaxed_ik/tree/dev)) from relaxed_ik_core and instead provide all of the pre-generated config files for some mostly used robot manipulators. The list of available robots include baxter, hubo, iiwa7, jaco7, panda, sawyer, ur5, and yumi. Please refer to the READMEs in those repos if you intend to work with a robot not in this list.