+++
title = "Core"
description = "Core"
date = "2019-08-07"
author = ""
sec = 2
+++

The core part of RelaxedIK is at [relaxed_ik_core](https://github.com/uwgraphics/relaxed_ik_core). The core contains a Rust implementation of RelaxedIK with both a library crate and a binary crate. The library crate includes all the kinematics libraries of RelaxedIK, while the binary crate is desgined for the purpose of testing. Since the core doesn't run independently on a pre-computed robot, please refer to the wrapper section for more information.
