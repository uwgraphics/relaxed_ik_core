# RangedIK Core

## Introduction

This solver implements the methods discussed in our paper: [*RangedIK: An Optimization-Based Robot Motion Generation Method for Ranged-Goal Tasks*][Link to appear][Video to appear]

[Introduction Plase holder]

## Getting Started

1. [Install Rust](https://www.rust-lang.org/learn/get-started)
2. Compile:
    ```bash
    cargo build
    ```
3. Run a small demo:
    ```bash
    cargo run --bin relaxed_ik_bin
    ```
### Use your own robot
1. Place your robot's URDF under `configs/urdfs/`
2. Make a setting file. Examples are under `configs/example_settings`

### Wrappers 
`wrappers/python_wrapper.py` provides a python wrapper. 

`relaxed_ik_ros1/scripts/relaxed_ik_rust.py` is an example to use the python wrapper.


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
@inproceedings{rakita2021collisionik,
  title={Collisionik: A per-instant pose optimization method for generating robot motions with environment collision avoidance},
  author={Rakita, Daniel and Shi, Haochen and Mutlu, Bilge and Gleicher, Michael},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={9995--10001},
  year={2021},
  organization={IEEE}
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

### Todo list

- [ ] Add demo that exploit tolerances
- [ ] Support environment collision avoidance
- [ ] Add wrapper for JavaScript
