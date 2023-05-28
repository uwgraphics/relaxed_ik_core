# RangedIK Core

Implementation of our ICRA'23 paper: *RangedIK: An Optimization-Based Robot Motion Generation Method for Ranged-Goal Tasks*

[[Pre-print](https://arxiv.org/pdf/2302.13935.pdf)]  [[Supplementary Video](https://www.youtube.com/watch?v=_QVAetYbpEY)] [[Presentation Video](https://www.youtube.com/watch?v=IKy0Yda8p4)] [[Poster](https://yepw.github.io/files/icra23_poster.pdf)]

## Introduction
RangedIK is a real-time motion synthesis method that exploits range flexibility to satisfy multiple kinematic requirements. RangedIK is particularly suitable for applications that allow some tolerance, such as wielding where the tool is allowed rotate along its principle axis. For more information, please refer to [our paper](https://arxiv.org/pdf/2302.13935.pdf). 

## Getting Started 

1. [Install Rust](https://www.rust-lang.org/learn/get-started)
2. Compile:
    ```bash
    cargo build
    ```
   The compiled library is at `/target/debug/librelaxed_ik_lib.so`
3. Run a small demo:
    ```bash
    cargo run --bin relaxed_ik_bin
    ```
### Use your own robot
1. Place your robot's URDF under `configs/urdfs/`
2. Make a setting file. Examples are under `configs/example_settings`

### Python (ROS) wrapper
`wrappers/python_wrapper.py` provides a python wrapper, which is used by the ROS 1 wrapper: [relaxed-ik-ros1](https://github.com/uwgraphics/relaxed_ik_ros1/tree/ranged-ik)

[`relaxed_ik_ros1/scripts/relaxed_ik_rust.py`](https://github.com/uwgraphics/relaxed_ik_ros1/blob/ranged-ik/scripts/relaxed_ik_rust.py) provides an example of using the python wrapper.

### JavaScript (WebAssembly) wrapper
1. [Install Rust](https://www.rust-lang.org/tools/install) 
2. `cargo install wasm-pack`
    * if there is a `linker 'cc' not found` error, run `sudo apt install gcc`
	* if it complains about openssl, on Ubuntu install it by `sudo apt-get install libssl-dev pkg-config openssl`. For windows, download and install [perl](https://strawberryperl.com/).
3. Compile to WebAssembly
    ```
    wasm-pack build --target web
    ```
4. [relaxed-ik-web-demo](https://github.com/yepw/relaxed-ik-web-demo) provides an example of running relaxed-ik in browser. 

## Supplementary Video

[YouTube video link](https://www.youtube.com/watch?v=_QVAetYbpEY)

## Citation

If you use RangedIK, please cite our ICRA paper: [*RangedIK: An Optimization-based Robot Motion Generation Method for Ranged-Goal Tasks*](https://arxiv.org/abs/2302.13935)
```
@article{wang2023rangedik,
  title={RangedIK: An Optimization-based Robot Motion Generation Method for Ranged-Goal Tasks},
  author={Wang, Yeping and Praveena, Pragathi and Rakita, Daniel and Gleicher, Michael},
  journal={arXiv preprint arXiv:2302.13935},
  year={2023}
}
```

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

