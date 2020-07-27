# relaxed_ik_core
## About
This is the core part of relaxed IK solver. Here is the project page that has more information about RelaxedIK that you may want to know about: https://uwgraphics.github.io/relaxed_ik_core/

RelaxedIK is an inverse kinematics (IK) solver designed for robot platforms such that the conversion between Cartesian end-effector pose goals (such as "move the robot's right arm end-effector to position X, while maintaining an end-effector orientation Y") to Joint-Space (i.e., the robot's rotation values for each joint degree-of-freedom at a particular time-point) is done both ACCURATELY and FEASIBLY.  By this, we mean that RelaxedIK attempts to find the closest possible solution to the desired end-effector pose goals without exhibiting negative effects such as self-collisions, environment collisions, kinematic-singularities, or joint-space discontinuities.

Usually you don't want to run this repo (although it is runnable, please refer to the run section), instead, you may want to work with one of the wrappers of RelaxedIK:
+ Wrapper in ROS: [relaxed_ik_ros1](https://github.com/uwgraphics/relaxed_ik_ros1)
+ Wrapper in ROS2 (Under developed): [relaxed_ik_ros2](https://github.com/uwgraphics/relaxed_ik_ros2)
+ Wrapper in Unity, Windows: [relaxed_ik_unity](https://github.com/uwgraphics/relaxed_ik_unity)

## Dependencies
To use relaxed_ik_core and all the wrappers that include relaxed_ik_core, you will first need to install Rust. Please go to https://www.rust-lang.org/learn/get-started for more infomation.

If you plan to extend any of the Rust code, we recommend using the Jetbrains rust plugin.

## Run
1. relaxed_ik_core is runnable on both linux and windows machines.
2. Enter the interactive mode by running: 
	```
	cargo run --bin relaxed_ik_bin
	```
3. Follow the prompt in the console to test pre-generated robot configuration files.
4. The joint angle solutions printed to the console should converge after a few seconds of running.

## Test
1. Run the regression tests by running:
	```
	cargo test --test regression_test -- --nocapture
	```
2. Hide output from test execution by removing `-- --nocapture` and simply running:
	```
	cargo test --test regression_test
	```
3. All available pre-computed robots will be tested automatically during the regression test
4. Test outputs will be generated at `relaxed_ik_core/tests/[robot_name]/output`, and they are compared with expected outputs at `relaxed_ik_core/tests/[robot_name]/expected`
5. Note that the expected outputs are retrieved from `/rosout` when running relaxed_ik with the same robot, configurations and inputs on ROS Melodic, Ubuntu 18.04. The tests may fail at a different operating system such as windows.

## Known Issues
1. The config files of these robot arms should be updated.