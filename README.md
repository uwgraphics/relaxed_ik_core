# relaxed_ik_core
The core part of relaxed IK solver.

## Run
1. Enter the interactive mode by running: 
	```
	cargo run --bin relaxed_ik_bin
	```
2. Follow the prompt in the console to test a pre-computed robot
3. The joint angle solutions printed to the console should converge after a few seconds of running

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
5. The expected outputs are retrieved from `/rosout` when running relaxed_ik with the same robot, configurations and inputs in ROS1

## Known Issues
1. The config files of these robot arms should be updated.