### Run
* Enter the interactive mode by running: 
```
cargo run --bin relaxed_ik_node
```
* Then follow the prompt in the console to test a pre-computed robot.
* The solver converges in expected time based on observation.

### Test
* Run the regression tests by running:
```
cargo test --test regression_test -- --nocapture
```
Hide output from test execution by removing `-- --nocapture` and simply running:
```
cargo test --test regression_test
```
* All available pre-computed robots will be tested automatically during the regression test.
* Test outputs will be generated at `relaxed_ik_core/tests/[robot_name]/output`, and they are compared with expected outputs at `relaxed_ik_core/tests/[robot_name]/expected`.
* The expected outputs are retrieved from `/rosout` when running relaxed_ik with the same robot, configurations and inputs in ROS1