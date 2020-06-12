# relaxed_ik_core

## Run
* Enter the interactive mode by running: 
```
cargo run --bin relaxed_ik_node
```
The solver converges in expected time.

## Test
* Run the regression tests by running:
```
cargo test --test ur5_regression_test -- --nocapture
```
Hide output from test execution by removing `-- --nocapture` and simply running:
```
cargo test --test ur5_regression_test
```
* Test outputs will be generated at `relaxed_ik_core/tests/ur5/output`, and they are compared with expected outputs at `relaxed_ik_core/tests/ur5/expected` as our regression tests. 16 pairs of inputs are tested in total.
* The expected outputs are retrieved from `/rosout` when running relaxed_ik with the same inputs in ROS1
