extern crate relaxed_ik_core;
use relaxed_ik_core::relaxed_ik;
use relaxed_ik_core::utils_rust::subscriber_utils::EEPoseGoalsSubscriber;
use nalgebra::{Vector3, UnitQuaternion, Quaternion};
use std::fs;

#[test]
fn compute_single_ja_solution() {
    println!("\n==========Regression Test 1 with UR5 initialized!==========\n");

    let sol_num = 100;
    println!("Check if the first {} computed joint solutions are the same as expected.\n", sol_num);

    // 4 * 4, 16 position and quaternion pairs in total
    let pos_v: Vec<Vec<f64>> = vec![vec![0.015, 0.015, 0.015], vec![0.03, 0.02, -0.03], vec![-0.2, 0.2, 0.0], vec![0.0, 0.0, 0.5]];
    let quat_v: Vec<Vec<f64>> = vec![vec![0.0, 0.0, 0.0, 1.0], vec![-0.5, 1.0, 0.5, 1.0], vec![0.0, -1.0, 0.2, 0.2], vec![0.0, -0.1, -0.1, 0.0]];

    for i in 0..pos_v.len() {
        for j in 0..quat_v.len() {
            let index = i * pos_v.len() + j;
            
            println!("Test {} Input:\n\tGoal position: {:?}\n\tGoal orientation: {:?}", index, pos_v[i], quat_v[j]);
            
            let pose = relaxed_ik::Pose::new(pos_v[i].clone(), quat_v[j].clone());
            let mut v = relaxed_ik::EEPoseGoals::new();
            v.ee_poses.push(pose);
            
            let mut r = relaxed_ik::RelaxedIK::from_loaded(1);

            let mut g = EEPoseGoalsSubscriber::new();
            g.pos_goals = Vec::new();
            g.quat_goals = Vec::new();

            let num_poses = v.ee_poses.len();

            for i in 0..num_poses {
                g.pos_goals.push( Vector3::new(v.ee_poses[i].position.x, v.ee_poses[i].position.y, v.ee_poses[i].position.z) );
                let tmp_q = Quaternion::new(v.ee_poses[i].orientation.coords.w, v.ee_poses[i].orientation.coords.x, v.ee_poses[i].orientation.coords.y, v.ee_poses[i].orientation.coords.z);
                g.quat_goals.push( UnitQuaternion::from_quaternion(tmp_q) );
            }
        
            let mut ja = String::new();
            for _counter in 0..sol_num {
                let x = r.solve(&g);
                let s = format!("{:?}\n", x);
                ja.push_str(&s);
            }

            fs::write(format!("tests/ur5/output/p{}q{}.out", i, j), ja.as_str()).expect("Unable to write data");
            println!("Test {} Output: \n\tp{}q{}.out at ./output\n", index, i, j);

            let expected: String = fs::read_to_string(format!("tests/ur5/expected/p{}q{}_e.out", i, j)).expect("Unable to read data");
            
            assert_eq!(ja, expected);
        }
    }
}