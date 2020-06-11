extern crate relaxed_ik_core;
use relaxed_ik_core::lib::relaxed_ik;
use relaxed_ik_core::lib::utils_rust::subscriber_utils::EEPoseGoalsSubscriber;
use std::sync::{Arc, Mutex};
// use rosrust;
use nalgebra::{Vector3, UnitQuaternion, Quaternion};
use relaxed_ik_core::lib::utils_rust::subscriber_utils::{*};

#[test]
fn compute_single_ja_solution() {
    println!("Test 1 initialized!");

    // println!("Please enter the ee goal position: ");

    let mut v = relaxed_ik::EEPoseGoals::new();
    let pos_v: Vec<f64> = vec![0.015, 0.015, 0.015];
    let quat_v: Vec<f64> = vec![1.0, 0.0, 0.0, 0.0];
    let pose = relaxed_ik::Pose::new(pos_v, quat_v);
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

    println!("{:?}", g.pos_goals);
    println!("{:?}", g.quat_goals);
    
    let x = r.solve(&g);
    println!("{:?}", x);
}