pub mod lib;
use crate::lib::relaxed_ik;
use crate::lib::utils_rust::subscriber_utils::EEPoseGoalsSubscriber;
use std::sync::{Arc, Mutex};
// use rosrust;
use nalgebra::{Vector3, UnitQuaternion, Quaternion};
use crate::lib::utils_rust::subscriber_utils::{*};

//// Added by HS ////
use std::io;
//// Added by HS ////

// mod msg {
//     rosrust::rosmsg_include!(relaxed_ik / EEPoseGoals, relaxed_ik / JointAngles);
// }

fn main() {
    // rosrust::init("relaxed_ik");

    println!("solver initialized!");

    //// Added by HS ////
    println!("Please enter the ee goal position: ");

    let mut v = relaxed_ik::EEPoseGoals::new();
    
    let mut pos_buf = String::new();
    io::stdin().read_line(&mut pos_buf).expect("Failed to read line");
    let pos_v: Vec<f64> = pos_buf.trim().split_whitespace().map(|x| x.parse::<f64>().unwrap()).collect();
    println!("{:?}", pos_v);

    println!("Please enter the ee goal orientation: ");
    let mut quat_buf = String::new();
    io::stdin().read_line(&mut quat_buf).expect("Failed to read line");
    let quat_v: Vec<f64> = quat_buf.trim().split_whitespace().map(|x| x.parse::<f64>().unwrap()).collect();
    println!("{:?}", quat_v);

    let pose = relaxed_ik::Pose::new(pos_v, quat_v);
    v.ee_poses.push(pose);
    //// Added by HS ////
    
    let mut r = relaxed_ik::RelaxedIK::from_loaded(1);

    let arc = Arc::new(Mutex::new(EEPoseGoalsSubscriber::new()));
    // let arc2 = arc.clone();
    // let subscriber = rosrust::subscribe("/relaxed_ik/ee_pose_goals", 3, move |v: msg::relaxed_ik::EEPoseGoals| {
    let mut g = arc.lock().unwrap();
    g.pos_goals = Vec::new();
    g.quat_goals = Vec::new();

    let num_poses = v.ee_poses.len();

    for i in 0..num_poses {
        g.pos_goals.push( Vector3::new(v.ee_poses[i].position.x, v.ee_poses[i].position.y, v.ee_poses[i].position.z) );
        let tmp_q = Quaternion::new(v.ee_poses[i].orientation.coords.w, v.ee_poses[i].orientation.coords.x, v.ee_poses[i].orientation.coords.y, v.ee_poses[i].orientation.coords.z);
        g.quat_goals.push( UnitQuaternion::from_quaternion(tmp_q) );
    }

    println!("{:?}", g.pos_goals);

    // let publisher = rosrust::publish("/relaxed_ik/joint_angle_solutions", 3).unwrap();

    // let rate1 = rosrust::rate(100.);
    // while arc.lock().unwrap().pos_goals.is_empty() {rate1.sleep();}

    // let rate = rosrust::rate(3000.);
    // while rosrust::is_ok() {
    // loop {
    // let x = r.solve(&arc.lock().unwrap());
    
    let x = r.solve(&g);
    println!("{:?}", x);

    // let mut ja = msg::relaxed_ik::JointAngles::default();
    let mut ja = relaxed_ik::JointAngles::new();
    for i in 0..x.len() {
        ja.data.push(x[i]);
    }
    // publisher.send(ja);

    // rate.sleep();
    // }
}