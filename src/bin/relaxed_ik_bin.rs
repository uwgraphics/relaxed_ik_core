extern crate relaxed_ik_lib;
use relaxed_ik_lib::relaxed_ik;
use relaxed_ik_lib::utils_rust::subscriber_utils::EEPoseGoalsSubscriber;
use std::sync::{Arc, Mutex};
use nalgebra::{Vector3, UnitQuaternion, Quaternion};

use std::{io, thread, time};

fn main() {
    println!("\nRelaxedIK initialized!\n");

    println!("Please enter the name of the robot: \n(Available options include baxter, hubo, hubo8, iiwa7, jaco7, panda, sawyer, ur5 and yumi.)");
    let mut name_buf = String::new();
    io::stdin().read_line(&mut name_buf).expect("Failed to read line");
    let name: String = name_buf.trim().to_string();

    let mut r = relaxed_ik::RelaxedIK::from_info_file_name(format!("{}_info.yaml", name), 1);

    let arc = Arc::new(Mutex::new(EEPoseGoalsSubscriber::new()));
    let mut g = arc.lock().unwrap();
    
    for i in 0..r.vars.robot.num_chains {
        println!("Chain {}: Please enter the ee goal position separated by white spaces (e.g., 0.015 0.015 0.015): ", i);
        let mut pos_buf = String::new();
        io::stdin().read_line(&mut pos_buf).expect("Failed to read line");
        let pos_v: Vec<f64> = pos_buf.trim().split_whitespace().map(|x| x.parse::<f64>().unwrap()).collect();
        let pos_goal = Vector3::new(pos_v[0], pos_v[1], pos_v[2]);
        // println!("{:?}", pos_goal);
        g.pos_goals.push(pos_goal);

        println!("Chain {}: Please enter the ee goal orientation separated by white spaces (e.g., 0.0 0.0 0.0 1.0): ", i);
        let mut quat_buf = String::new();
        io::stdin().read_line(&mut quat_buf).expect("Failed to read line");
        let quat_v: Vec<f64> = quat_buf.trim().split_whitespace().map(|x| x.parse::<f64>().unwrap()).collect();
        let tmp_q = Quaternion::new(quat_v[3], quat_v[0], quat_v[1], quat_v[2]);
        let quat_goal = UnitQuaternion::from_quaternion(tmp_q);
        // println!("{:?}", quat_goal);
        g.quat_goals.push(quat_goal);
    }

    let rate = time::Duration::from_millis(10);

    loop {
        let x = r.solve(&g);
        println!("{:?}", x);

        thread::sleep(rate);
    }
}
