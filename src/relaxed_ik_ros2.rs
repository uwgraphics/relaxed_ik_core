use crate::relaxed_ik;
use crate::utils_rust::subscriber_utils::EEPoseGoalsSubscriber;
use std::sync::{Arc, Mutex};
use nalgebra::{Vector3, UnitQuaternion, Quaternion};
// use crate::utils_rust::subscriber_utils::{*};
use std::io::{self, Write};
use std::os::raw::{c_double, c_int};

#[repr(C)]
pub struct Opt {
    data: *const c_double,
    length: c_int,
}

#[no_mangle]
pub unsafe extern "C" fn rust_run(pos_arr: *const c_double, pos_length: c_int, 
    quat_arr: *const c_double, quat_length: c_int) -> Opt {
    println!("\nWrapper called!\n");
    io::stdout().flush().unwrap();
    assert!(!pos_arr.is_null(), "Null pointer for pos goals");
    assert!(!quat_arr.is_null(), "Null pointer for quat goals");

    let pos_slice: &[c_double] = std::slice::from_raw_parts(pos_arr, pos_length as usize);
    let quat_slice: &[c_double] = std::slice::from_raw_parts(quat_arr, quat_length as usize);

    let pos_vec = pos_slice.to_vec();
    let quat_vec = quat_slice.to_vec();

    let ja = run(pos_vec, quat_vec);
    let ptr = ja.as_ptr();
    let len = ja.len();
    std::mem::forget(ja);
    Opt {data: ptr, length: len as c_int}
}

fn run(pos_goals: Vec<f64>, quat_goals: Vec<f64>) -> Vec<f64> {
    println!("\nRunner called!\n");

    let mut r = relaxed_ik::RelaxedIK::from_loaded(1);

    let mut v = relaxed_ik::EEPoseGoals::new();

    for i in 0..r.vars.robot.num_chains {   
        let pos_v: Vec<f64> = (&pos_goals[3*i..3*i+3]).to_vec();
        println!("pos: {:?}", pos_v);

        let quat_v: Vec<f64> = (&quat_goals[3*i..3*i+4]).to_vec();
        println!("quat: {:?}", quat_v);

        let pose = relaxed_ik::Pose::new(pos_v, quat_v);
        v.ee_poses.push(pose);
        io::stdout().flush().unwrap();
    }

    let arc = Arc::new(Mutex::new(EEPoseGoalsSubscriber::new()));
    // let arc2 = arc.clone();

    let mut g = arc.lock().unwrap();
    g.pos_goals = Vec::new();
    g.quat_goals = Vec::new();

    let num_poses = v.ee_poses.len();

    for i in 0..num_poses {
        g.pos_goals.push( Vector3::new(v.ee_poses[i].position.x, v.ee_poses[i].position.y, v.ee_poses[i].position.z) );
        let tmp_q = Quaternion::new(v.ee_poses[i].orientation.coords.w, v.ee_poses[i].orientation.coords.x, v.ee_poses[i].orientation.coords.y, v.ee_poses[i].orientation.coords.z);
        g.quat_goals.push( UnitQuaternion::from_quaternion(tmp_q) );
    }

    let x = r.solve(&g);
    println!("{:?}", x);
    x
}