use crate::relaxed_ik;
use crate::utils_rust::subscriber_utils::EEPoseGoalsSubscriber;
use std::sync::{Arc, Mutex};
use nalgebra::{Vector3, UnitQuaternion, Quaternion};
// use crate::utils_rust::subscriber_utils::{*};
use std::io::{self, Write};
use std::os::raw::{c_double, c_int};
use std::cell::RefCell;

#[repr(C)]
pub struct Opt {
    data: *const c_double,
    length: c_int,
}

#[no_mangle]
pub unsafe extern "C" fn run_solver(pos_arr: *const c_double, pos_length: c_int, 
    quat_arr: *const c_double, quat_length: c_int) -> Opt {
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

thread_local!(static R: RefCell<relaxed_ik::RelaxedIK> = RefCell::new(relaxed_ik::RelaxedIK::from_loaded(1)));

fn run(pos_goals: Vec<f64>, quat_goals: Vec<f64>) -> Vec<f64> {
    let mut x: Vec<f64> = Vec::new();

    let arc = Arc::new(Mutex::new(EEPoseGoalsSubscriber::new()));

    let mut g = arc.lock().unwrap();

    // while arc.lock().unwrap().pos_goals.is_empty() {}

    R.with(|r| {
        for i in 0..(*r.borrow()).vars.robot.num_chains {
            g.pos_goals.push( Vector3::new(pos_goals[3*i], pos_goals[3*i+1], pos_goals[3*i+2]) );
            let tmp_q = Quaternion::new(quat_goals[3*i], quat_goals[3*i+1], quat_goals[3*i+2], quat_goals[3*i+3]);
            g.quat_goals.push( UnitQuaternion::from_quaternion(tmp_q) );
        }

        // println!("pos: {:?}, quat: {:?}", g.pos_goals, g.quat_goals);
    
        x = (*r.borrow_mut()).solve(&g);
        println!("{:?}", x);
    });
    
    x
}