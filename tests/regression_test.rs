extern crate relaxed_ik_lib;
use relaxed_ik_lib::relaxed_ik;
use relaxed_ik_lib::utils_rust::subscriber_utils::EEPoseGoalsSubscriber;
use nalgebra::{Vector3, UnitQuaternion, Quaternion};
use std::fs;
use std::sync::{Arc, Mutex};

pub struct PosQuatPair {
    pub pos_v: Vec<Vec<Vec<f64>>>,
    pub quat_v: Vec<Vec<Vec<f64>>>
}

impl PosQuatPair {
    pub fn data_parser(fp: String) -> Self {
        let file = fs::read_to_string(fp).expect("Unable to open file");
        let pos_buf = file.split('\n').nth(0).expect("Unable to split the string");
        let quat_buf = file.split('\n').nth(1).expect("Unable to split the string");
        
        let pos_v: Vec<Vec<Vec<f64>>> = pos_buf.trim().split(';')
            .map(|x| x.trim().split(',')
            .map(|y| y.trim_matches(|c| c == '[' || c == ']' || c == ' ').split(' ')
            .map(|z| z.parse::<f64>().unwrap()).collect()).collect()).collect();
        // println!("{:?}", pos_v);
        let quat_v: Vec<Vec<Vec<f64>>> = quat_buf.trim().split(';')
        .map(|x| x.trim().split(',')
        .map(|y| y.trim_matches(|c| c == '[' || c == ']' || c == ' ').split(' ')
        .map(|z| z.parse::<f64>().unwrap()).collect()).collect()).collect();
        // println!("{:?}", quat_v);
        Self{pos_v, quat_v}
    }
}

#[test]
fn compute_ja_solution() {
    let robot_list = vec!["ur5", "yumi", "panda", "iiwa7"];

    for r_num in 0..robot_list.len() {
        let r_cur = robot_list[r_num];
        println!("\n========== Regression Test On JA Solutions with {} initialized! ==========\n", r_cur);

        let sol_num = 100;
        println!("Check if the first {} computed joint solutions are the same as expected.\n", sol_num);

        let pair = PosQuatPair::data_parser(format!("tests/{}/input/{}.in", r_cur, r_cur));

        for i in 0..pair.pos_v.len() {
            for j in 0..pair.quat_v.len() {
                let mut r = relaxed_ik::RelaxedIK::from_info_file_name(format!("{}_info.yaml", r_cur), 1);

                let index = i * pair.pos_v.len() + j;
                println!("{} Test {} Input: ", r_cur, index);

                let arc = Arc::new(Mutex::new(EEPoseGoalsSubscriber::new()));
                let mut g = arc.lock().unwrap();
                
                for k in 0..r.vars.robot.num_chains {
                    println!("\tChain {} Goal position: {:?}\n\tChain {} Goal orientation: {:?}", k, pair.pos_v[i][k], k, pair.quat_v[j][k]);
                    let pos_goal = Vector3::new(pair.pos_v[i][k][0], pair.pos_v[i][k][1], pair.pos_v[i][k][2]);
                    g.pos_goals.push(pos_goal);
            
                    let tmp_q = Quaternion::new(pair.quat_v[j][k][3], pair.quat_v[j][k][0], pair.quat_v[j][k][1], pair.quat_v[j][k][2]);
                    let quat_goal = UnitQuaternion::from_quaternion(tmp_q);
                    g.quat_goals.push(quat_goal);
                }
            
                let mut ja = String::new();
                for _counter in 0..sol_num {
                    let x = r.solve(&g);
                    let s = format!("{:?}\n", x);
                    ja.push_str(&s);
                }

                fs::write(format!("tests/{}/output/p{}q{}.out", r_cur, i, j), ja.as_str()).expect("Unable to write data");
                println!("{} Test {} Output: \n\tp{}q{}.out at ./{}/output\n", r_cur, index, i, j, r_cur);

                let expected: String = fs::read_to_string(format!("tests/{}/expected/p{}q{}_e.out", r_cur, i, j)).expect("Unable to read data");
                assert_eq!(ja, expected);
            }
        }
    }
}
