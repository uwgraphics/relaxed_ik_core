use crate::spacetime::arm;
use nalgebra;
use urdf_rs;

#[derive(Clone, Debug)]
pub struct Robot {
    pub arms: Vec<arm::Arm>,
    pub num_chains: usize,
    pub num_dofs: usize,
    pub chain_indices: Vec<Vec<usize>>,
    pub lower_joint_limits: Vec<f64>,
    pub upper_joint_limits: Vec<f64>
}

impl Robot {
    pub fn from_urdf(urdf: &str, base_links: &[String], ee_links: &[String], joint_ordering: Option<Vec<String>>) -> Self {
        
        // let chain = k::Chain::<f64>::from_urdf_file(urdf_fp).unwrap();
        let description : urdf_rs::Robot = urdf_rs::read_from_string(urdf).unwrap();
        let chain: k::Chain<f64> = k::Chain::from(description.clone());

        let mut arms: Vec<arm::Arm> = Vec::new();
        let num_chains = base_links.len();
        let mut chain_indices = Vec::new();
        let mut joint_names = Vec::new();
        let mut num_dofs = 0;

        let mut lower_joint_limits = Vec::new();
        let mut upper_joint_limits = Vec::new();

        for i in 0..num_chains {
            let base_link = chain.find_link(base_links[i].as_str()).unwrap();
            let ee_link = chain.find_link(ee_links[i].as_str()).unwrap();
            let serial_chain = k::SerialChain::from_end_to_root(&ee_link, &base_link);

            let mut axis_types: Vec<String> = Vec::new();
            let mut joint_types: Vec<String> = Vec::new();
            let disp_offset = nalgebra::Vector3::new(0.0, 0.0, 0.0);
            let mut displacements = Vec::new();
            let mut rot_offsets = Vec::new();
            let mut joint_indices = Vec::new();
            let mut articulated_joint_index = 0;

            serial_chain.iter().for_each(|node| {
                let joint = node.joint();
                match joint.joint_type {
                    k::JointType::Fixed => {
                        joint_types.push("fixed".to_string());
                    },
                    k::JointType::Rotational { axis } => {
                        if axis[0] == 1.0 {
                            axis_types.push("x".to_string());
                        } else if axis[1] == 1.0 {
                            axis_types.push("y".to_string());
                        } else if axis[2] == 1.0 {
                            axis_types.push("z".to_string());
                        } else if axis[0] == -1.0 {
                            axis_types.push("-x".to_string());
                        } else if axis[1] == -1.0 {
                            axis_types.push("-y".to_string());
                        } else if axis[2] == -1.0 {
                            axis_types.push("-z".to_string());
                        }
                        joint_types.push("revolute".to_string());
                        joint_names.push(joint.name.clone());
                        lower_joint_limits.push(joint.limits.unwrap().min);
                        upper_joint_limits.push(joint.limits.unwrap().max);
                        if let Some(ordering) = &joint_ordering {
                            if let Some(joint_index) = ordering.iter().position(|s| *s == joint.name) {
                                joint_indices.push(joint_index);
                            } else {
                                println!("Warning: joint {} not found in joint_ordering provided!", joint.name)
                            }
                        } else {
                            joint_indices.push(articulated_joint_index);
                            articulated_joint_index += 1;
                        }
                    }
                    k::JointType::Linear { axis } => {
                        if axis[0] == 1.0 {
                            axis_types.push("x".to_string());
                        } else if axis[1] == 1.0 {
                            axis_types.push("y".to_string());
                        } else if axis[2] == 1.0 {
                            axis_types.push("z".to_string());
                        } else if axis[0] == -1.0 {
                            axis_types.push("-x".to_string());
                        } else if axis[1] == -1.0 {
                            axis_types.push("-y".to_string());
                        } else if axis[2] == -1.0 {
                            axis_types.push("-z".to_string());
                        }
                        joint_types.push("prismatic".to_string());
                        joint_names.push(joint.name.clone());
                        lower_joint_limits.push(joint.limits.unwrap().min);
                        upper_joint_limits.push(joint.limits.unwrap().max);
                        if let Some(ordering) = &joint_ordering {
                            if let Some(joint_index) = ordering.iter().position(|s| *s == joint.name) {
                                joint_indices.push(joint_index);
                            } else {
                                println!("Warning: joint {} not found in joint_ordering provided!", joint.name)
                            }
                        } else {
                            joint_indices.push(articulated_joint_index);
                            articulated_joint_index += 1;
                        }
                    }
                }

                displacements.push(joint.origin().translation.vector);
                rot_offsets.push(joint.origin().rotation);
            });
            let arm: arm::Arm = arm::Arm::init(axis_types.clone(), displacements.clone(),
            rot_offsets.clone(), joint_types.clone());
            arms.push(arm);
            chain_indices.push(joint_indices);
            num_dofs += axis_types.len();
        }

        // Update the number of dofs if joint ordering is provided
        if let Some(ordering) = joint_ordering {
            num_dofs = ordering.len();
            let mut lower_joint_limits_new = Vec::new();
            let mut upper_joint_limits_new = Vec::new();
            ordering.iter().for_each(|name| {
                if let Some(joint_index) = joint_names.iter().position(|s| *s == *name) {
                    lower_joint_limits_new.push(lower_joint_limits[joint_index]);
                    upper_joint_limits_new.push(upper_joint_limits[joint_index]);
                }
            });
            lower_joint_limits = lower_joint_limits_new;
            upper_joint_limits = upper_joint_limits_new;   
        }

        // println!("axis types: {:?}", arms[0].axis_types);
        Robot{arms, num_chains, chain_indices, num_dofs, lower_joint_limits, upper_joint_limits}

    }

    pub fn get_frames(&mut self, x: &[f64]) {
        for i in 0..self.num_chains {
            let chain_values: Vec<f64> = self.chain_indices[i].iter().map(|&i| x[i]).collect();
            self.arms[i].get_frames(chain_values.as_slice());
        }
    }

    pub fn get_frames_immutable(&self, x: &[f64]) -> Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)> {
        let mut out: Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)> = Vec::new();
        for i in 0..self.num_chains {
            let chain_values: Vec<f64> = self.chain_indices[i].iter().map(|&i| x[i]).collect();
            out.push( self.arms[i].get_frames_immutable(chain_values.as_slice()) );
        }
        out
    }
    
    pub fn get_manipulability_immutable(&self, x: &[f64]) -> f64 {
        let mut out = 0.0;
        for i in 0..self.num_chains {
            let chain_values: Vec<f64> = self.chain_indices[i].iter().map(|&i| x[i]).collect();
            out += self.arms[i].get_manipulability_immutable( chain_values.as_slice() );
        }
        out
    }

    pub fn get_ee_pos_and_quat_immutable(&self, x: &[f64]) -> Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)> {
        let mut out: Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)> = Vec::new();
        for i in 0..self.num_chains {
            let chain_values: Vec<f64> = self.chain_indices[i].iter().map(|&i| x[i]).collect();
            out.push( self.arms[i].get_ee_pos_and_quat_immutable( chain_values.as_slice() ));
        }
        out
    }
}

