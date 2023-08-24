use crate::spacetime::arm;
use nalgebra;
use urdf_rs;

#[derive(Clone, Debug)]
pub struct Robot {
    pub arms: Vec<arm::Arm>,
    pub num_chains: usize,
    pub num_dofs: usize,
    pub chain_lengths: Vec<usize>,
    pub lower_joint_limits: Vec<f64>,
    pub upper_joint_limits: Vec<f64>
}

impl Robot {
    pub fn from_urdf(urdf: &str, base_links: &[String], ee_links: &[String]) -> Self {
        
        // let chain = k::Chain::<f64>::from_urdf_file(urdf).unwrap();
        let description : urdf_rs::Robot = urdf_rs::read_from_string(urdf).unwrap();
        let chain: k::Chain<f64> = k::Chain::from(description.clone());

        let mut arms: Vec<arm::Arm> = Vec::new();
        let num_chains = base_links.len();
        let mut chain_lengths = Vec::new();
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

            let mut first_link: bool = true;
            serial_chain.iter().for_each(|node| {
                let joint = node.joint();
                if first_link {
                    first_link = false;
                    return
                } else {
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
                            if joint.limits.is_none() {
                                joint_types.push("continuous".to_string());
                                lower_joint_limits.push(-999.0);
                                upper_joint_limits.push(999.0);
                            } else {
                                joint_types.push("revolute".to_string());
                                lower_joint_limits.push(joint.limits.unwrap().min);
                                upper_joint_limits.push(joint.limits.unwrap().max);
                            }
                        },
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
                            lower_joint_limits.push(joint.limits.unwrap().min);
                            upper_joint_limits.push(joint.limits.unwrap().max);
                        }
                    }
                }

                displacements.push(joint.origin().translation.vector);
                rot_offsets.push(joint.origin().rotation);
            });
            let arm: arm::Arm = arm::Arm::init(axis_types.clone(), displacements.clone(),
            rot_offsets.clone(), joint_types.clone());
            arms.push(arm);
            chain_lengths.push(axis_types.len() as usize);
            num_dofs += axis_types.len();
        }
        Robot{arms, num_chains, chain_lengths, num_dofs, lower_joint_limits, upper_joint_limits}

    }

    pub fn get_frames_immutable(&self, x: &[f64]) -> Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)> {
        let mut out: Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)> = Vec::new();
        let mut l = 0;
        let mut r = 0;
        for i in 0..self.num_chains {
            r += self.chain_lengths[i];
            out.push( self.arms[i].get_frames_immutable( &x[l..r] ) );
            l = r;
        }
        out
    }
    
    pub fn get_manipulability_immutable(&self, x: &[f64]) -> f64 {
        let mut out = 0.0;
        let mut l = 0;
        let mut r = 0;
        for i in 0..self.num_chains {
            r += self.chain_lengths[i];
            out += self.arms[i].get_manipulability_immutable( &x[l..r] );
            l = r;
        }
        out
    }

    pub fn get_ee_pos_and_quat_immutable(&self, x: &[f64]) -> Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)> {
        let mut out: Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)> = Vec::new();
        let mut l = 0;
        let mut r = 0;
        for i in 0..self.num_chains {
            r += self.chain_lengths[i];
            out.push( self.arms[i].get_ee_pos_and_quat_immutable( &x[l..r] ));
            l = r;
        }
        out
    }
}

