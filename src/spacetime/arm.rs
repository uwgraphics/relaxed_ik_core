use nalgebra;
use nalgebra::{Vector3, Vector6, UnitQuaternion, Unit, Matrix, DMatrix, DVector, ArrayStorage};

#[derive(Clone, Debug)]
pub struct Arm {
    pub axis_types: Vec<String>,
    pub displacements: Vec<nalgebra::Vector3<f64>>,
    pub rot_offset_quats: Vec<nalgebra::UnitQuaternion<f64>>,
    pub joint_types: Vec<String>,
    pub num_dof: usize,
    pub out_positions: Vec<nalgebra::Vector3<f64>>,
    pub out_rot_quats: Vec<nalgebra::UnitQuaternion<f64>>,
    __do_rot_offset: Vec<bool>,
    __is_prismatic: Vec<bool>,
    __is_revolute_or_continuous: Vec<bool>,
    __is_fixed: Vec<bool>,
    __is_x: Vec<bool>,
    __is_y: Vec<bool>,
    __is_z: Vec<bool>,
    __is_neg_x: Vec<bool>,
    __is_neg_y: Vec<bool>,
    __is_neg_z: Vec<bool>,
    __aux_matrix: nalgebra::Matrix3<f64>
}

impl Arm{
    pub fn init(axis_types: Vec<String>,
        disp_offsets: Vec<nalgebra::Vector3<f64>>,
        rot_offsets: Vec<UnitQuaternion<f64>>, joint_types: Vec<String>) -> Arm {

        let num_dof = axis_types.len();

        let mut __do_rot_offset: Vec<bool> = Vec::new();
        for i in 0..rot_offsets.len() {
            if rot_offsets[i][0] == 0.0 && rot_offsets[i][1] == 0.0 && rot_offsets[i][2] == 0.0 {
                __do_rot_offset.push(false);
            } else {
                __do_rot_offset.push(true);
            }
        }

        let mut displacements: Vec<nalgebra::Vector3<f64>> = Vec::new();
        for i in 0..disp_offsets.len() {
            displacements.push(disp_offsets[i].clone());
        }

        let mut rot_offset_quats: Vec<nalgebra::UnitQuaternion<f64>> = Vec::new();
        for i in 0..rot_offsets.len() {
            rot_offset_quats.push(rot_offsets[i]);
        }

        let mut out_positions: Vec<nalgebra::Vector3<f64>> = Vec::new();
        let mut out_rot_quats: Vec<nalgebra::UnitQuaternion<f64>> = Vec::new();
        for i in 0..rot_offsets.len() {
            out_positions.push(nalgebra::Vector3::new(0.,0.,0.));
            out_rot_quats.push(nalgebra::UnitQuaternion::identity());
        }

        let mut __is_prismatic: Vec<bool> = Vec::new();
        let mut __is_revolute_or_continuous: Vec<bool> = Vec::new();
        let mut __is_fixed: Vec<bool> = Vec::new();
        for i in 0..joint_types.len() {
            if joint_types[i] == String::from("prismatic") {
                __is_prismatic.push(true);
                __is_revolute_or_continuous.push(false);
                __is_fixed.push(false);
            }
            else if joint_types[i] == String::from("continuous") || joint_types[i] == String::from("revolute") {
                __is_prismatic.push(false);
                __is_revolute_or_continuous.push(true);
                __is_fixed.push(false);
            }
            else if joint_types[i] == String::from("fixed")  {
                __is_prismatic.push(false);
                __is_revolute_or_continuous.push(false);
                __is_fixed.push(true);
            }
        }

        let __aux_matrix: nalgebra::Matrix3<f64> = nalgebra::Matrix3::identity();

        let mut __is_x: Vec<bool> = Vec::new();
        let mut __is_y: Vec<bool> = Vec::new();
        let mut __is_z: Vec<bool> = Vec::new();
        let mut __is_neg_x: Vec<bool> = Vec::new();
        let mut __is_neg_y: Vec<bool> = Vec::new();
        let mut __is_neg_z: Vec<bool> = Vec::new();
        for i in 0..axis_types.len() {
            __is_x.push(false);
            __is_y.push(false);
            __is_z.push(false);
            __is_neg_x.push(false);
            __is_neg_y.push(false);
            __is_neg_z.push(false);
            if axis_types[i] == String::from("X") || axis_types[i] == String::from("x") {
                __is_x[i] = true;
            }
            else if axis_types[i] == String::from("X") || axis_types[i] == String::from("x") {
                __is_x[i] = true;
            }
            else if axis_types[i] == String::from("Y") || axis_types[i] == String::from("y") {
                __is_y[i] = true;
            }
            else if axis_types[i] == String::from("Z") || axis_types[i] == String::from("z") {
                __is_z[i] = true;
            }
            else if axis_types[i] == String::from("-x"){
                __is_neg_x[i] = true;
            }
            else if axis_types[i] == String::from("-y"){
                __is_neg_y[i] = true;
            }
            else if axis_types[i] == String::from("-z"){
                __is_neg_z[i] = true;
            }
        }

        // println!("displacements: {:?}", displacements);
        // println!("axis_types: {:?}", axis_types);
        // println!("__is_revolute_or_continuous: {:?}", __is_revolute_or_continuous);
        // println!("__do_rot_offset: {:?}", __do_rot_offset);
        // println!("rot_offset_quats: {:?}", rot_offset_quats);
        // println!("joint_types: {:?}", joint_types);
        Arm{axis_types, displacements, rot_offset_quats,
            joint_types, num_dof, out_positions, out_rot_quats, __do_rot_offset, __is_prismatic,
            __is_revolute_or_continuous, __is_fixed, __is_x, __is_y, __is_z, __is_neg_x, __is_neg_y,
            __is_neg_z, __aux_matrix}
    }

    pub fn get_frames_immutable(&self, x: &[f64]) -> (Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>) {
        let mut out_positions: Vec<nalgebra::Vector3<f64>> = Vec::new();
        let mut out_rot_quats: Vec<nalgebra::UnitQuaternion<f64>> = Vec::new();

        let mut pt: nalgebra::Vector3<f64> = nalgebra::Vector3::new(0.,0.,0.);
        let mut rot_quat = nalgebra::UnitQuaternion::identity();
       
        out_positions.push(pt);
        out_rot_quats.push(rot_quat);

        let mut joint_idx: usize = 0;
        for i in 0..self.displacements.len() {
            if self.__is_revolute_or_continuous[i] {

                pt = rot_quat * self.displacements[i] + pt;

                if self.__do_rot_offset[i] {
                    rot_quat = rot_quat * self.rot_offset_quats[i];
                }

                let joint_val = x[joint_idx];
                if self.__is_x[joint_idx] {
                    rot_quat = rot_quat * get_quat_x(joint_val);
                } else if self.__is_y[joint_idx] {
                    rot_quat = rot_quat * get_quat_y(joint_val);
                } else if self.__is_z[joint_idx] {
                    rot_quat = rot_quat * get_quat_z(joint_val);
                } else if self.__is_neg_x[joint_idx] {
                    rot_quat = rot_quat * get_quat_x(-joint_val);
                } else if self.__is_neg_y[joint_idx] {
                    rot_quat = rot_quat * get_quat_y(-joint_val);
                } else if self.__is_neg_z[joint_idx] {
                    rot_quat = rot_quat * get_quat_z(-joint_val);
                }

                out_positions.push(pt.clone());
                out_rot_quats.push(rot_quat.clone());

                joint_idx += 1;
            }
            else if self.__is_prismatic[i] {
                let joint_val = x[joint_idx];
                if self.__is_x[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(joint_val, 0., 0.);
                } else if self.__is_y[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(0., joint_val, 0.);
                } else if self.__is_z[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(0., 0., joint_val);
                } else if self.__is_neg_x[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(-joint_val, 0., 0.);
                } else if self.__is_neg_y[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(0., -joint_val, 0.);
                } else if self.__is_neg_z[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(0., 0., -joint_val);
                }
                if self.__do_rot_offset[i] {
                    rot_quat = rot_quat * self.rot_offset_quats[i];
                }
                out_positions.push(pt.clone());
                out_rot_quats.push(rot_quat.clone());
                joint_idx += 1;
            }
            else {
                pt = rot_quat * self.displacements[i] + pt;
                if self.__do_rot_offset[i] {
                    rot_quat = rot_quat * self.rot_offset_quats[i];
                }
                out_positions.push(pt.clone());
                out_rot_quats.push(rot_quat.clone());
            }
        }
        out_rot_quats.push(rot_quat.clone());

        (out_positions, out_rot_quats)
    }

    pub fn get_jacobian_immutable(&self, x: &[f64]) -> DMatrix<f64> {
        let (joint_positions, joint_rot_quats) = self.get_frames_immutable(x);

        let ee_position = joint_positions[joint_positions.len()-1];
        let pos_x: nalgebra::Vector3<f64> = nalgebra::Vector3::new(1.0, 0.0, 0.0);
        let pos_y: nalgebra::Vector3<f64> = nalgebra::Vector3::new(0.0, 1.0, 0.0);
        let pos_z: nalgebra::Vector3<f64> = nalgebra::Vector3::new(0.0, 0.0, 1.0);
        let neg_x: nalgebra::Vector3<f64> = nalgebra::Vector3::new(-1.0, 0.0, 0.0);
        let neg_y: nalgebra::Vector3<f64> = nalgebra::Vector3::new(0.0, -1.0, 0.0);
        let neg_z: nalgebra::Vector3<f64> = nalgebra::Vector3::new(0.0, 0.0, -1.0);

        let mut disp: Vector3<f64> = Vector3::new(0.0, 0.0, 0.0);
        let mut p_axis: Vector3<f64> = Vector3::new(0.0, 0.0, 0.0);
        let mut joint_idx: usize = 0;

        let mut jacobian: DMatrix<f64> = DMatrix::identity(6, x.len());

        for i in 1..self.displacements.len() {
            if self.__is_revolute_or_continuous[i-1] {
                disp = ee_position - joint_positions[i];
                if self.__is_x[joint_idx] {
                    p_axis = joint_rot_quats[i] * pos_x
                } else if self.__is_y[joint_idx] {
                    p_axis = joint_rot_quats[i] * pos_y
                } else if self.__is_z[joint_idx] {
                    p_axis = joint_rot_quats[i] * pos_z
                } else if self.__is_neg_x[joint_idx] {
                    p_axis = joint_rot_quats[i] * neg_x
                } else if self.__is_neg_y[joint_idx] {
                    p_axis = joint_rot_quats[i] * neg_y
                } else if self.__is_neg_z[joint_idx] {
                    p_axis = joint_rot_quats[i] * neg_z
                } 

                let linear = p_axis.cross(&disp);
                jacobian.set_column(joint_idx, & Vector6::new( linear.x, linear.y, linear.z,
                                                                    p_axis.x, p_axis.y, p_axis.z ));

                joint_idx += 1;
            }
        }

        jacobian
    }

    pub fn get_manipulability_immutable(&self, x: &[f64]) -> f64 {
        let jacobian = self.get_jacobian_immutable(x);
        (jacobian.clone() * jacobian.transpose()).determinant().sqrt()
    }

    pub fn get_ee_pos_and_quat_immutable(&self, x: &[f64]) -> (nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>) {
        let mut pt: nalgebra::Vector3<f64> = nalgebra::Vector3::new(0.,0.,0.);
        let mut rot_quat = nalgebra::UnitQuaternion::identity();

        let mut joint_idx: usize = 0;
        for i in 0..self.displacements.len() {
            if self.__is_revolute_or_continuous[i] {
                pt = rot_quat * self.displacements[i] + pt;

                if self.__do_rot_offset[i] {
                    rot_quat = rot_quat * self.rot_offset_quats[i];
                }

                let joint_val = x[joint_idx];
                if self.__is_x[joint_idx] {
                    rot_quat = rot_quat * get_quat_x(joint_val);
                } else if self.__is_y[joint_idx] {
                    rot_quat = rot_quat * get_quat_y(joint_val);
                } else if self.__is_z[joint_idx] {
                    rot_quat = rot_quat * get_quat_z(joint_val);
                } else if self.__is_neg_x[joint_idx] {
                    rot_quat = rot_quat * get_quat_x(-joint_val);
                } else if self.__is_neg_y[joint_idx] {
                    rot_quat = rot_quat * get_quat_y(-joint_val);
                } else if self.__is_neg_z[joint_idx] {
                    rot_quat = rot_quat * get_quat_z(-joint_val);
                }

                joint_idx += 1;
            }
            else if self.__is_prismatic[i] {
                let joint_val = x[joint_idx];
                if self.__is_x[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(joint_val, 0., 0.);
                } else if self.__is_y[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(0., joint_val, 0.);
                } else if self.__is_z[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(0., 0., joint_val);
                } else if self.__is_neg_x[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(-joint_val, 0., 0.);
                } else if self.__is_neg_y[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(0., -joint_val, 0.);
                } else if self.__is_neg_z[joint_idx] {
                    pt = rot_quat * self.displacements[i] + pt + nalgebra::Vector3::new(0., 0., -joint_val);
                }
                if self.__do_rot_offset[i] {
                    rot_quat = rot_quat * self.rot_offset_quats[i];
                }
                joint_idx += 1;
            }
            else {
                pt = rot_quat * self.displacements[i] + pt;
                if self.__do_rot_offset[i] {
                    rot_quat = rot_quat * self.rot_offset_quats[i];
                }
            }
        }

        (pt, rot_quat)
    }
}

pub fn get_rot_x(val: f64) -> nalgebra::Matrix3<f64> {
    nalgebra::Matrix3::new(1., 0., 0., 0., val.cos(), -val.sin(), 0.0, val.sin(), val.cos())
}

pub fn get_rot_y(val: f64) -> nalgebra::Matrix3<f64> {
    nalgebra::Matrix3::new(val.cos(), 0.0, val.sin(), 0., 1., 0., -val.sin(), 0., val.cos())
}

pub fn get_rot_z(val: f64) -> nalgebra::Matrix3<f64> {
    nalgebra::Matrix3::new(val.cos(), -val.sin(), 0., val.sin(), val.cos(), 0., 0., 0., 1.)
}

pub fn get_neg_rot_x(val: f64) -> nalgebra::Matrix3<f64> {
    get_rot_x(-val)
}

pub fn get_neg_rot_y(val: f64) -> nalgebra::Matrix3<f64> {
    get_rot_y(-val)
}

pub fn get_neg_rot_z(val: f64) -> nalgebra::Matrix3<f64> {
    get_rot_z(-val)
}

pub fn get_quat_x(val: f64) -> nalgebra::UnitQuaternion<f64> {
    UnitQuaternion::from_euler_angles(val, 0., 0.)
}

pub fn get_quat_y(val: f64) -> nalgebra::UnitQuaternion<f64> {
    UnitQuaternion::from_euler_angles(0., val, 0.)
}

pub fn get_quat_z(val: f64) -> nalgebra::UnitQuaternion<f64> {
    UnitQuaternion::from_euler_angles(0., 0., val)
}

pub fn get_neg_quat_x(val: f64) -> nalgebra::UnitQuaternion<f64> {
    get_quat_x(-val)
}

pub fn get_neg_quat_y(val: f64) -> nalgebra::UnitQuaternion<f64> {
    get_quat_y(-val)
}

pub fn get_neg_quat_z(val: f64) -> nalgebra::UnitQuaternion<f64> {
    get_quat_z(-val)
}

pub fn euler_triple_to_3x3(t: &Vec<f64>) -> nalgebra::Matrix3<f64>{
    let xm = get_rot_x(t[0]);
    let ym = get_rot_y(t[1]);
    let zm = get_rot_z(t[2]);

    let zy = zm*ym;
    zy*xm
}
