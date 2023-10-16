use crate::spacetime::arm;
use nalgebra::{self, Point3, Vector3};
use urdf_rs;
use parry3d_f64::shape::{self, Shape};
use collada;

fn convert_ros_package_path(filename: &str) -> Option<String> {
    // Check if filename starts with "package://"
    if filename.starts_with("package://") {
        // Extract the package name and relative file path
        let parts: Vec<&str> = filename["package://".len()..].splitn(2, '/').collect();
        if parts.len() != 2 {
            eprintln!("Invalid package URI: {}", filename);
            return None;
        }
        let (package_name, relative_path) = (parts[0], parts[1]);

        // Retrieve ROS_PACKAGE_PATH from environment variables
        if let Ok(ros_package_path) = std::env::var("ROS_PACKAGE_PATH") {
            // Check if "catkin" is present in ROS_PACKAGE_PATH
            if ros_package_path.contains("catkin") {
                // Search for the absolute path to the package
                for path in ros_package_path.split(':') {
                    let possible_path = format!("{}/{}/{}", path, package_name, relative_path);
                    if std::path::Path::new(&possible_path).exists() {
                        return Some(possible_path);
                    }
                }
                println!("Package '{}' not found in ROS_PACKAGE_PATH.", package_name);
            } else {
                println!("'catkin' not found in ROS_PACKAGE_PATH.");
            }
        } else {
            println!("ROS_PACKAGE_PATH not set.");
        }
    } else {
        println!("Filename '{}' does not start with 'package://'.", filename);
    }
    None
}

fn load_dae_to_aabb(file_path: &str) -> Option<shape::SharedShape> {
    let doc = collada::document::ColladaDocument::from_path(std::path::Path::new(file_path)).ok()?;
    
    let mut all_vertices = Vec::new();
    let mut all_indices = Vec::new();

    if let Some(object_set) = doc.get_obj_set() {
        for object in object_set.objects {
            // Convert vertices to parry3d format
            let vertices: Vec<_> = object.vertices.iter().map(|vertex| {
                Point3::new(vertex.x, vertex.y, vertex.z) // Replace with actual field names
            }).collect();

            // Suppose `geometry` is one instance of `collada::Geometry` from object.geometry
            let mut indices = Vec::new();
            for primitive_element in object.geometry {
                for mesh in primitive_element.mesh {
                    match mesh {
                        collada::PrimitiveElement::Polylist(polylist) => {
                            println!("Polylist is not supported yet!")
                        },
                        collada::PrimitiveElement::Triangles(triangles) => {
                            // Append indices from triangles directly
                            // Again, you'll need to extract and perhaps re-arrange the indices 
                            // based on your actual data structure and requirements.
                            // Example: (Assuming triangles has a field 'indices' that is Vec<usize>)
                            for vertex in triangles.vertices {
                                // Ensure your index type matches (converting usize to u32 if safe)
                                let triangle = [
                                    vertex.0 as u32,
                                    vertex.1 as u32,
                                    vertex.2 as u32,
                                ];
                                indices.push(triangle);
                            }
                        },
                    }
                }
            }
            // Potentially add vertices and indices to all_vertices and all_indices
            // if multiple objects' geometries should be combined into a single TriMesh
            all_vertices.extend(vertices);
            all_indices.extend(indices);
        }
    };
    // println!("all_vertices: {:?}", all_vertices);
    // println!("all_indices: {:?}", all_indices);
    
    let trimesh = shape::TriMesh::new(all_vertices, all_indices);
    // Retrieve AABB of TriMesh
    let mesh_aabb = trimesh.compute_local_aabb();
    let half_extents = mesh_aabb.half_extents();
    Some(shape::SharedShape::cuboid(half_extents.x / 1000.0, half_extents.y / 1000.0, half_extents.z / 1000.0))
}

#[derive(Clone)]
pub struct Robot {
    pub arms: Vec<arm::Arm>,
    pub num_chains: usize,
    pub num_dofs: usize,
    pub chain_indices: Vec<Vec<usize>>,
    pub lower_joint_limits: Vec<f64>,
    pub upper_joint_limits: Vec<f64>,
    pub link_meshes: Vec<Vec<shape::SharedShape>>
}

impl Robot {
    pub fn from_urdf(urdf: &str, base_links: &[String], ee_links: &[String], joint_ordering: Option<Vec<String>>) -> Self {
        
        // let chain = k::Chain::<f64>::from_urdf_file(urdf_fp).unwrap();
        let description : urdf_rs::Robot = urdf_rs::read_from_string(urdf).unwrap();
        let chain: k::Chain<f64> = k::Chain::from(description.clone());

        let mut arms: Vec<arm::Arm> = Vec::new();
        let num_chains = base_links.len();
        let mut link_meshes = Vec::new();
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
            let mut chain_meshes = Vec::new();
            
            serial_chain.iter_links().for_each(|link| {
                let mut shapes_vec = Vec::new();
                link.collisions.iter().for_each(|c| {
                    let parry_pose = *c.origin();
                
                    let parry_shape = match &c.geometry {
                        k::link::Geometry::Box { depth, width, height } => {
                            let half_extents = Vector3::new(*depth / 2.0, *width / 2.0, *height / 2.0);
                            shape::SharedShape::cuboid(half_extents.x, half_extents.y, half_extents.z)
                        },
                        k::link::Geometry::Cylinder { radius, length } => {
                            shape::SharedShape::cylinder(*length / 2.0, *radius)
                        },
                        k::link::Geometry::Capsule { radius, length } => {
                            let half_length = *length / 2.0;
                            let a = Point3::new(0.0, -half_length, 0.0);
                            let b = Point3::new(0.0, half_length, 0.0);
                            shape::SharedShape::capsule(a, b, *radius)
                        },
                        k::link::Geometry::Sphere { radius } => {
                            shape::SharedShape::ball(*radius)
                        },
                        k::link::Geometry::Mesh { filename, scale } => {
                            let file_path = convert_ros_package_path(filename).unwrap();
                            println!("Loading {:?} from {:?}", filename, file_path);
                            load_dae_to_aabb(&file_path).unwrap()
                        },
                    };
                    // println!("parry_pose: {:?}", parry_pose);
                    // println!("parry_shape: {:?}", parry_shape.0.shape_type());
                    shapes_vec.push((parry_pose, parry_shape))
                });
                if !shapes_vec.is_empty() {
                    println!("link name: {:?}", link.name);
                    let compound_shape = shape::SharedShape::compound(shapes_vec);
                    chain_meshes.push(compound_shape);
                }
            });
            // println!("chain_meshes: {:?}", chain_meshes.len());
            link_meshes.push(chain_meshes);

            serial_chain.iter_joints().for_each(|joint| {
                println!("joint name: {:?}", joint.name);
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
        Robot{arms, num_chains, chain_indices, num_dofs, lower_joint_limits, upper_joint_limits, link_meshes}

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

