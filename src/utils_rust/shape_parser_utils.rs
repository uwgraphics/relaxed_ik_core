
#[derive(Clone, Debug)]
pub struct Cuboid {
    pub name: String,
    pub x_halflength: f64,
    pub y_halflength: f64,
    pub z_halflength: f64,
    pub coordinate_frame: String,
    pub rx: f64,
    pub ry: f64,
    pub rz: f64,
    pub tx: f64,
    pub ty: f64,
    pub tz: f64
}
impl Cuboid {
    pub fn new(name: String, x_halflength: f64, y_halflength: f64, z_halflength: f64, coordinate_frame: String,
        rx: f64, ry: f64, rz: f64, tx: f64, ty: f64, tz: f64) -> Self {
        Self {name, x_halflength, y_halflength, z_halflength, coordinate_frame, rx, ry, rz, tx, ty, tz}
    }
}

#[derive(Clone, Debug)]
pub struct Sphere {
    pub name: String,
    pub radius: f64,
    pub coordinate_frame: String,
    pub tx: f64,
    pub ty: f64,
    pub tz: f64
}
impl Sphere {
    pub fn new(name: String, radius: f64, coordinate_frame: String, tx: f64, ty: f64, tz: f64) -> Self {
        Self {name, radius, coordinate_frame, tx, ty, tz}
    }
}

#[derive(Clone, Debug)]
pub struct CuboidEnv {
    pub name: String,
    pub x_halflength: f64,
    pub y_halflength: f64,
    pub z_halflength: f64,
    pub rx: f64,
    pub ry: f64,
    pub rz: f64,
    pub tx: f64,
    pub ty: f64,
    pub tz: f64,
    pub is_dynamic: bool,
}
impl CuboidEnv {
    pub fn new(name: String, x_halflength: f64, y_halflength: f64, z_halflength: f64,
        rx: f64, ry: f64, rz: f64, tx: f64, ty: f64, tz: f64, is_dynamic: bool) -> Self {
        Self {name, x_halflength, y_halflength, z_halflength, rx, ry, rz, tx, ty, tz, is_dynamic}
    }
}

#[derive(Clone, Debug)]
pub struct SphereEnv {
    pub name: String,
    pub radius: f64,
    pub tx: f64,
    pub ty: f64,
    pub tz: f64,
    pub is_dynamic: bool
}
impl SphereEnv {
    pub fn new(name: String, radius: f64, tx: f64, ty: f64, tz: f64, is_dynamic: bool) -> Self {
        Self {name, radius, tx, ty, tz, is_dynamic}
    }
}

#[derive(Clone, Debug)]
pub struct PCEnv {
    pub name: String,
    pub rx: f64,
    pub ry: f64,
    pub rz: f64,
    pub tx: f64,
    pub ty: f64,
    pub tz: f64,
    pub is_dynamic: bool,
    pub points: Vec<SphereEnv>
}

impl PCEnv {
    pub fn new(name: String, rx: f64, ry: f64, rz: f64, tx: f64, ty: f64, tz: f64, is_dynamic: bool, points: Vec<SphereEnv>) -> Self {
        Self {name, rx, ry, rz, tx, ty, tz, is_dynamic, points}
    }
}