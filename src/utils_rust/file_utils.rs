use std::env;
use std::fs::File;
use std::io::prelude::*;
use std::fs::read_dir;
use path_slash::PathBufExt;

pub fn get_path_to_src() -> String {
    let path = env::current_dir().unwrap();
    let s = path.to_slash().unwrap();
    let s1 = String::from(s);
    let path_to_src = s1 + "/";
    path_to_src
}