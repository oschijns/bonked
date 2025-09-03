//! Program to convert 3D files

/// Convert wavefront obj format to bonked format
mod convert_obj;

/// Convert GLTF into bonked format
mod convert_gltf;

use ::gltf::{Glb, Gltf};
use ::obj::{Obj, load_obj};
use clap::{Parser, ValueEnum};
use std::{
    error::Error,
    fs::File,
    io::BufReader,
    path::{Path, PathBuf},
    process,
};

/// Read a common 3D file and convert it into bonked specific format
#[derive(Debug, Parser)]
#[command(version, about, long_about = None)]
struct Args {
    /// The input file to convert (wavefront .obj or GLTF)
    #[arg(short, long)]
    input: PathBuf,

    /// Endianness of the target system
    #[arg(short, long)]
    endian: Endianness,
}

/// Target architecture endianness
#[derive(Debug, Clone, Copy, ValueEnum)]
enum Endianness {
    /// little endian
    Little,

    /// big endian
    Big,
}

fn main() {
    let args = Args::parse();

    // identify the input file
    if let Some(e) = args.input.extension() {
        let e = e.to_ascii_lowercase();
        let ext = e.to_string_lossy();
        match ext.as_ref() {
            "obj" => match open_obj(&args.input) {
                Ok(obj) => {}
                Err(err) => {
                    eprintln!("Failed to load .obj file: {}", err);
                    process::exit(10)
                }
            },
            "gltf" => match Gltf::open(&args.input) {
                Ok(gltf) => {}
                Err(err) => {
                    eprintln!("Failed to load .gltf file: {}", err);
                    process::exit(11)
                }
            },
            "glb" => match open_glb(&args.input) {
                Ok(glb) => {}
                Err(err) => {
                    eprintln!("Failed to load .glb file: {}", err);
                    process::exit(12)
                }
            },
            _ => {
                eprintln!(
                    "Could not identify file type based on extension \".{}\"",
                    ext
                );
                process::exit(2)
            }
        }
    } else {
        eprintln!(
            "No extension provided in file \"{}\"",
            args.input.to_string_lossy()
        );
        process::exit(1)
    }
}

/// helper function to open .obj file
fn open_obj(path: &Path) -> Result<Obj, Box<dyn Error>> {
    let input = BufReader::new(File::open(path)?);
    let obj = load_obj(input)?;
    Ok(obj)
}

/// helper function to open .glb file
fn open_glb(path: &Path) -> Result<Glb, Box<dyn Error>> {
    let input = BufReader::new(File::open(path)?);
    let glb = Glb::from_reader(input)?;
    Ok(glb)
}
