//! Convert 3D file format to bonked format

use bonked3d_level::level::{Mesh, decomp::ConvertError};
use gltf::Gltf;

/// Convert gltf file into level part
pub fn convert_gltf(gltf: &Gltf) -> Result<Mesh, ConvertError> {
    todo!()
}
