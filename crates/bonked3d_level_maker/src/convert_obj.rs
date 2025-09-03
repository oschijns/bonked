//! Convert 3D file format to bonked format

use bonked3d_level::level::{
    Index, Mesh, Vector,
    decomp::ConvertError,
    generic_mesh::{GenericMesh, Vertex},
};
use obj::Obj;

/// Convert obj file into level part
pub fn convert_obj(obj: &Obj) -> Result<Mesh, ConvertError> {
    let vertices = obj
        .vertices
        .iter()
        .map(|v| Vertex::new(v.position))
        .collect::<Vec<_>>();

    let indices = obj
        .indices
        .as_chunks::<3>()
        .0
        .iter()
        .map(|i| {
            let [x, y, z] = i;
            Vector([*x as Index, *y as Index, *z as Index])
        })
        .collect::<Vec<_>>();

    // prepare a generic mesh and convert it into a level part
    let mesh = GenericMesh { vertices, indices };
    mesh.to_serial_mesh()
}
