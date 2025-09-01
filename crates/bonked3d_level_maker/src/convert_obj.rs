//! Convert 3D file format to bonked format

use bonked3d_level::{
    level::{
        GenericMesh, Index, LevelPart, Real, ToLevelIndex, ToLevelVertex, ToParryIndex,
        ToParryPoint, Vector, decomp::ConvertError,
    },
    parry::math::{self, Point},
};
use obj::Obj;

/// Convert obj file into level part
pub fn convert_obj(obj: &Obj) -> Result<LevelPart, ConvertError> {
    let vertices = obj
        .vertices
        .iter()
        .map(|v| Vertex::from(*v))
        .collect::<Vec<_>>();

    let indices = obj
        .indices
        .as_chunks::<3>()
        .0
        .iter()
        .map(|i| TriIndices(*i))
        .collect::<Vec<_>>();

    // prepare a generic mesh and convert it into a level part
    let mesh = GenericMesh::new(&vertices, &indices);
    mesh.to_level_part()
}

/// Vertex
struct Vertex {
    /// Positionnal coordinates of the vertex
    position: [f32; 3],

    /// Normal of the vertex
    normal: [f32; 3],
}

/// Triplet of indices
struct TriIndices([u16; 3]);

impl From<obj::Vertex> for Vertex {
    #[inline]
    fn from(value: obj::Vertex) -> Self {
        Vertex {
            position: value.position,
            normal: value.normal,
        }
    }
}

impl ToLevelVertex for Vertex {
    #[inline]
    fn get_position(&self) -> Vector<Real, 3> {
        Vector(self.position)
    }
}

impl ToLevelIndex for TriIndices {
    #[inline]
    fn get_index(&self) -> Vector<Index, 3> {
        Vector(self.0)
    }
}

impl ToParryPoint for Vertex {
    #[inline]
    fn to_parry(&self) -> Point<math::Real> {
        let [x, y, z] = self.position;
        Point::new(x, y, z)
    }
}

impl ToParryIndex for TriIndices {
    #[inline]
    fn to_parry(&self) -> [u32; 3] {
        let [x, y, z] = self.0;
        [x as u32, y as u32, z as u32]
    }
}
