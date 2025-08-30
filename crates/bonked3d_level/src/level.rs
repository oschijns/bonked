//! Mesh data to store onto file or load from a file

/// Common operations on some basic types
mod base;

/// Handle mesh decomposition
mod decomp;

/// Encoding and decoding implementation for the mesh
mod encoding;

/// Reconstruct the convex hulls from the level part
mod reconstruct;

use alloc::vec::Vec;
use parry::math::{self, Point};

/// Mesh that can be used for rendering and generating colliders
pub struct LevelPart {
    /// Positional coordinates
    positions: List<Vector<Real, 3>>,

    /// NormalsVec
    normals: Option<Vec<Vector<i8, 3>>>,

    /// Colors
    colors: Option<Vec<Vector<u8, 3>>>,

    /// UV coordinates
    uvs: Option<Vec<Vector<Real, 2>>>,

    /// Indices for rendering
    indices: List<Vector<Index, 3>>,

    /// Indices for convex hulls
    hull_indices: List<List<Vector<Index, 3>>>,
}

/// Numeric type used to index vertices and thus limit the
/// maximum number of vertices that can be defined in a mesh.
pub type Index = u16;

/// Floating point type used to encode coordinates
pub type Real = f32;

/// Vector type
pub struct Vector<N, const DIM: usize>(pub [N; DIM]);

/// List of elements (allow encoding its size using Index type)
pub struct List<T>(pub Vec<T>);

/// Generic mesh data to convert
pub struct GenericMesh<'m, V, I>
where
    V: ToParryPoint,
    I: ToParryIndex,
{
    /// Vertices data to serialize
    vertices: &'m [V],

    /// Indices to compose the triangles
    indices: &'m [I],
}

pub trait ToParryPoint {
    /// convert to be used in VHACD
    fn to_parry(&self) -> Point<math::Real>;
}

pub trait ToParryIndex {
    /// convert to be used in VHACD
    fn to_parry(&self) -> [u32; 3];
}
