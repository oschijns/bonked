//! Mesh data to store onto file or load from a file

/// Common operations on some basic types
mod base;

/// Handle mesh decomposition
pub mod decomp;

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
pub struct GenericMesh<'m, V, I> {
    /// Vertices data to serialize
    vertices: &'m [V],

    /// Indices to compose the triangles
    indices: &'m [I],
}

/// Get data from a vertex
pub trait ToLevelVertex {
    /// Get the positional coordinates of the vertex
    fn get_position(&self) -> Vector<Real, 3>;

    /// Get the normal of the vertex
    #[inline]
    fn get_normal(&self) -> Option<Vector<i8, 3>> {
        None
    }

    /// Get the color of the vertex
    #[inline]
    fn get_color(&self) -> Option<Vector<u8, 3>> {
        None
    }

    /// Get the UV texture coordinates of the vertex
    #[inline]
    fn get_uv(&self) -> Option<Vector<Real, 2>> {
        None
    }
}

/// Cast triangle index to serializable format
pub trait ToLevelIndex {
    /// convert to be used in VHACD
    fn get_index(&self) -> Vector<Index, 3>;
}

/// Convert a vertex into a parry's point
pub trait ToParryPoint {
    /// Point usable by parry
    fn to_parry(&self) -> Point<math::Real>;
}

/// Convert a triangle index into a parry's triangle index
pub trait ToParryIndex {
    /// Triangle index usable by parry
    fn to_parry(&self) -> [u32; 3];
}
