//! Mesh data to store onto file or load from a file

/// Handle mesh decomposition
pub mod decomp;

/// Encoding and decoding implementation for the mesh
mod encoding;

/// Generic mesh that is being used as an intermediary to generate optimized level parts
pub mod generic_mesh;

/// Reconstruct the convex hulls from the level part
mod reconstruct;

use alloc::vec::Vec;

/// Level that is composed of multiple parts
pub struct Level {
    /// Meshes defined in the level
    meshes: List<Mesh>,

    /// Define mesh brushes
    mesh_brushes: List<MeshBrush>,
}

/// Brush using a mesh
pub struct MeshBrush {
    /// Index to reference a mesh
    index: Index,

    /// isometry to position the brush
    isometry: Isometry,
}

/// Isometry to position a brush in the level
pub struct Isometry {
    /// position of the brush
    position: Vector<Real, 3>,

    /// rotation of the brush
    rotation: Vector<i8, 4>,
}

/// Mesh that can be used for rendering and generating colliders
pub struct Mesh {
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
#[derive(Debug, Clone, Copy)]
pub struct Vector<N, const DIM: usize>(pub [N; DIM]);

/// List of elements (allow encoding its size using Index type)
#[derive(Debug, Clone)]
pub struct List<T>(pub Vec<T>);
