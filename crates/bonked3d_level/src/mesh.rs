//! Mesh data to store onto file or load from a file

/// Encoding and decoding implementation for the mesh
mod encoding;

/// Numeric type used to index vertices and thus limit the
/// maximum number of vertices that can be defined in a mesh.
pub type Index = u16;

/// Floating point type used to encode coordinates
pub type Real = f32;

/// Vector type
pub struct Vector<N, const DIM: usize>(pub [N; DIM]);

/// List of elements (allow encoding its size using Index type)
pub struct List<T>(pub Vec<T>);

bitfield::bitfield! {
    /// Specify if vertices have some extra data (normal, color, UV)
    pub struct Mask(u8);
    impl Debug;

    /// Normals are defined
    pub use_normals, set_normals_use: 0;

    /// Colors are defined
    pub use_colors, set_colors_use: 1;

    /// UV coordinates are defined
    pub use_uvs, set_uvs_use: 2;
}

/// Mesh data that can be used for rendering and generating colliders
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
    indices: List<Index>,

    /// Indices for convex hulls
    hull_indices: List<List<Index>>,
}
