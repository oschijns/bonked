//! Generic mesh that is being used as an intermediary to generate optimized level parts

use crate::level::{Index, Real, Vector};
use parry::{math, na::Quaternion};

/// Generic mesh data to convert
pub struct GenericMesh {
    /// Vertices data to serialize
    pub vertices: Vec<Vertex>,

    /// Indices to compose the triangles
    pub indices: Vec<Vector<Index, 3>>,
}

/// Generic vertex
pub struct Vertex {
    /// Positionnal coordinate of the vertex
    pub position: Vector<Real, 3>,

    /// Normal of the vertex if any
    pub normal: Option<Vector<i8, 3>>,

    /// Vertex color if any
    pub color: Option<Vector<u8, 3>>,

    /// Texture UV coordinates
    pub uv: Option<Vector<Real, 2>>,
}

impl Vertex {
    /// Create a new vertex with positional coordinates
    #[inline]
    pub fn new(coords: [Real; 3]) -> Self {
        Self {
            position: Vector(coords),
            normal: None,
            color: None,
            uv: None,
        }
    }
}

impl Vector<Real, 3> {
    /// Convert generic vertex into 3D point usable by parry
    #[inline]
    pub fn parry_point(&self) -> math::Point<math::Real> {
        let [x, y, z] = self.0;
        math::Point::new(x as math::Real, y as math::Real, z as math::Real)
    }
}

impl Vector<Index, 3> {
    /// Convert triangle of indices into array usable by parry
    #[inline]
    pub fn parry_index(&self) -> [u32; 3] {
        let [x, y, z] = self.0;
        [x as u32, y as u32, z as u32]
    }
}

impl Vector<i8, 4> {
    /// Create a serializable vector from a parry's quaternion
    #[inline]
    pub fn from_parry_quat(q: &Quaternion<math::Real>) -> Self {
        use parry_float::to_i8 as to;

        Vector([to(q.i), to(q.j), to(q.k), to(q.w)])
    }

    /// Convert optimized quaternion into one usable by parry
    #[inline]
    pub fn to_parry_quat(&self) -> Quaternion<math::Real> {
        use parry_float::from_i8 as from;

        let [x, y, z, w] = self.0;
        Quaternion::new(from(w), from(x), from(y), from(z)).normalize()
    }
}

/// Encode and decode unit vectors
macro_rules! impl_unit_vector {
    ( impl $to_float:ident & $from_float:ident for ( $float:ty => $int:ty ) use ( $to_int:path, $from_int:path)  ) => {
        impl Vector<$int, 3> {
            #[inline]
            pub fn $to_float(&self) -> [$float; 3] {
                use $from_int as from;
                let [x, y, z] = self.0;
                [from(x), from(y), from(z)]
            }

            #[inline]
            pub fn $from_float(coords: [$float; 3]) -> Self {
                use $to_int as to;
                let [x, y, z] = coords;
                Vector([to(x), to(y), to(z)])
            }
        }
    };
}
impl_unit_vector![ impl to_f32 & from_f32 for ( f32 => i8 ) use ( float32::to_i8, float32::from_i8 ) ];
impl_unit_vector![ impl to_f32 & from_f32 for ( f32 => u8 ) use ( float32::to_u8, float32::from_u8 ) ];
impl_unit_vector![ impl to_f64 & from_f64 for ( f64 => i8 ) use ( float64::to_i8, float64::from_i8 ) ];
impl_unit_vector![ impl to_f64 & from_f64 for ( f64 => u8 ) use ( float64::to_u8, float64::from_u8 ) ];

/// Implement functions to convert from floating point number to single byte representation
macro_rules! float_to_unit {
    ( ( $float:ty => $int:ty ) use ( $to:ident, $from:ident) in $const:literal ) => {
        #[inline]
        pub fn $to(float: $float) -> $int {
            const UNIT: $float = $const;
            (float * UNIT) as $int
        }

        #[inline]
        pub fn $from(int: $int) -> $float {
            const UNIT: $float = 1.0 / $const;
            int as $float * UNIT
        }
    };
}

pub mod float32 {
    float_to_unit![ ( f32 => i8 ) use ( to_i8, from_i8 ) in 127.0 ];
    float_to_unit![ ( f32 => u8 ) use ( to_u8, from_u8 ) in 255.0 ];
}

pub mod float64 {
    float_to_unit![ ( f64 => i8 ) use ( to_i8, from_i8 ) in 127.0 ];
    float_to_unit![ ( f64 => u8 ) use ( to_u8, from_u8 ) in 255.0 ];
}

#[cfg(feature = "parry-f32")]
pub use float32 as parry_float;

#[cfg(feature = "parry-f64")]
pub use float64 as parry_float;
