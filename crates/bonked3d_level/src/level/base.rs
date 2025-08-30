//! Basic operations on some basic types

use crate::level::{GenericMesh, ToParryIndex, ToParryPoint, Vector};
use num_traits::AsPrimitive;
use parry::math::{self, Point};

impl<'m, V, I> GenericMesh<'m, V, I>
where
    V: ToParryPoint,
    I: ToParryIndex,
{
    /// Create a new container for a generic mesh
    #[inline]
    pub fn new(vertices: &'m [V], indices: &'m [I]) -> Self {
        Self { vertices, indices }
    }
}

impl<N> ToParryPoint for Vector<N, 3>
where
    N: AsPrimitive<math::Real>,
{
    /// Convert the serialized vector into a parry's point
    #[inline]
    fn to_parry(&self) -> Point<math::Real> {
        Point::new(self.0[0].as_(), self.0[1].as_(), self.0[2].as_())
    }
}

impl<N> ToParryIndex for Vector<N, 3>
where
    N: AsPrimitive<u32>,
{
    /// Convert the serialized vector into a parry's triangle index
    #[inline]
    fn to_parry(&self) -> [u32; 3] {
        [self.0[0].as_(), self.0[1].as_(), self.0[2].as_()]
    }
}
