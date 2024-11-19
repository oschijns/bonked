use crate::{Collider, Position};
use parry3d::math::{Isometry, Real, Translation, Vector};

impl<A> Collider<A> {
    /// Check if the collision layer of the first object match the one of the second
    #[inline]
    pub fn can_collide_with(&self, other: &Self) -> bool {
        (self.layer & other.mask) != 0
    }
}

impl Position {
    /// Get the end-point of
    pub fn get_end_point(&self, displacement: Vector<Real>) -> Isometry<Real> {
        let mut end_pos = self.0;
        end_pos.append_translation_mut(&Translation::from(displacement));
        end_pos
    }
}

/// Check if the Vector is null
#[inline]
pub(crate) fn is_null(vec: &Vector<Real>) -> bool {
    vec.x == 0.0 && vec.y == 0.0 && vec.z == 0.0
}
