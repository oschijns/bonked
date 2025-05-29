//! Fixed body which does not report collisions

use super::{kinematic_body::KinematicBody, Mask, Object};
use alloc::sync::Arc;
use parry::{
    bounding_volume::Aabb,
    math::{Isometry, Real},
    query::{contact, Contact},
    shape::Shape,
};

/// A fixed body in the world
pub struct StaticBody {
    /// Collision shape used by this body
    shape: Arc<dyn Shape>,

    /// Isometry (transform) of the body
    isometry: Isometry<Real>,

    /// Specify the layer this body belongs to
    layer: Mask,
}

impl StaticBody {
    /// Build a new static body
    pub fn new(shape: Arc<dyn Shape>, isometry: Isometry<Real>, layer: Mask) -> Self {
        Self {
            shape,
            isometry,
            layer,
        }
    }
}

impl Object for StaticBody {
    #[inline]
    fn shape(&self) -> &dyn Shape {
        self.shape.as_ref()
    }

    #[inline]
    fn isometry(&self) -> &Isometry<f32> {
        &self.isometry
    }

    /// Compute the AABB of this fixed body
    #[inline]
    fn aabb(&self) -> Aabb {
        self.shape.compute_aabb(&self.isometry)
    }

    /// Check that the other body detect collision on the layer this fixed body is
    #[inline]
    fn layer_match(&self, other: &dyn Object) -> bool {
        self.layer & other.mask() != 0
    }

    #[inline]
    fn layer(&self) -> Mask {
        self.layer
    }
}

impl StaticBody {
    /// Detect a collision between this fixed body and a kinematic body
    pub fn collides_with(&self, kinematic: &KinematicBody) -> Option<Contact> {
        // Check if the fixed body and the kinematic body are colliding
        contact(
            kinematic.next_isometry(),
            kinematic.shape(),
            &self.isometry,
            self.shape(),
            0.0,
        )
        .unwrap_or(None)
    }
}
