//! Trigger zone which detect intersection with kinematic bodies

use super::{kinematic_body::KinematicBody, Mask, Object};
use alloc::sync::Arc;
use parry::{
    bounding_volume::Aabb,
    math::{Isometry, Real},
    query::intersection_test,
    shape::Shape,
};

/// A trigger zone in the world
pub struct TriggerArea {
    /// Collision shape used by this zone
    shape: Arc<dyn Shape>,

    /// Isometry (transform) of the zone
    isometry: Isometry<Real>,

    /// Layers this trigger zone can detect objects on
    mask: Mask,
}

impl Object for TriggerArea {
    #[inline]
    fn shape(&self) -> &dyn Shape {
        self.shape.as_ref()
    }

    #[inline]
    fn isometry(&self) -> &Isometry<f32> {
        &self.isometry
    }

    /// Compute the AABB of this trigger zone
    #[inline]
    fn aabb(&self) -> Aabb {
        self.shape.compute_aabb(&self.isometry)
    }

    /// Check if this trigger can detect the given body
    #[inline]
    fn layer_match(&self, other: &dyn Object) -> bool {
        self.mask & other.layer() != 0
    }

    #[inline]
    fn mask(&self) -> Mask {
        self.mask
    }
}

impl TriggerArea {
    /// Check if this trigger intersects with the given kinematic body
    pub fn intersects_with(&self, kinematic: &KinematicBody) -> bool {
        intersection_test(
            kinematic.isometry(),
            kinematic.shape(),
            &self.isometry,
            self.shape(),
        )
        .unwrap_or(false)
    }
}
