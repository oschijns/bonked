//! Trigger zone which detect intersection with kinematic bodies

use super::{CommonData, Mask, Object, MASK_ALL};
use crate::world::aabb::Aabb;
use alloc::sync::Arc;
use bvh_arena::VolumeHandle;
use delegate::delegate;
use parry::{
    math::{Isometry, Real},
    shape::Shape,
};

/// A trigger zone in the world
pub struct TriggerArea {
    /// Shape, isometry and handle
    common: CommonData,

    /// Layers this trigger zone can detect objects on
    mask: Mask,
}

impl TriggerArea {
    /// Create a new trigger area
    #[inline]
    pub fn new(shape: Arc<dyn Shape>, isometry: Isometry<Real>, mask: Mask) -> Self {
        Self {
            common: CommonData::new(shape, isometry),
            mask,
        }
    }
}

impl Object for TriggerArea {
    delegate! {
        to self.common {
            fn set_handle(&mut self, handle: VolumeHandle);
            fn unset_handle(&mut self);
            fn handle(&self) -> Option<VolumeHandle>;
            fn shape(&self) -> &dyn Shape;
            fn isometry(&self) -> &Isometry<f32>;
        }
    }

    /// Compute the AABB of this trigger zone
    #[inline]
    fn aabb(&self) -> Aabb {
        Aabb::new(
            self.common.shape.compute_aabb(&self.common.isometry),
            MASK_ALL,
            self.mask,
        )
    }

    #[inline]
    fn mask(&self) -> Mask {
        self.mask
    }
}
