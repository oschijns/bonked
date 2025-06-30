//! Trigger zone which detect intersection with kinematic bodies

use super::{CommonData, Mask, Object, OptPayload, WeakPayload, MASK_ALL};
use crate::{object::kinematic_body::KinematicBody, world::aabb::Aabb};
use alloc::sync::Arc;
use bvh_arena::VolumeHandle;
use delegate::delegate;
use parry::{
    math::{Isometry, Real},
    shape::Shape,
};

/// Callback function to assign to a trigger
pub type Callback = fn(WeakPayload, &mut KinematicBody);

/// A trigger zone in the world
pub struct TriggerArea {
    /// Shape, isometry and handle
    common: CommonData,

    /// Layers this trigger zone can detect objects on
    mask: Mask,

    /// Function called when this trigger area overlap with a kinematic body
    overlap: Callback,
}

impl TriggerArea {
    /// Create a new trigger area
    #[inline]
    pub fn new(
        shape: Arc<dyn Shape>,
        isometry: Isometry<Real>,
        payload: OptPayload,
        mask: Mask,
        overlap: Callback,
    ) -> Self {
        Self {
            common: CommonData::new(shape, isometry, payload),
            mask,
            overlap,
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
            fn payload(&self) -> OptPayload;
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

impl TriggerArea {
    /// Access the callback defined for when this area overlap with a body
    #[inline]
    pub fn on_overlap(&self) -> Callback {
        self.overlap
    }
}
