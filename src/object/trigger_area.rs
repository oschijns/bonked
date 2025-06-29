//! Trigger zone which detect intersection with kinematic bodies

use super::{CommonData, Mask, Object, MASK_ALL};
use crate::{object::kinematic_body::KinematicBody, world::aabb::Aabb};
use alloc::sync::Arc;
use bvh_arena::VolumeHandle;
use delegate::delegate;
use parry::{
    math::{Isometry, Real},
    shape::Shape,
};

/// Callback function to assign to a trigger
pub type Callback<PT, PK> = fn(&mut PT, &mut KinematicBody<PK>);

/// A trigger zone in the world
pub struct TriggerArea<PT = (), PK = ()> {
    /// Shape, isometry and handle
    common: CommonData<PT>,

    /// Layers this trigger zone can detect objects on
    mask: Mask,

    /// Function called when this trigger area overlap with a kinematic body
    overlap: Callback<PT, PK>,
}

impl<PT, PK> TriggerArea<PT, PK> {
    /// Create a new trigger area
    #[inline]
    pub fn new(
        shape: Arc<dyn Shape>,
        isometry: Isometry<Real>,
        payload: PT,
        mask: Mask,
        overlap: Callback<PT, PK>,
    ) -> Self {
        Self {
            common: CommonData::new(shape, isometry, payload),
            mask,
            overlap,
        }
    }
}

impl<PT, PK> Object<PT> for TriggerArea<PT, PK> {
    delegate! {
        to self.common {
            fn set_handle(&mut self, handle: VolumeHandle);
            fn unset_handle(&mut self);
            fn handle(&self) -> Option<VolumeHandle>;
            fn shape(&self) -> &dyn Shape;
            fn isometry(&self) -> &Isometry<f32>;
            fn payload(&self) -> &PT;
            fn payload_mut(&mut self) -> &mut PT;
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

impl<PT, PK> TriggerArea<PT, PK> {
    /// Access the callback defined for when this area overlap with a body
    #[inline]
    pub fn on_overlap(&self) -> Callback<PT, PK> {
        self.overlap
    }
}
