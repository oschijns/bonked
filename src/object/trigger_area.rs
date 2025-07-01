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

/// Function called on overlaps between this trigger and a body
pub type OnOverlap<T, B> = fn(&mut TriggerArea<T, B>, &mut KinematicBody<B>);

/// A trigger zone in the world
pub struct TriggerArea<P = (), B = ()> {
    /// Shape, isometry and handle
    common: CommonData<P>,

    /// Layers this trigger zone can detect objects on
    mask: Mask,

    /// Function called when this trigger area overlap with a kinematic body
    on_overlap: OnOverlap<P, B>,
}

impl<P, B> TriggerArea<P, B> {
    /// Create a new trigger area
    #[inline]
    pub fn new(
        shape: Arc<dyn Shape>,
        isometry: Isometry<Real>,
        payload: P,
        mask: Mask,
        on_overlap: OnOverlap<P, B>,
    ) -> Self {
        Self {
            common: CommonData::new(shape, isometry, payload),
            mask,
            on_overlap,
        }
    }
}

impl<P, B> Object for TriggerArea<P, B> {
    type Payload = P;

    delegate! {
        to self.common {
            #[inline] fn set_handle(&mut self, handle: VolumeHandle);
            #[inline] fn unset_handle(&mut self);
            #[inline] fn handle(&self) -> Option<VolumeHandle>;
            #[inline] fn shape(&self) -> &dyn Shape;
            #[inline] fn isometry(&self) -> &Isometry<Real>;
            #[inline] fn payload(&self) -> &P;
            #[inline] fn payload_mut(&mut self) -> &mut P;
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

impl<P, B> TriggerArea<P, B> {
    /// Access the callback defined for when this area overlap with a body
    #[inline]
    pub fn on_overlap(&mut self, body: &mut KinematicBody<B>) {
        (self.on_overlap)(self, body)
    }
}
