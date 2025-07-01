//! Fixed body which does not report collisions

use super::{CommonData, Mask, Object, MASK_ALL};
use crate::world::aabb::Aabb;
use alloc::sync::Arc;
use bvh_arena::VolumeHandle;
use delegate::delegate;
use parry::{
    math::{Isometry, Real},
    shape::Shape,
};

/// A fixed body in the world
pub struct StaticBody<P = ()> {
    /// Shape, isometry and handle
    common: CommonData<P>,

    /// Specify the layer this body belongs to
    layer: Mask,
}

impl<P> StaticBody<P> {
    /// Build a new static body
    #[inline]
    pub fn new(shape: Arc<dyn Shape>, isometry: Isometry<Real>, payload: P, layer: Mask) -> Self {
        Self {
            common: CommonData::new(shape, isometry, payload),
            layer,
        }
    }
}

impl<P> Object for StaticBody<P> {
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

    /// Compute the AABB of this fixed body
    #[inline]
    fn aabb(&self) -> Aabb {
        Aabb::new(
            self.common.shape.compute_aabb(&self.common.isometry),
            self.layer,
            MASK_ALL,
        )
    }

    #[inline]
    fn layer(&self) -> Mask {
        self.layer
    }

    /// Try to cast the object into a static body
    #[inline]
    fn as_static(&self) -> Option<&StaticBody<Self::Payload>> {
        Some(self)
    }
}
