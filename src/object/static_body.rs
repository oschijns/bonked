//! Fixed body which does not report collisions

use super::{CommonData, Mask, Object, OptPayload, MASK_ALL};
use crate::world::aabb::Aabb;
use alloc::sync::Arc;
use bvh_arena::VolumeHandle;
use delegate::delegate;
use parry::{
    math::{Isometry, Real},
    shape::Shape,
};

/// A fixed body in the world
pub struct StaticBody {
    /// Shape, isometry and handle
    common: CommonData,

    /// Specify the layer this body belongs to
    layer: Mask,
}

impl StaticBody {
    /// Build a new static body
    #[inline]
    pub fn new(
        shape: Arc<dyn Shape>,
        isometry: Isometry<Real>,
        payload: OptPayload,
        layer: Mask,
    ) -> Self {
        Self {
            common: CommonData::new(shape, isometry, payload),
            layer,
        }
    }
}

impl Object for StaticBody {
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
}
