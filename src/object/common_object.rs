//! Data encountered in all objects

use crate::mask::Mask;

use super::{LayerFilter, Object};
use alloc::sync::Arc;
use parry::{
    bounding_volume::Aabb,
    math::{Isometry, Real},
    shape::Shape,
};

/// Common data shared between static and dynamic bodies
pub(crate) struct CommonObject {
    /// Collision shape used by this zone
    pub(crate) shape: Arc<dyn Shape>,

    /// Isometry of this body
    pub(crate) isometry: Isometry<Real>,

    /// Collision mask for this object
    pub(crate) layer_filter: LayerFilter,

    /// Indicate if this object is a trigger area instead of a physical object
    pub(crate) is_trigger: bool,
}

impl CommonObject {
    /// Create a new common data instance
    #[inline]
    pub fn new(
        shape: Arc<dyn Shape>,
        isometry: Isometry<Real>,
        layer_filter: LayerFilter,
        is_trigger: bool,
    ) -> Self {
        Self {
            shape,
            isometry,
            layer_filter,
            is_trigger,
        }
    }
}

impl Object for CommonObject {
    /// Access the shape assigned to this body
    #[inline]
    fn shape(&self) -> &dyn Shape {
        self.shape.as_ref()
    }

    /// Access the isometry of this shape
    #[inline]
    fn isometry(&self) -> &Isometry<Real> {
        &self.isometry
    }

    /// Build a generic AABB for this body
    #[inline]
    fn aabb(&self) -> Aabb {
        self.shape.compute_aabb(&self.isometry)
    }

    #[inline]
    fn is_dynamic(&self) -> bool {
        false
    }

    #[inline]
    fn is_trigger_area(&self) -> bool {
        self.is_trigger
    }

    #[inline]
    fn layer_filter(&self) -> &LayerFilter {
        &self.layer_filter
    }

    /// Return true if this object pass the given filter
    #[inline]
    fn pass_filter(&self, filter: Mask) -> bool {
        (self.layer_filter.layer & filter) != 0
    }
}
