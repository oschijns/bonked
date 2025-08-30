//! Static object

use super::Object;
use crate::{
    mask::{LayerFilter, Mask},
    object::common_object::CommonObject,
};
use delegate::delegate;
use parry::{
    bounding_volume::Aabb,
    math::{Isometry, Real},
    shape::{Shape, SharedShape},
};

/// Static objects cannot move over time
pub struct StaticObject {
    /// Shape, isometry and handle
    common: CommonObject,
}

impl StaticObject {
    /// Build a new static body
    #[inline]
    pub fn new(
        shape: SharedShape,
        isometry: Isometry<Real>,
        layer_filter: LayerFilter,
        is_trigger: bool,
    ) -> Self {
        Self {
            common: CommonObject::new(shape, isometry, layer_filter, is_trigger),
        }
    }
}

impl Object for StaticObject {
    delegate! {
        to self.common {
            #[inline] fn shape(&self) -> &dyn Shape;
            #[inline] fn isometry(&self) -> &Isometry<Real>;
            #[inline] fn aabb(&self) -> Aabb;
            #[inline] fn is_dynamic(&self) -> bool;
            #[inline] fn is_trigger_area(&self) -> bool;
            #[inline] fn layer_filter(&self) -> &LayerFilter;
        }
    }

    /// Return true if this object pass the given filter
    #[inline]
    fn pass_filter(&self, filter: Mask) -> bool {
        (self.common.layer_filter.layer & filter) != 0
    }

    /// Try to cast the object into a static body
    #[inline]
    fn as_static(&self) -> Option<&StaticObject> {
        Some(self)
    }
}
