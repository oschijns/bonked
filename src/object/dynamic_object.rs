//! Dynamic object

use super::Object;
use crate::{
    mask::{LayerFilter, Mask},
    object::common_object::CommonObject,
};
use delegate::delegate;
use parry::{
    bounding_volume::Aabb,
    math::{Pose, Real, Vector},
    shape::{Shape, SharedShape},
};

/// Dynamic objects can move over time
pub struct DynamicObject {
    /// Shape, isometry
    common: CommonObject,

    /// Weight of this object, define how two objects can push against each other
    weight: Real,

    /// Velocity of the object.
    /// It can be accessed directly to modify each coordinate individually.
    pub velocity: Vector,

    /// Next position of the object
    pub next_position: Vector,
}

impl DynamicObject {
    /// Create a new dynamic object
    pub fn new(
        shape: SharedShape,
        isometry: Pose,
        layer_filter: LayerFilter,
        is_trigger: bool,
        weight: Real,
    ) -> Self {
        Self {
            common: CommonObject::new(shape, isometry, layer_filter, is_trigger),
            weight,
            velocity: Vector::ZERO,
            next_position: isometry.translation,
        }
    }
}

impl Object for DynamicObject {
    delegate! {
        to self.common {
            #[inline] fn shape(&self) -> &dyn Shape;
            #[inline] fn isometry(&self) -> &Pose;
            #[inline] fn is_trigger_area(&self) -> bool;
            #[inline] fn layer_filter(&self) -> &LayerFilter;
        }
    }

    /// Compute the AABB of this moving body
    #[inline]
    fn aabb(&self) -> Aabb {
        let next = Pose::from_parts(self.next_position, self.common.isometry.rotation);
        self.common
            .shape
            .compute_swept_aabb(&self.common.isometry, &next)
    }

    /// Return true if this object pass the given filter
    #[inline]
    fn pass_filter(&self, filter: Mask) -> bool {
        (self.common.layer_filter.layer & filter) != 0
    }

    /// The object is dynamic
    #[inline]
    fn is_dynamic(&self) -> bool {
        true
    }

    /// Get the velocity of the body (if it has one)
    #[inline]
    fn velocity(&self) -> Vector {
        self.velocity
    }

    /// Try to cast the object into a dynamic object
    #[inline]
    fn as_dynamic(&self) -> Option<&Self> {
        Some(self)
    }
}

impl DynamicObject {
    /// Compute the estimated next isometry by applying the velocity
    pub fn pre_update(&mut self, delta_time: Real) {
        // submit the computed new isometry
        self.common.isometry.translation = self.next_position;

        // Now move the estimated next isometry to
        // its expected location based on the velocity.
        self.next_position += self.velocity * delta_time;
    }

    /// Access the weight of the dynamic object
    #[inline]
    pub fn weight(&self) -> Real {
        self.weight
    }

    /// Access the next isometry of the body
    #[inline]
    pub fn next_isometry(&self) -> Pose {
        Pose::from_parts(self.next_position, self.common.isometry.rotation)
    }

    /// Apply a hit result to this body
    pub(crate) fn apply_hit(
        &mut self,
        time_of_impact: Real,
        others_normal: Vector,
        others_weight: Option<Real>,
    ) {
        // get the normal of the surface of the other object
        let normal = others_normal;
        let ratio = if let Some(weight) = others_weight {
            1.0 - (self.weight / (self.weight + weight))
        } else {
            1.0
        };

        // Prevent the object to pass through other objects
        self.next_position += normal * (time_of_impact * ratio);

        // The dot product specify if the angle between the two vectors is accute or obtuse.
        let dot = normal.dot(self.velocity);
        let push_back = normal * (dot * ratio);

        if dot > 0.0 {
            // angle is accute => add to the velocity
            self.velocity += push_back;
        } else {
            // angle is obtuse => cut off from the velocity
            self.velocity -= push_back;
        }
    }
}
