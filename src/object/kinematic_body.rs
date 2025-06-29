//! Kinematic body which reports collisions

use super::{CommonData, Mask, Object};
use crate::world::aabb::Aabb;
use alloc::{boxed::Box, sync::Arc, vec::Vec};
use bvh_arena::VolumeHandle;
use core::cmp::Ordering;
use delegate::delegate;
use nalgebra_glm::is_null;
use parry::{
    math::{Isometry, Real, Translation, Vector},
    query::ShapeCastHit,
    shape::Shape,
};

/// A kinematic body in the world
pub struct KinematicBody<P = ()> {
    /// Shape, isometry and handle
    common: CommonData<P>,

    /// Collision layer of the body
    layer: Mask,

    /// Collision mask of the body
    mask: Mask,

    /// Weight of this object, define how two objects can push against each other
    weight: Real,

    /// Velocity of the object.
    /// It can be accessed directly to modify each coordinate individually.
    pub velocity: Vector<Real>,

    /// Target isometry at the next tick
    next_isometry: Isometry<Real>,

    /// Store collision results
    /// Hit results are stored in boxes so that reordoring the vector can be quicker
    #[allow(clippy::vec_box)]
    hits: Vec<Box<HitResult>>,
}

/// Hit result
pub struct HitResult {
    /// parry's shape cast hit
    hit: ShapeCastHit,

    /// weight ratio between this object and the object being hit
    weight_ratio: Real,
}

impl<P> KinematicBody<P> {
    /// Create a new kinematic body
    pub fn new(
        shape: Arc<dyn Shape>,
        isometry: Isometry<Real>,
        payload: P,
        layer: Mask,
        mask: Mask,
        weight: Real,
    ) -> Self {
        Self {
            common: CommonData::new(shape, isometry, payload),
            layer,
            mask,
            weight,
            velocity: Vector::zeros(),
            next_isometry: isometry,
            hits: Vec::new(),
        }
    }
}

impl<P> Object<P> for KinematicBody<P> {
    delegate! {
        to self.common {
            fn set_handle(&mut self, handle: VolumeHandle);
            fn unset_handle(&mut self);
            fn handle(&self) -> Option<VolumeHandle>;
            fn shape(&self) -> &dyn Shape;
            fn isometry(&self) -> &Isometry<f32>;
            fn payload(&self) -> &P;
            fn payload_mut(&mut self) -> &mut P;
        }
    }

    /// Compute the AABB of this moving body
    #[inline]
    fn aabb(&self) -> Aabb {
        Aabb::new(
            self.common
                .shape
                .compute_swept_aabb(&self.common.isometry, &self.next_isometry),
            self.layer,
            self.mask,
        )
    }

    /// Access the layer this body exists in
    #[inline]
    fn layer(&self) -> Mask {
        self.layer
    }

    /// Access the layers this body will interact with
    #[inline]
    fn mask(&self) -> Mask {
        self.mask
    }

    /// Get the velocity of the body (if it has one)
    #[inline]
    fn velocity(&self) -> Vector<Real> {
        self.velocity
    }
}

impl<P> KinematicBody<P> {
    /// Compute the estimated next isometry by applying the velocity
    pub fn pre_update(&mut self, delta_time: Real) {
        // submit the computed new isometry
        self.common.isometry = self.next_isometry;

        // Now move the estimated next isometry to
        // its expected location based on the velocity.
        let translation = Translation::from(self.velocity * delta_time);
        self.next_isometry.append_translation_mut(&translation);

        // Reset the list of hits
        self.hits.clear();
    }

    /// Access the weight of the kinematic body
    #[inline]
    pub fn weight(&self) -> Real {
        self.weight
    }

    /// Access the next isometry of the body
    #[inline]
    pub fn next_isometry(&self) -> &Isometry<Real> {
        &self.next_isometry
    }

    /// Apply the collision to this body
    #[inline]
    pub fn add_hit(&mut self, hit: ShapeCastHit, other_weight: Option<Real>) {
        // Compare the weight of the two object to deduce
        // which one should push back the other more.
        let weight_ratio = if let Some(w) = other_weight {
            1.0 - (self.weight / (self.weight + w))
        } else {
            // the other object is a static body
            1.0
        };

        // add the hit result to the set
        self.hits.push(Box::new(HitResult { hit, weight_ratio }));
    }

    /// Apply the hits to the body
    pub fn apply_hits(&mut self, delta_time: Real, epsilon: Real) {
        // order the hits from closest to furthest
        self.hits.sort_by(|a, b| a.order(b, epsilon));

        // Compute how much we must push back the object
        let mut offset = Vector::<Real>::zeros();
        for hit in self.hits.iter() {
            // push back the object according to its mass
            let normal = hit.hit.normal1.into_inner();
            let ratio = hit.weight_ratio;

            // push back the object
            offset -= normal * (hit.hit.time_of_impact * ratio);

            // cut off form the velocity
            self.velocity -= normal * (normal.dot(&self.velocity) * ratio);
        }

        // check if the push back is relevant
        if !is_null(&offset, epsilon) {
            // apply the push back to the position of the object
            let translation = Translation::from(offset * delta_time);
            self.next_isometry.append_translation_mut(&translation);
        }
    }
}

impl HitResult {
    fn order(&self, other: &Self, epsilon: Real) -> Ordering {
        let ta = self.hit.time_of_impact;
        let tb = other.hit.time_of_impact;
        if (ta - tb).abs() < epsilon {
            Ordering::Equal
        } else if ta < tb {
            Ordering::Less
        } else {
            Ordering::Greater
        }
    }
}
