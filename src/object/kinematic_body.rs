//! Kinematic body which reports collisions

use super::{CommonData, Mask, Object, OptPayload, WeakPayload};
use crate::{object::contact::Contact, world::aabb::Aabb};
use alloc::{boxed::Box, sync::Arc, vec::Vec};
use bvh_arena::VolumeHandle;
use delegate::delegate;
use nalgebra_glm::is_null;
use parry::{
    math::{Isometry, Real, Translation, Vector},
    query::ShapeCastHit,
    shape::Shape,
};

/// A kinematic body in the world
pub struct KinematicBody {
    /// Shape, isometry and handle
    common: CommonData,

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
    contacts: Vec<Box<Contact>>,
}

impl KinematicBody {
    /// Create a new kinematic body
    pub fn new(
        shape: Arc<dyn Shape>,
        isometry: Isometry<Real>,
        payload: OptPayload,
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
            contacts: Vec::new(),
        }
    }
}

impl Object for KinematicBody {
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

impl KinematicBody {
    /// Compute the estimated next isometry by applying the velocity
    pub fn pre_update(&mut self, delta_time: Real) {
        // submit the computed new isometry
        self.common.isometry = self.next_isometry;

        // Now move the estimated next isometry to
        // its expected location based on the velocity.
        let translation = Translation::from(self.velocity * delta_time);
        self.next_isometry.append_translation_mut(&translation);

        // Reset the list of hits
        self.contacts.clear();
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
    pub fn add_contact(
        &mut self,
        hit: ShapeCastHit,
        other_weight: Option<Real>,
        payload: WeakPayload,
    ) {
        // Compare the weight of the two object to deduce
        // which one should push back the other more.
        let weight_ratio = if let Some(w) = other_weight {
            1.0 - (self.weight / (self.weight + w))
        } else {
            // the other object is a static body
            1.0
        };

        // add the hit result to the set
        self.contacts
            .push(Box::new(Contact::new(hit, weight_ratio, payload)));
    }

    /// Apply the hits to the body
    pub fn apply_contacts(&mut self, delta_time: Real, epsilon: Real) {
        // order the hits from closest to furthest
        self.contacts.sort_by(|a, b| a.order(b, epsilon));

        // Compute how much we must push back the object
        let mut offset = Vector::<Real>::zeros();
        for contact in self.contacts.iter() {
            // push back the object according to its mass
            let hit = contact.hit();
            let normal = hit.normal1.into_inner();
            let ratio = contact.weight_ratio();

            // push back the object
            offset -= normal * (hit.time_of_impact * ratio);

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
