//! Kinematic body which reports collisions

use super::{utils::Accumulator, CommonData, Mask, Object};
use crate::world::aabb::Aabb;
use alloc::sync::Arc;
use bvh_arena::VolumeHandle;
use delegate::delegate;
use parry::{
    math::{Isometry, Real, Translation, Vector},
    query::{cast_shapes, Contact, ShapeCastHit, ShapeCastOptions},
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

    /// Velocity of the object
    velocity: Vector<Real>,

    /// Target isometry at the next tick
    next_isometry: Isometry<Real>,

    /// Accumulator for correcting the position of the body
    accumulator: Accumulator,
}

impl KinematicBody {
    /// Create a new kinematic body
    pub fn new(shape: Arc<dyn Shape>, isometry: Isometry<Real>, layer: Mask, mask: Mask) -> Self {
        Self {
            common: CommonData::new(shape, isometry),
            layer,
            mask,
            velocity: Vector::zeros(),
            next_isometry: isometry,
            accumulator: Accumulator::default(),
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

    #[inline]
    fn layer(&self) -> Mask {
        self.layer
    }

    #[inline]
    fn mask(&self) -> Mask {
        self.mask
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
    }

    /// Access the next isometry of the body
    #[inline]
    pub fn next_isometry(&self) -> &Isometry<Real> {
        &self.next_isometry
    }

    /// Detect a collision between those two kinematic bodies
    pub fn collides_with(
        &self,
        other: &KinematicBody,
        options: ShapeCastOptions,
    ) -> Option<ShapeCastHit> {
        // Check if the two objects will collide
        cast_shapes(
            &self.isometry(),
            &self.velocity,
            self.shape(),
            &other.isometry(),
            &other.velocity,
            other.shape(),
            options,
        )
        .unwrap_or(None)
    }

    /// Apply contact to this kinematic body
    pub fn apply_contact(&mut self, contact: &Contact) {
        self.accumulator
            .add_collision(&contact.point2, &contact.normal2, contact.dist);
    }

    /// Apply collision from other kinematic body to this body
    pub fn apply_collision(&mut self, collision: &ShapeCastHit) {}
}
