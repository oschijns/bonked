//! Kinematic body which reports collisions

use super::{Mask, Object};
use alloc::sync::Arc;
use parry::{
    bounding_volume::Aabb,
    math::{Isometry, Real, Translation, Vector},
    query::{cast_shapes, ShapeCastHit, ShapeCastOptions},
    shape::Shape,
};

/// A kinematic body in the world
pub struct KinematicBody {
    /// Collision shape used by this body
    shape: Arc<dyn Shape>,

    /// Isometry (transform) of the body
    isometry: Isometry<Real>,

    /// Velocity of the object
    velocity: Vector<Real>,

    /// Target isometry at the next tick
    next_isometry: Isometry<Real>,

    /// Collision layer of the body
    layer: Mask,

    /// Collision mask of the body
    mask: Mask,
}

impl KinematicBody {
    /// Create a new kinematic body
    pub fn new(shape: Arc<dyn Shape>, isometry: Isometry<Real>, layer: Mask, mask: Mask) -> Self {
        Self {
            shape,
            isometry,
            velocity: Vector::zeros(),
            next_isometry: isometry,
            layer,
            mask,
        }
    }
}

impl Object for KinematicBody {
    #[inline]
    fn shape(&self) -> &dyn Shape {
        self.shape.as_ref()
    }

    #[inline]
    fn isometry(&self) -> &Isometry<f32> {
        &self.isometry
    }

    /// Compute the AABB of this moving body
    #[inline]
    fn aabb(&self) -> Aabb {
        self.shape
            .compute_swept_aabb(&self.isometry, &self.next_isometry)
    }

    /// Check the two bodies match along layers and mask
    fn layer_match(&self, other: &dyn Object) -> bool {
        self.layer & other.mask() != 0 && self.mask & other.layer() != 0
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
        self.isometry = self.next_isometry;

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
            &self.isometry,
            &self.velocity,
            self.shape(),
            &other.isometry,
            &other.velocity,
            other.shape(),
            options,
        )
        .unwrap_or(None)
    }
}
