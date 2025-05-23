//! Simple physics engine for the game

/// Handles for physics objects
pub mod handle;

/// Static body
pub mod static_body;

/// Kinematic body
pub mod kinematic_body;

/// Trigger area
pub mod trigger_area;

use super::Mask;
use parry::{
    bounding_volume::Aabb,
    math::{Isometry, Real},
    shape::Shape,
};

/// Trait implemented for static and dynamic bodies
pub trait Object {
    /// Access the shape assigned to this body
    fn shape(&self) -> &dyn Shape;

    /// Access the isometry of this shape
    fn isometry(&self) -> &Isometry<Real>;

    /// Create an Axis-Aligned Bounding Box for this body
    fn aabb(&self) -> Aabb;

    /// Check if the collision layer matches with the other body
    fn layer_match(&self, other: &dyn Object) -> bool;

    /// Get the layer(s) this body belongs to
    #[inline]
    fn layer(&self) -> Mask {
        0
    }

    /// Get the layers this body can interact with
    #[inline]
    fn mask(&self) -> Mask {
        0
    }
}
