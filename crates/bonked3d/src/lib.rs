#![cfg_attr(not(feature = "std"), no_std)]

#[macro_use]
extern crate alloc;

/// Math functions
pub mod math;

/// Systems for processing objects
pub mod system;

use alloc::sync::Arc;
use parry3d::{
    bounding_volume::Aabb,
    math::{Isometry, Point, Real, UnitVector, Vector},
    shape::Shape,
};

/// Collision mask
pub type Mask = u32;

/// Collider of the object
pub struct Collider {
    /// Collision shape
    pub shape: Arc<dyn Shape>,

    /// Collision mask
    pub mask: Mask,
}

/// Current position of the object for this tick
pub struct Position(pub Isometry<Real>);

/// Position of the object for the next tick
pub struct NextPosition(pub Isometry<Real>);

/// Current velocity of the object for this tick
pub struct Velocity(pub Vector<Real>);

/// Velocity of the object for the next tick
pub struct NextVelocity(pub Vector<Real>);

/// Bounding box of the object
pub struct BoundingBox(pub Aabb);

/// Gravity force to apply to the object
pub struct Gravity(pub Vector<Real>);

/// Collision state
pub struct CollisionStatus(pub Box<dyn Accumulator>);

/// Collision accumulator
pub trait Accumulator: Send + Sync {
    /// Add the contact point and normal to this accumulator
    fn add_contact(&mut self, point: &Point<Real>, normal: &UnitVector<Real>);

    /// Add the velocity of the other object to this accumulator
    fn add_velocity(&mut self, velocity: &Vector<Real>);
}
