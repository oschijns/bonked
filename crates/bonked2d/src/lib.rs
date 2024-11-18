#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

/// Accumulator for contact processing
pub mod accumulator;

/// Math functions
pub mod math;

/// Systems for processing objects
pub mod system;

use accumulator::Accumulator;
use alloc::{boxed::Box, sync::Arc};
use parry2d::{
    bounding_volume::Aabb,
    math::{Isometry, Real, Vector},
    shape::Shape,
};

/// Collision mask
pub type Mask = u32;

/// Collider of the object
pub struct Collider<A> {
    /// Collision shape
    pub shape: Arc<dyn Shape>,

    /// Collision mask
    pub mask: Mask,

    /// Attributes
    pub attributes: A,
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
pub struct CollisionStatus<A>(pub Box<dyn Accumulator<A>>);
