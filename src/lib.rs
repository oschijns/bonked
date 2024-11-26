#![cfg_attr(not(feature = "std"), no_std)]

// cannot use 2D and 3D features at the same time
#[cfg(all(feature = "2d", feature = "3d"))]
compile_error!("The '2d' & '3d' features cannot be used at the same time.");

// cannot use f32 and f64 features at the same time
#[cfg(all(feature = "parry-f32", feature = "parry-f64"))]
compile_error!("The 'parry-f32' & 'parry-f64' features cannot be used at the same time.");

// cannot use u32 and u64 features at the same time
#[cfg(all(feature = "mask-u32", feature = "mask-u64"))]
compile_error!("The 'mask-u32' & 'mask-u64' features cannot be used at the same time.");

/// Use alloc crate for no_std support
extern crate alloc;

#[cfg(all(feature = "2d", feature = "parry-f32"))]
pub extern crate parry2d as parry;

#[cfg(all(feature = "2d", feature = "parry-f64"))]
pub extern crate parry2d_f64 as parry;

#[cfg(all(feature = "3d", feature = "parry-f32"))]
pub extern crate parry3d as parry;

#[cfg(all(feature = "3d", feature = "parry-f64"))]
pub extern crate parry3d_f64 as parry;

use accumulator::Accumulator;
use alloc::{boxed::Box, sync::Arc};
use parry::{
    bounding_volume::Aabb,
    math::{Isometry, Real, Vector},
    shape::Shape,
};

/// Accumulator for contact processing
pub mod accumulator;

/// Systems for processing objects
pub mod system;

#[cfg(feature = "mask-u32")]
pub type Mask = u32;

#[cfg(feature = "mask-u64")]
pub type Mask = u64;

/// Collider of the object
pub struct Collider<A> {
    /// Collision shape
    pub shape: Arc<dyn Shape>,

    /// Collision layer
    pub layer: Mask,

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

impl<A> Collider<A> {
    /// Check if the collision layer of the first object match the one of the second
    #[inline]
    pub fn can_collide_with(&self, other: &Self) -> bool {
        (self.layer & other.mask) != 0
    }
}