//! Module containing basic objects definitions.

pub mod traits;
pub mod volume;

use crate::{pointer::RefCounted, Mask};
use bvh_arena::VolumeHandle;
use parry::{
    math::{Isometry, Real, Vector},
    shape::Shape,
};

// define masks where in one no bit is set and in the other all bits are set
pub const MASK_NONE: Mask = 0;
pub const MASK_ALL: Mask = !0;

/// A simple collider object.
pub struct Collider {
    /// Layer of the collider.
    layer: Mask,

    /// Mask of the collider.
    mask: Mask,

    /// Shape of the collider.
    shape: RefCounted<dyn Shape>,

    /// Isometry of the collider.
    isometry: Isometry<Real>,

    /// Handle to the volume in the BVH.
    handle: Option<VolumeHandle>,
}

/// A collider that can move.
pub struct DynamicCollider {
    /// Simple collider object.
    base: Collider,

    /// Velocity of the collider.
    velocity: Vector<Real>,
}
