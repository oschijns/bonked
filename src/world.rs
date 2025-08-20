//! Define a physics world containing fixed and kinematic bodies

/// Collection of objects
pub mod set;

/// Iterators for physics events
pub mod iter;

/// Handle world state update
mod update;

/// Allow making various geometry queries
mod query;

use crate::object::{DynamicObject, StaticObject};
use alloc::vec::Vec;
use parry::{partitioning::BvhWorkspace, query::ShapeCastHit};
use set::{Index, Set};

/// Define a physics world
#[derive(Default)]
pub struct World {
    /// Store the list of static bodies
    statics: Set<StaticObject>,

    /// Store the list of dynamic objects
    dynamics: Set<DynamicObject>,

    /// Allocate a workspace for broadphase processing
    workspace: BvhWorkspace,

    /// Intersection between a dynamic body and a trigger area
    on_trigger: Vec<OnContact>,

    /// Collision result between two physics bodies
    on_collision: Vec<OnContact<ShapeCastHit>>,
}

/// Result of a RayCast call
#[derive(Debug, Clone, Copy)]
pub struct Ident {
    /// Identifier of the object in its own set
    pub index: Index,

    /// Specify if it is either a static or a dynamic object
    pub is_dynamic: bool,
}

/// Store data for either trigger events or collision events
struct OnContact<D = ()> {
    /// Identifier of the first object (necessarily dynamic)
    index1: Index,

    /// Identifier of the second object (can be either static or dynamic)
    ident2: Ident,

    /// Extra data (ShapeCastHit)
    data: D,
}

impl World {
    /// Create a new world
    pub fn new() -> Self {
        Self {
            statics: Set::new(),
            dynamics: Set::new(),
            workspace: BvhWorkspace::default(),
            on_trigger: Vec::new(),
            on_collision: Vec::new(),
        }
    }

    /// Create a new empty world with a predefined capacity
    pub fn with_capacity(cap_static: usize, cap_dynamic: usize) -> Self {
        Self {
            statics: Set::with_capacity(cap_static),
            dynamics: Set::with_capacity(cap_dynamic),
            workspace: BvhWorkspace::default(),
            on_trigger: Vec::with_capacity(cap_dynamic),
            on_collision: Vec::with_capacity(cap_dynamic),
        }
    }

    /// Access the set of static objects
    #[inline]
    pub fn statics(&self) -> &Set<StaticObject> {
        &self.statics
    }

    /// Access the set of static objects mutably
    #[inline]
    pub fn statics_mut(&mut self) -> &mut Set<StaticObject> {
        &mut self.statics
    }

    /// Access the set of dynamic objects
    #[inline]
    pub fn dynamics(&self) -> &Set<DynamicObject> {
        &self.dynamics
    }

    /// Access the set of dynamic objects mutably
    #[inline]
    pub fn dynamics_mut(&mut self) -> &mut Set<DynamicObject> {
        &mut self.dynamics
    }
}

impl Ident {
    /// Create a new identifier for an object the the world
    #[inline]
    pub fn new(index: Index, is_dynamic: bool) -> Self {
        Self { index, is_dynamic }
    }
}

impl<D> OnContact<D> {
    /// Create a new on contact event
    #[inline]
    pub fn new(index1: Index, ident2: Ident, data: D) -> Self {
        Self {
            index1,
            ident2,
            data,
        }
    }
}
