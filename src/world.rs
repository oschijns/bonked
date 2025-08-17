//! Define a physics world containing fixed and kinematic bodies

/// Collection of objects
pub mod set;

/// Handle world state update
mod update;

use crate::{
    object::{DynamicObject, StaticObject},
    world::set::Id,
};
use alloc::vec::Vec;
use parry::{math::Real, partitioning::BvhWorkspace, query::ShapeCastHit};
use set::Set;

/// Define a physics world
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

    /// Epsilon value
    epsilon: Real,
}

impl World {
    /// Create a new world
    pub fn new(epsilon: Real) -> Self {
        Self {
            statics: Set::new(),
            dynamics: Set::new(),
            workspace: BvhWorkspace::default(),
            on_trigger: Vec::new(),
            on_collision: Vec::new(),
            epsilon,
        }
    }

    /// Create a new empty world with a predefined capacity
    pub fn with_capacity(epsilon: Real, cap_static: usize, cap_dynamic: usize) -> Self {
        Self {
            statics: Set::with_capacity(cap_static),
            dynamics: Set::with_capacity(cap_dynamic),
            workspace: BvhWorkspace::default(),
            on_trigger: Vec::with_capacity(cap_dynamic),
            on_collision: Vec::with_capacity(cap_dynamic),
            epsilon,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct OnContact<D = ()> {
    /// Identifier of the first object (necessarily dynamic)
    id1: Id,

    /// Identifier of the second object (can be either static or dynamic)
    id2: Id,

    /// Is the second object dynamic or static
    dynamic2: bool,

    /// Extra data (ShapeCastHit)
    data: D,
}

impl<D> OnContact<D> {
    pub fn new(id1: Id, id2: Id, dynamic2: bool, data: D) -> Self {
        Self {
            id1,
            id2,
            dynamic2,
            data,
        }
    }
}
