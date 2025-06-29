//! Define a physics world containing fixed and kinematic bodies

/// Base operations available on a world
mod base;

/// Collection of objects
pub mod set;

/// Axis-Aligned Bounding Box (AABB)
pub mod aabb;

use crate::{
    object::{kinematic_body::KinematicBody, static_body::StaticBody, trigger_area::TriggerArea},
    Shared,
};
use parry::math::Real;
use set::Set;

/// Define a physics world
#[derive(Default)]
pub struct World<PK = (), PS = (), PT = ()> {
    /// Store the list of kinematic bodies
    kinematic_set: Set<KinematicBody<PK>>,

    /// Store the list of static bodies
    static_set: Set<StaticBody<PS>>,

    /// Store the list of trigger areas
    trigger_set: Set<TriggerArea<PT, PK>>,

    /// Epsilon value
    epsilon: Real,
}

impl<PK, PS, PT> World<PK, PS, PT> {
    /// Create a new world
    pub fn new(epsilon: Real) -> Self {
        Self {
            kinematic_set: Set::default(),
            static_set: Set::default(),
            trigger_set: Set::default(),
            epsilon,
        }
    }

    /// Create a new empty world with a predefined capacity
    pub fn with_capacity(
        epsilon: Real,
        cap_kinematic: usize,
        cap_static: usize,
        cap_trigger: usize,
    ) -> Self {
        Self {
            kinematic_set: Set::with_capacity(cap_kinematic),
            static_set: Set::with_capacity(cap_static),
            trigger_set: Set::with_capacity(cap_trigger),
            epsilon,
        }
    }
}

impl<PK, PS, PT> World<PK, PS, PT> {
    /// Add a kinematic body to the world
    #[inline]
    pub fn add_kinematic(&mut self, body: Shared<KinematicBody<PK>>) {
        self.kinematic_set.store(body); // don't update the partition here
    }

    /// Remove a kinematic body from the world
    #[inline]
    pub fn remove_kinematic(&mut self, body: &Shared<KinematicBody<PK>>) {
        self.kinematic_set.quick_remove(body);
    }

    /// Add a static body to the world
    #[inline]
    pub fn add_static(&mut self, body: Shared<StaticBody<PS>>) {
        self.static_set.add(body);
    }

    /// Remove a static body from the world
    #[inline]
    pub fn remove_static(&mut self, body: &Shared<StaticBody<PS>>) {
        self.static_set.clean_remove(body);
    }

    /// Add a trigger area to the world
    #[inline]
    pub fn add_trigger(&mut self, area: Shared<TriggerArea<PT, PK>>) {
        self.trigger_set.add(area);
    }

    /// Remove a trigger area from the world
    #[inline]
    pub fn remove_trigger(&mut self, area: &Shared<TriggerArea<PT, PK>>) {
        self.trigger_set.clean_remove(area);
    }

    /// Access the set of kinematic bodies
    pub fn kinematics(&self) -> &Set<KinematicBody<PK>> {
        &self.kinematic_set
    }

    /// Access the set of static bodies
    pub fn statics(&self) -> &Set<StaticBody<PS>> {
        &self.static_set
    }

    /// Access the set of trigger areas
    pub fn triggers(&self) -> &Set<TriggerArea<PT, PK>> {
        &self.trigger_set
    }

    /// Mutable access the set of kinematic bodies
    pub fn kinematics_mut(&mut self) -> &mut Set<KinematicBody<PK>> {
        &mut self.kinematic_set
    }

    /// Mutable access the set of static bodies
    pub fn statics_mut(&mut self) -> &mut Set<StaticBody<PS>> {
        &mut self.static_set
    }

    /// Mutable access the set of trigger areas
    pub fn triggers_mut(&mut self) -> &mut Set<TriggerArea<PT, PK>> {
        &mut self.trigger_set
    }
}
