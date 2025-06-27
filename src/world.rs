//! Define a physics world containing fixed and kinematic bodies

/// Collection of objects
pub mod set;

/// Axis-Aligned Bounding Box (AABB)
pub mod aabb;

use crate::{
    object::{
        collides, intersects, kinematic_body::KinematicBody, static_body::StaticBody,
        trigger_area::TriggerArea, Object,
    },
    Shared,
};
use parry::{math::Real, query::ShapeCastOptions};
use set::Set;

/// Define a physics world
#[derive(Default)]
pub struct World {
    /// Store the list of kinematic bodies
    kinematic_set: Set<KinematicBody>,

    /// Store the list of static bodies
    static_set: Set<StaticBody>,

    /// Store the list of trigger areas
    trigger_set: Set<TriggerArea>,

    /// Epsilon value
    epsilon: Real,
}

impl World {
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

    /// Update the state of the world
    pub fn update(&mut self, delta_time: Real) {
        // Options for kinematic bodies collisions
        let options = ShapeCastOptions::with_max_time_of_impact(delta_time);

        // Check collisions between kinematic bodies and static bodies
        for kinematic in self.kinematic_set.iter_mut() {
            // prepare the  kinematic body for current update
            let mut mut_kine = kinematic.write();
            mut_kine.pre_update(delta_time);
            let aabb = mut_kine.aabb();

            // check for collisions with static bodies
            self.static_set
                .partition
                .for_each_overlaps(&aabb, |astatic| {
                    // if there is a contact between the two bodies,
                    // apply the result to the kinematic body
                    if let Some(hit) =
                        collides::<KinematicBody, StaticBody>(&mut_kine, &astatic.read(), options)
                    {
                        mut_kine.add_hit(hit, None);
                    }
                });
        }

        // Check collisions inbetween kinematic bodies
        self.kinematic_set.repartition();
        self.kinematic_set
            .partition
            .for_each_overlaping_pair(|kinematic1, kinematic2| {
                // get mutable access to both bodies
                let mut mut_k1 = kinematic1.write();
                let mut mut_k2 = kinematic2.write();

                if let Some(hit) =
                    collides::<KinematicBody, KinematicBody>(&mut_k1, &mut_k2, options)
                {
                    mut_k1.add_hit(hit, Some(mut_k2.weight()));
                    mut_k2.add_hit(hit.swapped(), Some(mut_k1.weight()));
                }
            });

        // resolve actual motion using accumulated collision hits
        for kinematic in self.kinematic_set.iter_mut() {
            kinematic.write().apply_hits(delta_time, self.epsilon);
        }

        // Check intersections between kinematic bodies and trigger areas
        for kinematic in self.kinematic_set.iter_mut() {
            // mutable access to the kinematic body
            let mut mut_kine = kinematic.write();
            let aabb = mut_kine.aabb();
            // check for intersections with trigger areas
            self.trigger_set
                .partition
                .for_each_overlaps(&aabb, |atrigger| {
                    // if there is an intersection between the two areas,
                    // apply the result to the kinematic body
                    if intersects::<KinematicBody, TriggerArea>(&mut_kine, &atrigger.read()) {
                        // TODO
                    }
                });
        }
    }

    /// Add a kinematic body to the world
    #[inline]
    pub fn add_kinematic(&mut self, body: Shared<KinematicBody>) {
        self.kinematic_set.store(body); // don't update the partition here
    }

    /// Remove a kinematic body from the world
    #[inline]
    pub fn remove_kinematic(&mut self, body: &Shared<KinematicBody>) {
        self.kinematic_set.quick_remove(body);
    }

    /// Add a static body to the world
    #[inline]
    pub fn add_static(&mut self, body: Shared<StaticBody>) {
        self.static_set.add(body);
    }

    /// Remove a static body from the world
    #[inline]
    pub fn remove_static(&mut self, body: &Shared<StaticBody>) {
        self.static_set.clean_remove(body);
    }

    /// Add a trigger area to the world
    #[inline]
    pub fn add_trigger(&mut self, area: Shared<TriggerArea>) {
        self.trigger_set.add(area);
    }

    /// Remove a trigger area from the world
    #[inline]
    pub fn remove_trigger(&mut self, area: &Shared<TriggerArea>) {
        self.trigger_set.clean_remove(area);
    }

    /// Access the set of kinematic bodies
    pub fn kinematics(&self) -> &Set<KinematicBody> {
        &self.kinematic_set
    }

    /// Access the set of static bodies
    pub fn statics(&self) -> &Set<StaticBody> {
        &self.static_set
    }

    /// Access the set of trigger areas
    pub fn triggers(&self) -> &Set<TriggerArea> {
        &self.trigger_set
    }

    /// Mutable access the set of kinematic bodies
    pub fn kinematics_mut(&mut self) -> &mut Set<KinematicBody> {
        &mut self.kinematic_set
    }

    /// Mutable access the set of static bodies
    pub fn statics_mut(&mut self) -> &mut Set<StaticBody> {
        &mut self.static_set
    }

    /// Mutable access the set of trigger areas
    pub fn triggers_mut(&mut self) -> &mut Set<TriggerArea> {
        &mut self.trigger_set
    }
}
