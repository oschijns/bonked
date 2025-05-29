//! Define a physics world containing fixed and kinematic bodies

/// Collection of objects
pub mod set;

use super::object::{
    handle::Handle, kinematic_body::KinematicBody, static_body::StaticBody,
    trigger_area::TriggerArea, Object,
};
use crate::collections::HashSet;
use alloc::{sync::Arc, vec::Vec};
use delegate::delegate;
use parry::{
    math::Real,
    partitioning::{Qbvh, QbvhUpdateWorkspace},
    query::visitors::BoundingVolumeIntersectionsSimultaneousVisitor,
};
use set::Set;

/// Define a physics world
#[derive(Default)]
pub struct World {
    /// Store the list of trigger areas
    trigger_set: Set<TriggerArea>,

    /// Store the list of static bodies
    static_set: Set<StaticBody>,

    /// Store the list of kinematic bodies
    kinematic_set: Set<KinematicBody>,

    /// List of the handles
    pub handles: HashSet<Handle>,

    /// Broadphase resolution
    qbvh: Qbvh<Handle>,

    /// Workspace for the world
    workspace: QbvhUpdateWorkspace,

    /// Stack to store stuff?
    stack: Vec<(u32, u32)>,

    /// Margin for broadphase resolution
    margin: Real,
}

impl World {
    /// Create a new empty world with a predefined capacity
    pub fn with_capacity(cap_static: usize, cap_kinematic: usize, cap_trigger: usize) -> Self {
        Self {
            static_set: Set::with_capacity(cap_static),
            kinematic_set: Set::with_capacity(cap_kinematic),
            trigger_set: Set::with_capacity(cap_trigger),
            handles: HashSet::with_capacity(cap_static + cap_kinematic + cap_trigger),
            qbvh: Qbvh::new(),
            workspace: QbvhUpdateWorkspace::default(),
            stack: Vec::new(),
            margin: Real::EPSILON,
        }
    }

    //*
    /// Proceed to the broadphase
    pub fn update(&mut self) {
        // Count the total number of kinematic bodies in the physics world
        let count = self.kinematic_set.len();

        // This section is inspired by rapier's implementation
        // https://docs.rs/rapier3d/0.26.0/src/rapier3d/geometry/broad_phase_qbvh.rs.html

        // Define set for each of the various collision types
        let mut intersect = Vec::with_capacity(count);
        let mut coll_s = Vec::with_capacity(count);
        let mut coll_k = Vec::with_capacity(count);

        // Visitor to find collision pairs
        let mut visitor =
            BoundingVolumeIntersectionsSimultaneousVisitor::new(|h1: &Handle, h2: &Handle| {
                match (h1, h2) {
                    (Handle::Trigger(t), Handle::Kinematic(k))
                    | (Handle::Kinematic(k), Handle::Trigger(t)) => {
                        let trigger = unsafe { t.as_ref() }.unwrap();
                        let k_body = unsafe { k.as_ref() }.unwrap();
                        if trigger.layer_match(k_body) {
                            intersect.push((trigger, k_body));
                        }
                    }
                    (Handle::Static(s), Handle::Kinematic(k))
                    | (Handle::Kinematic(k), Handle::Static(s)) => {
                        let s_body = unsafe { s.as_ref() }.unwrap();
                        let k_body = unsafe { k.as_ref() }.unwrap();
                        if s_body.layer_match(k_body) {
                            coll_s.push((s_body, k_body));
                        }
                    }
                    (Handle::Kinematic(k1), Handle::Kinematic(k2)) => {
                        let kine1 = unsafe { k1.as_ref() }.unwrap();
                        let kine2 = unsafe { k2.as_ref() }.unwrap();
                        if kine1.layer_match(kine2) {
                            coll_k.push((kine1, kine2));
                        }
                    }
                    _ => {}
                }
                true
            });

        // check if we need to perform a full rewrite
        let full_rebuild = self.qbvh.raw_nodes().is_empty();
        if full_rebuild {
            // Rebuild all the AABBs
            self.qbvh.clear_and_rebuild(
                self.handles.iter().map(|handle| {
                    let aabb = handle.as_dyn().aabb();
                    (*handle, aabb)
                }),
                self.margin,
            );

            // traverse
            self.qbvh
                .traverse_bvtt_with_stack(&self.qbvh, &mut visitor, &mut self.stack);
        } else {
            // update the AABB of the colliders that have been modified
            let _ = self.qbvh.refit(self.margin, &mut self.workspace, |handle| {
                handle.as_dyn().aabb()
            });

            // traverse
            self.qbvh
                .traverse_modified_bvtt_with_stack(&self.qbvh, &mut visitor, &mut self.stack);
            self.qbvh.rebalance(self.margin, &mut self.workspace);
        }

        // TODO: perform the narrow phase
    }
    // */
}

macro_rules! impl_for_body_set {
    ( $body_set:ident [$body_type:ty] as ( $add:ident, $remove:ident ) ) => {
        impl World {
            /// Add a fixed body to the world
            pub fn $add(&mut self, body: Arc<$body_type>) {
                let handle = self.$body_set.add(body);
                self.handles.insert(handle);
                //self.qbvh.pre_update_or_insert(handle);
            }

            /// Remove a fixed body from the world
            pub fn $remove(&mut self, body: Arc<$body_type>) {
                if let Some(handle) = self.$body_set.remove(body) {
                    self.handles.remove(&handle);
                    //self.qbvh.remove(handle);
                }
            }
        }
    };
}

// Implement methods for trigger areas
impl_for_body_set! {
    trigger_set[TriggerArea] as (add_trigger, remove_trigger)
}

// Implement methods for static bodies
impl_for_body_set! {
    static_set[StaticBody] as (add_static, remove_static)
}

// Implement methods for kinematic bodies
impl_for_body_set! {
    kinematic_set[KinematicBody] as (add_kinematic, remove_kinematic)
}

// Expose some methods from the underlying vector
impl World {
    // delegate methods for static bodies set
    delegate! {
        to self.static_set {
            #[call(reserve)]
            pub fn reserve_statics(&mut self, additional: usize);
            #[call(reserve_exact)]
            pub fn reserve_exact_statics(&mut self, additional: usize);
            #[call(shrink_to_fit)]
            pub fn shrink_to_fit_statics(&mut self);
            #[call(shrink_to)]
            pub fn shrink_to_statics(&mut self, min_capacity: usize);
            #[call(iter)]
            pub fn iter_statics(&self) -> impl Iterator<Item = &Arc<StaticBody>>;
        }
    }

    // delegate methods for kinematic bodies set
    delegate! {
        to self.kinematic_set {
            #[call(reserve)]
            pub fn reserve_kinematics(&mut self, additional: usize);
            #[call(reserve_exact)]
            pub fn reserve_exact_kinematics(&mut self, additional: usize);
            #[call(shrink_to_fit)]
            pub fn shrink_to_fit_kinematics(&mut self);
            #[call(shrink_to)]
            pub fn shrink_to_kinematics(&mut self, min_capacity: usize);
            #[call(iter)]
            pub fn iter_kinematics(&self) -> impl Iterator<Item = &Arc<KinematicBody>>;
            #[call(iter_mut)]
            pub fn iter_mut_kinematics(&mut self) -> impl Iterator<Item = &mut Arc<KinematicBody>>;
        }
    }

    // delegate methods for trigger areas set
    delegate! {
        to self.trigger_set {
            #[call(reserve)]
            pub fn reserve_triggers(&mut self, additional: usize);
            #[call(reserve_exact)]
            pub fn reserve_exact_triggers(&mut self, additional: usize);
            #[call(shrink_to_fit)]
            pub fn shrink_to_fit_triggers(&mut self);
            #[call(shrink_to)]
            pub fn shrink_to_triggers(&mut self, min_capacity: usize);
            #[call(iter)]
            pub fn iter_triggers(&self) -> impl Iterator<Item = &Arc<TriggerArea>>;
        }
    }
}
