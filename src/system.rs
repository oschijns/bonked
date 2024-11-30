use crate::{
    BoundingBox, CanCollideWith, Collider, CollisionStatus, Gravity, NextPosition, NextVelocity,
    Position, RayCaster, Velocity, NULL_VEL,
};
use hecs::{Entity, PreparedQuery, With, Without, World};
use parry::{
    bounding_volume::BoundingVolume,
    math::{Real, Translation},
    query::{contact, RayIntersection},
};

/// Store prepared queries to be applied to a world
pub struct Querier<'q, A: 'static + Send + Sync> {
    /// The delta time used for computation
    delta_time: Real,

    /// Recopy the "next" position to the "current" position to prepare for the next tick
    recopy_positions: PreparedQuery<(&'q NextPosition, &'q mut Position)>,

    /// Recopy the "next" velocity to the "current" velocity to prepare for the next tick
    recopy_velocities: PreparedQuery<(&'q NextVelocity, &'q mut Velocity)>,

    /// Reset the collision status
    reset_status: PreparedQuery<(&'q Position, &'q Velocity, &'q mut CollisionStatus<A>)>,

    /// Update the bounding-box of static objects
    compute_static_boxes:
        PreparedQuery<Without<(&'q Collider<A>, &'q Position, &'q mut BoundingBox), &'q Velocity>>,

    /// Update the swept-box of objects
    recompute_swept_boxes: PreparedQuery<(
        &'q Collider<A>,
        &'q Position,
        &'q Velocity,
        &'q mut BoundingBox,
        &'q mut NextPosition,
    )>,

    /// Process moving objects
    process_kinematics: PreparedQuery<
        With<
            (
                &'q Collider<A>,
                &'q NextPosition,
                &'q BoundingBox,
                &'q mut CollisionStatus<A>,
            ),
            &'q Velocity,
        >,
    >,

    /// Get static objects in read-only
    get_statics:
        PreparedQuery<Without<(&'q Collider<A>, &'q Position, &'q BoundingBox), &'q Velocity>>,

    /// Process moving objects
    get_kinematics: PreparedQuery<(&'q Collider<A>, &'q Position, &'q Velocity, &'q BoundingBox)>,

    /// Use collision status to resolve object placement and velocity
    process_status: PreparedQuery<(
        &'q CollisionStatus<A>,
        &'q Velocity,
        &'q mut NextPosition,
        &'q mut NextVelocity,
    )>,

    /// Apply the gravity to the next velocity
    process_gravity: PreparedQuery<(&'q Gravity, &'q mut NextVelocity)>,

    /// Perform a raycast against physics bodies in the world
    do_raycast: PreparedQuery<(&'q Collider<A>, &'q Position, &'q BoundingBox)>,
}

impl<'q, A: Send + Sync> Querier<'q, A> {
    /// Create a new querier
    pub fn new(delta_time: Real) -> Self {
        Self {
            delta_time,
            recopy_positions: Default::default(),
            recopy_velocities: Default::default(),
            reset_status: Default::default(),
            compute_static_boxes: Default::default(),
            recompute_swept_boxes: Default::default(),
            process_kinematics: Default::default(),
            get_statics: Default::default(),
            get_kinematics: Default::default(),
            process_gravity: Default::default(),
            process_status: Default::default(),
            do_raycast: Default::default(),
        }
    }

    /// Get the delta time set for this physics simulation
    #[inline]
    pub fn get_delta_time(&self) -> Real {
        self.delta_time
    }

    /// Recopy "next" state to "current" state for next tick
    pub fn prepare_next_tick(&mut self, world: &mut World) {
        for (_, (next, current)) in self.recopy_positions.query_mut(world) {
            current.0 = next.0;
        }
        for (_, (next, current)) in self.recopy_velocities.query_mut(world) {
            current.0 = next.0;
        }
        for (_, (pos, vel, status)) in self.reset_status.query_mut(world) {
            status.0.reset(&pos.0, &vel.0);
        }
    }

    /// Compute the bounding boxes of static objects
    pub fn compute_bounding_boxes(&mut self, world: &mut World) {
        for (_, (collider, position, bounding_box)) in self.compute_static_boxes.query_mut(world) {
            bounding_box.0 = collider.shape.compute_aabb(&position.0);
        }
    }

    /// Recompute the bounding boxes of objects
    pub fn recompute_swept_boxes(&mut self, world: &mut World) {
        for (_, (collider, position, velocity, bounding_box, next_pos)) in
            self.recompute_swept_boxes.query_mut(world)
        {
            let mut end_pos = position.0;
            end_pos.append_translation_mut(&Translation::from(velocity.0 * self.delta_time));
            next_pos.0 = end_pos;
            bounding_box.0 = collider.shape.compute_swept_aabb(&position.0, &next_pos.0);
        }
    }

    /// Compute collisions between kinematic and static objects
    pub fn compute_collisions_with_statics(&mut self, world: &mut World) {
        for (id1, (coll1, next_pos1, box1, stat1)) in self.process_kinematics.query(world).iter() {
            for (id2, (coll2, pos2, box2)) in self.get_statics.query(world).iter() {
                // if the two objects are different (should always be true)
                // and their bounding boxes overlap
                if id1 != id2 && coll1.can_collide_with(coll2) && box1.0.intersects(&box2.0) {
                    match contact(
                        &next_pos1.0,
                        coll1.shape.as_ref(),
                        &pos2.0,
                        coll2.shape.as_ref(),
                        0.0,
                    ) {
                        Ok(Some(contact)) => {
                            stat1.0.add_contact(&contact, &NULL_VEL, &coll2.attributes);
                        }
                        Ok(None) => {}
                        Err(unsupported) => {
                            panic!["{}", unsupported];
                        }
                    }
                }
            }
        }
    }

    /// Compute collisions between kinematic objects
    pub fn compute_collisions_with_kinematics(&mut self, world: &mut World) {
        // count the number of entities that have been processed
        for (id1, (coll1, next_pos1, box1, stat1)) in self.process_kinematics.query(world).iter() {
            // skip the entities that have been already processed
            for (id2, (coll2, pos2, vel2, box2)) in self.get_kinematics.query(world).iter() {
                if id1 != id2 && coll1.can_collide_with(coll2) && box1.0.intersects(&box2.0) {
                    match contact(
                        &next_pos1.0,
                        coll1.shape.as_ref(),
                        &pos2.0,
                        coll2.shape.as_ref(),
                        0.0,
                    ) {
                        Ok(Some(contact)) => {
                            stat1.0.add_contact(&contact, &vel2.0, &coll2.attributes);
                        }
                        Ok(None) => {}
                        Err(unsupported) => {
                            panic!["{}", unsupported];
                        }
                    }
                }
            }
        }
    }

    /// Use the collision status to deduce how to position the objects
    pub fn apply_collision_status(&mut self, world: &mut World) {
        for (_, (status, vel, next_pos, next_vel)) in self.process_status.query_mut(world) {
            // Should the position be overriden ?
            if let Some(new_pos) = status.0.get_position() {
                next_pos.0 = new_pos;
            }

            // should the velocity be overriden ?
            if let Some(new_vel) = status.0.get_velocity() {
                next_vel.0 = new_vel;
            } else {
                next_vel.0 = vel.0;
            }
        }
    }

    /// Apply the gravity
    pub fn apply_gravity(&mut self, world: &mut World) {
        for (_, (grav, next_vel)) in self.process_gravity.query_mut(world) {
            next_vel.0 += grav.0 * self.delta_time;
        }
    }

    /// Apply a raycast to all physics bodies in the world
    pub fn raycast(
        &mut self,
        world: &mut World,
        ray: &RayCaster,
    ) -> Option<(Entity, RayIntersection)> {
        // identify the closest object hit
        let mut found = None;
        let mut max_dist = Real::MAX;

        // iterate over all the physics object in the world
        for (id, (collider, position, bounding_box)) in self.do_raycast.query_mut(world) {
            if ray.can_collide_with(collider) && ray.internal.aabb.intersects(&bounding_box.0) {
                if let Some(hit) = collider.shape.cast_ray_and_get_normal(
                    &position.0,
                    &ray.internal.ray,
                    ray.internal.length,
                    ray.solid,
                ) {
                    // elect this object as the closest hit object
                    if hit.time_of_impact < max_dist {
                        found = Some((id, hit));
                        max_dist = hit.time_of_impact;
                    }
                }
            }
        }
        found
    }
}
