use super::World;
use crate::{
    object::{
        collides, intersects, kinematic_body::KinematicBody, static_body::StaticBody,
        trigger_area::TriggerArea, Object,
    },
    world::aabb::Aabb,
    Mask, Shared,
};
use parry::{
    math::Real,
    query::{Ray, RayIntersection, ShapeCastOptions},
};

impl<B, T> World<T, B>
where
    B: Clone,
    T: Clone,
{
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
                    let astatic = astatic.read();
                    if let Some(hit) =
                        collides::<KinematicBody<B>, StaticBody<B>>(&mut_kine, &astatic, options)
                    {
                        mut_kine.add_contact(hit, None, astatic.payload().clone());
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
                    collides::<KinematicBody<B>, KinematicBody<B>>(&mut_k1, &mut_k2, options)
                {
                    mut_k1.add_contact(hit, Some(mut_k2.weight()), mut_k2.payload().clone());
                    mut_k2.add_contact(
                        hit.swapped(),
                        Some(mut_k1.weight()),
                        mut_k1.payload().clone(),
                    );
                }
            });

        // resolve actual motion using accumulated collision hits
        for kinematic in self.kinematic_set.iter_mut() {
            kinematic.write().apply_contacts(delta_time, self.epsilon);
        }

        // Check intersections between kinematic bodies and trigger areas
        for kinematic in self.kinematic_set.iter_mut() {
            // mutable access to the kinematic body
            let mut mut_kine = kinematic.write();
            let aabb = mut_kine.aabb();
            // check for intersections with trigger areas
            self.trigger_set
                .partition
                .for_each_overlaps(&aabb, |trigger| {
                    let mut trigger = trigger.write();
                    if intersects::<KinematicBody<B>, TriggerArea<T, B>>(&mut_kine, &trigger) {
                        // the kinematic body intersect with this trigger area
                        // call the callback of the trigger on both
                        trigger.on_overlap(&mut mut_kine)
                    }
                });
        }
    }
}

impl<B, T> World<T, B> {
    /// Perform a raycast with the static and/or kinematic bodies in this world
    pub fn raycast(
        &self,
        ray: &Ray,
        max_time_of_impact: Real,
        mask: Mask,
        hit_statics: bool,
        hit_kinematics: bool,
    ) -> RayResult<B> {
        // Define the AABB around the ray
        let aabb = Aabb::from_ray(ray, max_time_of_impact, mask);

        // Try to find the best candidate
        let mut found = RayResult::None;
        let mut time = Real::MAX;

        // Check static bodies
        if hit_statics {
            self.static_set.partition.for_each_overlaps(&aabb, |body| {
                let b = body.read();
                if let Some(hit) =
                    b.shape()
                        .cast_ray_and_get_normal(b.isometry(), ray, max_time_of_impact, true)
                {
                    // if the hit is closer to the origin, replace the previous result
                    if hit.time_of_impact < time {
                        time = hit.time_of_impact;
                        found = RayResult::Static {
                            hit,
                            object: body.clone(),
                        };
                    }
                }
            });
        }

        // Check kinematic bodies
        if hit_kinematics {
            self.kinematic_set
                .partition
                .for_each_overlaps(&aabb, |body| {
                    let b = body.read();
                    if let Some(hit) = b.shape().cast_ray_and_get_normal(
                        b.isometry(),
                        ray,
                        max_time_of_impact,
                        true,
                    ) {
                        // if the hit is closer to the origin, replace the previous result
                        if hit.time_of_impact < time {
                            time = hit.time_of_impact;
                            found = RayResult::Kinematic {
                                hit,
                                object: body.clone(),
                            };
                        }
                    }
                });
        }

        found
    }
}

/// Return data relative to the object that have been hit by the raycast
pub enum RayResult<P> {
    /// No object has been hit
    None,

    /// The object hit is a static body
    Static {
        /// Ray intersection data
        hit: RayIntersection,

        /// Reference to the object
        object: Shared<StaticBody<P>>,
    },

    /// The object hit is a kinematic body
    Kinematic {
        /// Ray intersection data
        hit: RayIntersection,

        /// Reference to the object
        object: Shared<KinematicBody<P>>,
    },
}
