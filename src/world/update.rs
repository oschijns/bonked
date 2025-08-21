//! Update the state of the world every frame

use super::{OnContact, World};
use crate::{NULL_VECTOR, object::Object, world::Ident};
use parry::{
    math::Real,
    query::{self, ShapeCastOptions},
};

impl World {
    pub fn start(&mut self) {
        self.statics.bvh.refit(&mut self.workspace);
    }

    pub fn update(&mut self, options: ShapeCastOptions, change_detection_margin: Real) {
        // Prepare the new update.
        self.on_trigger.clear();
        self.on_collision.clear();
        for (&idx, obj) in self.dynamics.objects.iter() {
            let mut obj = obj.borrow_mut();
            obj.pre_update(options.max_time_of_impact);
            self.dynamics
                .bvh
                .insert_or_update_partially(obj.aabb(), idx, change_detection_margin);
        }
        self.dynamics.bvh.refit(&mut self.workspace);

        // Broad-phase for dynamic objects against static objects.
        let leaf_pairs = self
            .dynamics
            .bvh
            .leaf_pairs(&self.statics.bvh, |d_node, s_node| {
                d_node.intersects(s_node)
            });

        // For each pair of leaves.
        for (d_idx, s_idx) in leaf_pairs {
            // If the BVH trees contains IDs not bound to objects, this will panic!
            let mut d_obj = self.dynamics.objects.get(&d_idx).unwrap().borrow_mut();
            let s_obj = self.statics.get(s_idx).unwrap();

            // Check that the dynamic object is not a trigger.
            // Trigger areas wherever they are dynamic or static should only detect dynamic bodies.
            if !d_obj.is_trigger_area() && d_obj.layer_filter().can_interact(*s_obj.layer_filter())
            {
                // Narrow-phase for completing filtering the pairs.
                if s_obj.is_trigger_area() {
                    // Check for intersection between the dynamic body and a trigger area.
                    if query::intersection_test(
                        d_obj.isometry(),
                        d_obj.shape(),
                        s_obj.isometry(),
                        s_obj.shape(),
                    )
                    .unwrap_or(false)
                    {
                        self.on_trigger
                            .push(OnContact::new(d_idx, Ident::new(s_idx, false), ()));
                    }
                } else if let Some(hit) = query::cast_shapes(
                    d_obj.isometry(),
                    &d_obj.velocity(),
                    d_obj.shape(),
                    s_obj.isometry(),
                    &NULL_VECTOR,
                    s_obj.shape(),
                    options,
                )
                .unwrap_or(None)
                {
                    // Collision between a dynamic body and a static body.
                    let iso = d_obj.isometry() * hit.normal2;
                    d_obj.apply_hit(hit.time_of_impact, &iso, None);

                    // Store the result for later use
                    self.on_collision
                        .push(OnContact::new(d_idx, Ident::new(s_idx, false), hit));
                }
            }
        }

        // Broad-phase for dynamic objects only
        self.dynamics.bvh.traverse_bvtt_single_tree::<true>(
            &mut self.workspace,
            &mut |id1, id2| {
                // If the BVH trees contains IDs not bound to objects, this will panic!
                let mut obj1 = self.dynamics.objects.get(&id1).unwrap().borrow_mut();
                let mut obj2 = self.dynamics.objects.get(&id2).unwrap().borrow_mut();

                // Check that the collision masks allow for interactions.
                if obj1.layer_filter().can_interact(*obj2.layer_filter()) {
                    // If either object is a trigger area, the result will be stored as an on trigger event
                    if obj1.is_trigger_area() || obj2.is_trigger_area() {
                        // Check for intersection between the dynamic body and a trigger area.
                        if query::intersection_test(
                            obj1.isometry(),
                            obj1.shape(),
                            obj2.isometry(),
                            obj2.shape(),
                        )
                        .unwrap_or(false)
                        {
                            self.on_trigger
                                .push(OnContact::new(id1, Ident::new(id2, true), ()));
                        }
                    } else if let Some(hit) = query::cast_shapes(
                        obj1.isometry(),
                        &obj1.velocity(),
                        obj1.shape(),
                        obj2.isometry(),
                        &obj2.velocity(),
                        obj2.shape(),
                        options,
                    )
                    .unwrap_or(None)
                    {
                        let nrm1 = obj1.isometry() * hit.normal1;
                        let nrm2 = obj2.isometry() * hit.normal2;

                        // Apply the collision push back to both objects
                        obj1.apply_hit(hit.time_of_impact, &nrm2, Some(obj2.weight()));
                        obj2.apply_hit(hit.time_of_impact, &nrm1, Some(obj1.weight()));

                        // Store the result for later use
                        self.on_collision
                            .push(OnContact::new(id1, Ident::new(id2, true), hit));
                    }
                }
            },
        );
    }
}
