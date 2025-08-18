//! Allow making various geometry casts.
//! This includes point casts, ray casts and shape casts.

use crate::{
    mask::Mask,
    object::Object,
    world::{World, set::Id},
};
use parry::{math::Real, query::Ray};

/// Identify an object in the world (distinguish static from dynamic objects)
#[derive(Debug, Clone, Copy)]
pub struct Ident {
    /// Identifier of the object in its own set
    pub id: Id,

    /// Specify if it is either a static or a dynamic object
    pub is_dynamic: bool,
}

impl Ident {
    #[inline]
    pub fn new(id: Id, is_dynamic: bool) -> Self {
        Self { id, is_dynamic }
    }
}

impl World {
    /// Cast a ray in the world and report the closest object hit (if any)
    pub fn ray_cast(
        &self,
        ray: &Ray,
        mut max_time_of_impact: Real,
        filter: Mask,
    ) -> Option<(Ident, Real)> {
        // Store the id of a static object result if no dynamic object is hit
        let mut tmp_s_id: Option<Id> = None;

        // TODO: ideally, we would return a `RayIntersection` instead

        // cast a ray against static objects
        if let Some((id, dist)) =
            self.statics
                .bvh
                .cast_ray(ray, max_time_of_impact, |id, best_so_far| {
                    if let Some(obj) = self.statics.objects.get(&id) {
                        let obj = obj.borrow();
                        if !obj.is_trigger_area() && obj.pass_filter(filter) {
                            return obj.shape().cast_ray(obj.isometry(), ray, best_so_far, true);
                        }
                    }
                    None
                })
        {
            // store result for static objects
            max_time_of_impact = dist;
            tmp_s_id = Some(id);
        }

        // cast a ray against dynamic objects
        if let Some((id, dist)) =
            self.dynamics
                .bvh
                .cast_ray(ray, max_time_of_impact, |id, best_so_far| {
                    if let Some(obj) = self.dynamics.objects.get(&id) {
                        let obj = obj.borrow();
                        if !obj.is_trigger_area() && obj.pass_filter(filter) {
                            return obj.shape().cast_ray(obj.isometry(), ray, best_so_far, true);
                        }
                    }
                    None
                })
        {
            // found a dynamic object closer
            Some((Ident::new(id, true), dist))
        } else if let Some(id) = tmp_s_id {
            // no dynamic object found, fall back to the static object that was found before
            Some((Ident::new(id, false), max_time_of_impact))
        } else {
            // neither dynamic nor static object found
            None
        }
    }
}
