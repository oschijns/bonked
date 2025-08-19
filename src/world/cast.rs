//! Allow making various geometry casts.
//! This includes point casts, ray casts and shape casts.

use crate::{
    NULL_VECTOR,
    mask::Mask,
    object::Object,
    world::{Ident, World, set::Index},
};
use core::cell::RefCell;
use parry::{
    math::Real,
    query::{Ray, RayIntersection},
    shape::FeatureId,
};

impl World {
    /// Cast a ray in the world and report the closest object hit (if any)
    pub fn ray_cast(
        &self,
        ray: &Ray,
        mut max_time_of_impact: Real,
        filter: Mask,
    ) -> Option<(Ident, RayIntersection)> {
        // Store the id of a static object result if no dynamic object is hit
        let mut tmp_st_idx: Option<Index> = None;

        // Store ray intersection extra data in a temporary variable.
        let tmp_hit = RefCell::new(RayIntersection::new(
            max_time_of_impact,
            NULL_VECTOR,
            FeatureId::Unknown,
        ));

        // cast a ray against static objects
        if let Some((idx, dist)) =
            self.statics
                .bvh
                .cast_ray(ray, max_time_of_impact, |idx, best_so_far| {
                    // Access the static object data and check that it passes the filter.
                    if let Some(obj) = self.statics.objects.get(&idx) {
                        let obj = obj.borrow();
                        if !obj.is_trigger_area() && obj.pass_filter(filter) {
                            // Perform a raycast against the object.
                            if let Some(hit) = obj.shape().cast_ray_and_get_normal(
                                obj.isometry(),
                                ray,
                                best_so_far,
                                true,
                            ) {
                                tmp_hit.replace(hit);
                                return Some(hit.time_of_impact);
                            }
                        }
                    }
                    None
                })
        {
            // store result for static objects
            max_time_of_impact = dist;
            tmp_st_idx = Some(idx);
        }

        // cast a ray against dynamic objects
        if let Some((idx, _)) =
            self.dynamics
                .bvh
                .cast_ray(ray, max_time_of_impact, |idx, best_so_far| {
                    // Access the dynamic object data and check that it passes the filter.
                    if let Some(obj) = self.dynamics.objects.get(&idx) {
                        let obj = obj.borrow();
                        if !obj.is_trigger_area() && obj.pass_filter(filter) {
                            // Perform a raycast against the object.
                            if let Some(hit) = obj.shape().cast_ray_and_get_normal(
                                obj.isometry(),
                                ray,
                                best_so_far,
                                true,
                            ) {
                                tmp_hit.replace(hit);
                                return Some(hit.time_of_impact);
                            }
                        }
                    }
                    None
                })
        {
            // found a dynamic object closer
            Some((Ident::new(idx, true), tmp_hit.into_inner()))
        } else if let Some(idx) = tmp_st_idx {
            // no dynamic object found, fall back to the static object that was found before
            Some((Ident::new(idx, false), tmp_hit.into_inner()))
        } else {
            // neither dynamic nor static object found
            None
        }
    }
}
