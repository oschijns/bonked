//! Allow making various geometry casts.
//! This includes point casts, ray casts and shape casts.

use crate::{
    NULL_VECTOR,
    mask::Mask,
    object::Object,
    world::{Ident, World, set::Index},
};
use alloc::vec::Vec;
use core::cell::{Ref, RefCell};
use parry::{
    math::{Isometry, Point, Real},
    query::{self, PointProjection, Ray, RayIntersection},
    shape::{FeatureId, Shape},
};

impl World {
    /// Test for intersection between the world and some shape.
    /// Populate the provided `results` vector that can be reused to avoid reallocations.
    pub fn intersect_shape(
        &self,
        shape: &dyn Shape,
        isometry: &Isometry<Real>,
        filter: Mask,
        detect_triggers: bool,
        results: &mut Vec<Ident>,
    ) {
        // Find potential intersections using the AABB
        let aabb = shape.compute_aabb(isometry);

        // Operations to perform on both static and dynamic objects
        let check_object = |obj: Ref<'_, dyn Object>| {
            let solid = !obj.is_trigger_area();
            (detect_triggers || solid)
                && obj.pass_filter(filter)
                && query::intersection_test(obj.isometry(), obj.shape(), isometry, shape)
                    .unwrap_or(false)
        };

        // Check for intersections with static objects
        for idx in self.statics.intersect_aabb(&aabb) {
            if let Some(obj) = self.statics.get(idx) {
                if check_object(obj) {
                    results.push(Ident::new(idx, false));
                }
            }
        }

        // Check for intersections with dynamic objects
        for idx in self.dynamics.intersect_aabb(&aabb) {
            if let Some(obj) = self.dynamics.get(idx) {
                if check_object(obj) {
                    results.push(Ident::new(idx, true));
                }
            }
        }
    }

    /// Cast a ray in the world and report the closest object hit (if any).
    pub fn ray_cast(
        &self,
        ray: &Ray,
        mut max_time_of_impact: Real,
        filter: Mask,
        detect_triggers: bool,
    ) -> Option<(Ident, RayIntersection)> {
        // Store the id of a static object result if no dynamic object is hit
        let mut tmp_st_idx: Option<Index> = None;

        // Store ray intersection extra data in a temporary variable.
        let tmp_hit = RefCell::new(RayIntersection::new(
            max_time_of_impact,
            NULL_VECTOR,
            FeatureId::Unknown,
        ));

        // Repeatable raycast process for both static and dynamic objects.
        let check_object = |obj: Ref<'_, dyn Object>, best_so_far: Real| {
            // Check if the object pass the filter.
            let solid = !obj.is_trigger_area();
            if (detect_triggers || solid) && obj.pass_filter(filter) {
                // Perform a raycast against the object.
                if let Some(hit) =
                    obj.shape()
                        .cast_ray_and_get_normal(obj.isometry(), ray, best_so_far, solid)
                {
                    tmp_hit.replace(hit);
                    return Some(hit.time_of_impact);
                }
            }
            None
        };

        // cast a ray against static objects
        if let Some((idx, dist)) =
            self.statics
                .cast_ray(ray, max_time_of_impact, |idx, best_so_far| {
                    // Access the static object data and check that it passes the filter.
                    if let Some(obj) = self.statics.get(idx) {
                        check_object(obj, best_so_far)
                    } else {
                        None
                    }
                })
        {
            // store result for static objects
            max_time_of_impact = dist;
            tmp_st_idx = Some(idx);
        }

        // cast a ray against dynamic objects
        if let Some((idx, _)) =
            self.dynamics
                .cast_ray(ray, max_time_of_impact, |idx, best_so_far| {
                    // Access the dynamic object data and check that it passes the filter.
                    if let Some(obj) = self.dynamics.get(idx) {
                        check_object(obj, best_so_far)
                    } else {
                        None
                    }
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

    /// Project a point in the world and find the closest object to it.
    pub fn project_point(
        &self,
        point: &Point<Real>,
        mut max_distance: Real,
        filter: Mask,
        detect_triggers: bool,
    ) -> Option<(Ident, Real, PointProjection)> {
        // Storage for the result
        let mut result: Option<(Ident, Real, PointProjection)> = None;

        // Repeatable point projection process for both static and dynamic objects.
        let check_object = |obj: Ref<'_, dyn Object>| {
            // Check if the object pass the filter.
            let solid = !obj.is_trigger_area();
            if (detect_triggers || solid) && obj.pass_filter(filter) {
                // Perform a raycast against the object.
                Some(obj.shape().project_point(obj.isometry(), point, solid))
            } else {
                None
            }
        };

        // Project the point against static objects.
        if let Some((idx, (dist, proj))) =
            self.statics
                .project_point(point, max_distance, |idx, _best_so_far| {
                    if let Some(obj) = self.statics.get(idx) {
                        check_object(obj)
                    } else {
                        None
                    }
                })
        {
            max_distance = dist;
            result = Some((Ident::new(idx, false), dist, proj));
        }

        // Project the point against dynamic objects.
        if let Some((idx, (dist, proj))) =
            self.dynamics
                .project_point(point, max_distance, |idx, _best_so_far| {
                    if let Some(obj) = self.dynamics.get(idx) {
                        check_object(obj)
                    } else {
                        None
                    }
                })
        {
            Some((Ident::new(idx, true), dist, proj))
        } else {
            result
        }
    }
}
