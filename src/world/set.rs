// Simple data structure for storing colliders.

use crate::{
    collider::{
        bounding_box::{BoundingBox, HasBoundingBox},
        handle::HasHandle,
    },
    pointer::{Mut, RefCounted},
};
use bvh_arena::Bvh;
use parry::math::Real;

/// Simple data structure for storing colliders.
#[derive(Default)]
pub struct Set<T>(Bvh<RefCounted<Mut<T>>, BoundingBox>);

impl<T> Set<T>
where
    T: HasBoundingBox + HasHandle,
{
    /// Inserts a new collider into the set.
    pub fn insert(&mut self, collider: RefCounted<Mut<T>>, delta: Real) {
        // extract the collider from the pointer
        let mut inner = get_inner!(mut collider);

        // insert the collider into the set
        let volume = inner.compute_bounding_box(delta);
        let handle = self.0.insert(collider.clone(), volume);

        // store the handle in the collider
        inner.assign_handle(handle);
    }
}

impl<T> Set<T>
where
    T: HasHandle,
{
    /// Removes a collider from the set.
    pub fn remove(&mut self, collider: &T) {
        if let Some(handle) = collider.get_handle() {
            self.0.remove(handle);
        }
    }
}

impl<T> Set<T>
where
    T: HasBoundingBox,
{
    /// Finds all colliders that overlap with the given collider.
    pub fn find_overlaps_with(&self, collider: &T) {
        if let Some(bbox) = collider.get_bounding_box() {
            self.0.for_each_overlaps(bbox, |other| {
                let other = get_inner!(other);

                // TODO: Perform narrow collision detection
            });
        }
    }
}
