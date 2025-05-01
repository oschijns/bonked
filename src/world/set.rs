// Simple data structure for storing colliders.

use crate::{
    collider::{
        traits::{BoundingBox, HasHandle},
        volume::Volume,
    },
    pointer::{Mut, RefCounted},
};
use bvh_arena::Bvh;
use parry::math::Real;

/// Simple data structure for storing colliders.
#[derive(Default)]
pub struct Set<T>(Bvh<RefCounted<Mut<T>>, Volume>);

impl<T> Set<T>
where
    T: BoundingBox + HasHandle,
{
    /// Inserts a new collider into the set.
    pub fn insert(&mut self, collider: RefCounted<Mut<T>>, delta: Real) {
        let volume = collider.borrow().get_bounding_box(delta);
        let handle = self.0.insert(collider.clone(), volume);
        collider.borrow_mut().assign_handle(handle);
    }

    /// Removes a collider from the set.
    pub fn remove(&mut self, collider: RefCounted<Mut<T>>) {
        if let Some(handle) = collider.borrow().get_handle() {
            self.0.remove(handle);
        }
    }
}
