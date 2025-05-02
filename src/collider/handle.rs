//! Define a set of traits for handling collisions between heterogenous objects.

use super::{Collider, DynamicCollider};
use bvh_arena::VolumeHandle;

/// Trait for managing the handle of a collider.
pub(crate) trait HasHandle {
    /// Get the handle of the collider.
    fn get_handle(&self) -> Option<VolumeHandle>;

    /// Assign the handle to the collider.
    fn assign_handle(&mut self, handle: VolumeHandle);

    /// Unset the handle of the collider.
    fn unset_handle(&mut self);
}

/// Colliders have an associated handle.
impl HasHandle for Collider {
    #[inline]
    fn get_handle(&self) -> Option<VolumeHandle> {
        self.handle
    }

    #[inline]
    fn assign_handle(&mut self, handle: VolumeHandle) {
        self.handle = Some(handle);
    }

    #[inline]
    fn unset_handle(&mut self) {
        self.handle = None;
    }
}

/// Dynamic colliders have an associated handle.
impl HasHandle for DynamicCollider {
    #[inline]
    fn get_handle(&self) -> Option<VolumeHandle> {
        self.base.handle
    }

    #[inline]
    fn assign_handle(&mut self, handle: VolumeHandle) {
        self.base.handle = Some(handle);
    }

    #[inline]
    fn unset_handle(&mut self) {
        self.base.handle = None;
    }
}
