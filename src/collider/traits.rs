//! Define a set of traits for handling collisions between heterogenous objects.

use super::{volume::Volume, Collider, DynamicCollider};
use bvh_arena::VolumeHandle;
use parry::math::{Real, Translation};

/// Get the bounding box of an object.
pub(crate) trait BoundingBox {
    /// Get the axis-aligned bounding box of this object.
    fn get_bounding_box(&self, delta: Real) -> Volume;
}

/// Assign the handle to the collider.
pub(crate) trait HasHandle {
    /// Get the handle of the collider.
    fn get_handle(&self) -> Option<VolumeHandle>;

    /// Assign the handle to the collider.
    fn assign_handle(&mut self, handle: VolumeHandle);

    /// Unset the handle of the collider.
    fn unset_handle(&mut self);
}

impl BoundingBox for Collider {
    fn get_bounding_box(&self, _delta: Real) -> Volume {
        Volume {
            layer: self.layer,
            mask: self.mask,
            aabb: self.shape.compute_aabb(&self.isometry),
        }
    }
}

impl BoundingBox for DynamicCollider {
    fn get_bounding_box(&self, delta: Real) -> Volume {
        // Compute the location of the end point
        let translation = Translation::from(self.velocity * delta);
        let mut end_iso = self.base.isometry;
        end_iso.append_translation_mut(&translation);

        // Compute the swept AABB
        Volume {
            layer: self.base.layer,
            mask: self.base.mask,
            aabb: self
                .base
                .shape
                .compute_swept_aabb(&self.base.isometry, &end_iso),
        }
    }
}

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
