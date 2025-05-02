//! Axis-Aligned Bounding Box mapping between parry's AABB and bvh_arena's BoundingVolume

use super::{Collider, DynamicCollider};
use crate::Mask;
use parry::{
    bounding_volume::{Aabb, BoundingVolume},
    math::{Real, Translation},
};

/// Wrapper around parry's AABB type to implement the BoundingVolume trait from bvh_arena
#[derive(Clone, Copy)]
pub struct BoundingBox {
    /// collision layer of the object
    pub layer: Mask,

    /// collision mask of the object
    pub mask: Mask,

    /// parry's AABB type
    pub aabb: Aabb,
}

/// Get the bounding box of an object.
pub(crate) trait HasBoundingBox {
    /// Compute the axis-aligned bounding box of this object.
    fn compute_bounding_box(&mut self, delta: Real) -> BoundingBox;

    /// Access the resolved axis-aligned bounding box of this object.
    fn get_bounding_box(&self) -> &Option<BoundingBox>;
}

/// Implement BVH BoundingVolume trait for our BoundingBox type
impl bvh_arena::BoundingVolume for BoundingBox {
    /// Merge two AABB into one
    fn merge(self, other: Self) -> Self {
        let mut new = self.aabb;
        new.merge(&other.aabb);
        Self {
            layer: self.layer | other.layer,
            mask: self.mask | other.mask,
            aabb: new,
        }
    }

    /// Get the area/BoundingBox covered by this AABB
    #[inline]
    fn area(&self) -> f32 {
        self.aabb.volume()
    }

    /// Return true if two AABB overlaps
    fn overlaps(&self, other: &Self) -> bool {
        // compare the layer of one with the mask of the other
        if self.layer & other.mask != 0 && self.mask & other.layer != 0 {
            // check if the AABBs overlap
            self.aabb.intersects(&other.aabb)
        } else {
            false
        }
    }
}

/// Allow collider to build a bounding box
impl HasBoundingBox for Collider {
    /// Compute the axis-aligned bounding box of this object without storing it.
    fn compute_bounding_box(&mut self, _delta: Real) -> BoundingBox {
        BoundingBox {
            layer: self.layer,
            mask: self.mask,
            aabb: self.shape.compute_aabb(&self.isometry),
        }
    }

    /// Return none.
    #[inline]
    fn get_bounding_box(&self) -> &Option<BoundingBox> {
        const NONE: Option<BoundingBox> = None;
        &NONE
    }
}

/// Allow dynamic collider to build a bounding box
impl HasBoundingBox for DynamicCollider {
    /// Compute the axis-aligned bounding box of this object and store it.
    fn compute_bounding_box(&mut self, delta: Real) -> BoundingBox {
        // Compute the location of the end point
        let translation = Translation::from(self.velocity * delta);
        let mut end_iso = self.base.isometry;
        end_iso.append_translation_mut(&translation);

        // Compute the swept AABB
        let bbox = BoundingBox {
            layer: self.base.layer,
            mask: self.base.mask,
            aabb: self
                .base
                .shape
                .compute_swept_aabb(&self.base.isometry, &end_iso),
        };
        self.resolved_sweep = Some(bbox);
        bbox
    }

    /// Access the resolved axis-aligned bounding box of this object.
    #[inline]
    fn get_bounding_box(&self) -> &Option<BoundingBox> {
        &self.resolved_sweep
    }
}
