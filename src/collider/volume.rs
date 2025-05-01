//! Axis-Aligned Bounding Box mapping between parry's AABB and bvh_arena's BoundingVolume

use crate::Mask;
use parry::bounding_volume::{Aabb, BoundingVolume};

/// Wrapper around parry's AABB type to implement the BoundingVolume trait from bvh_arena
#[derive(Clone, Copy)]
pub struct Volume {
    /// collision layer of the object
    pub layer: Mask,

    /// collision mask of the object
    pub mask: Mask,

    /// parry's AABB type
    pub aabb: Aabb,
}

impl bvh_arena::BoundingVolume for Volume {
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

    /// Get the area/volume covered by this AABB
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
