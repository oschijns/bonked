//! Axis-Aligned Bounding Box (AABB)

use crate::Mask;
use parry::bounding_volume as p;

/// Axis-Aligned Bounding Box (AABB)
#[derive(Debug, Clone, Copy)]
pub struct Aabb {
    /// Parry's Axis-Aligned Bounding Box
    aabb: p::Aabb,

    /// Collision layer where this AABB belongs
    layer: Mask,

    /// Collision mask for this AABB
    mask: Mask,
}

impl Aabb {
    /// Create a new Aabb with the given parameters
    #[inline]
    pub fn new(aabb: p::Aabb, layer: Mask, mask: Mask) -> Self {
        Self { aabb, layer, mask }
    }

    /// Access the Parry's Axis-Aligned Bounding Box
    #[inline]
    pub fn aabb(&self) -> &p::Aabb {
        &self.aabb
    }

    /// Access the collision layer where this AABB belongs
    #[inline]
    pub fn layer(&self) -> Mask {
        self.layer
    }

    /// Access the collision mask for this AABB
    #[inline]
    pub fn mask(&self) -> Mask {
        self.mask
    }
}

impl Default for Aabb {
    fn default() -> Self {
        Self {
            aabb: p::Aabb::new_invalid(),
            layer: 0,
            mask: 0,
        }
    }
}

/// implement BVH Arena bounding volume trait to our custom AABB
impl bvh_arena::BoundingVolume for Aabb {
    fn merge(self, other: Self) -> Self {
        Self {
            aabb: p::BoundingVolume::merged(&self.aabb, &other.aabb),
            layer: self.layer | other.layer,
            mask: self.mask | other.mask,
        }
    }

    #[inline]
    fn area(&self) -> f32 {
        self.aabb.volume()
    }

    fn overlaps(&self, other: &Self) -> bool {
        if self.layer & other.mask != 0 && self.mask & other.layer != 0 {
            p::BoundingVolume::intersects(&self.aabb, &other.aabb)
        } else {
            false
        }
    }
}
