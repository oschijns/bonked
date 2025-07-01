//! Axis-Aligned Bounding Box (AABB)

use crate::Mask;
use parry::{
    bounding_volume as p,
    math::{Point, Real},
    query::Ray,
};

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
    /// Create a new AABB with the given parameters
    #[inline]
    pub fn new(aabb: p::Aabb, layer: Mask, mask: Mask) -> Self {
        Self { aabb, layer, mask }
    }

    /// Create a new AABB from a ray
    pub fn from_ray(ray: &Ray, max_time_of_impact: Real, layer: Mask, mask: Mask) -> Self {
        let (mins, maxs) = ray.origin.coords.inf_sup(&(ray.dir * max_time_of_impact));
        let aabb = p::Aabb::new(Point::from(mins), Point::from(maxs));
        Self::new(aabb, layer, mask)
    }

    /// Create a new AABB from a point
    pub fn from_point(point: &Point<Real>, layer: Mask, mask: Mask) -> Self {
        let aabb = p::Aabb::new(*point, *point);
        Self::new(aabb, layer, mask)
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
