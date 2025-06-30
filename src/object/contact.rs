//! Contact between two solid objects

use crate::object::payload::WeakPayload;
use core::cmp::Ordering;
use parry::{math::Real, query::ShapeCastHit};

/// Contact data
pub struct Contact {
    /// parry's shape cast hit
    hit: ShapeCastHit,

    /// Weight ratio between the two objects in contact
    weight_ratio: Real,

    /// Payload of the other object
    payload: WeakPayload,
}

impl Contact {
    /// Create a new contact result
    #[inline]
    pub fn new(hit: ShapeCastHit, weight_ratio: Real, payload: WeakPayload) -> Self {
        Self {
            hit,
            weight_ratio,
            payload,
        }
    }

    /// Access parry's hit data
    #[inline]
    pub fn hit(&self) -> &ShapeCastHit {
        &self.hit
    }

    /// Get the weight ratio of this contact
    #[inline]
    pub fn weight_ratio(&self) -> Real {
        self.weight_ratio
    }

    /// Get the payload data of the other object
    #[inline]
    pub fn payload(&self) -> WeakPayload {
        self.payload.clone()
    }

    /// Compare two contact results to order them from nearest to furtherest
    pub fn order(&self, other: &Self, epsilon: Real) -> Ordering {
        let ta = self.hit.time_of_impact;
        let tb = other.hit.time_of_impact;
        if (ta - tb).abs() < epsilon {
            Ordering::Equal
        } else if ta < tb {
            Ordering::Less
        } else {
            Ordering::Greater
        }
    }
}
