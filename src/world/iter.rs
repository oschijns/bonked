//! Iterators for events raised during a physics update

use crate::world::{Ident, OnContact, World, set::Index};
use core::slice::Iter;
use parry::query::ShapeCastHit;

impl World {
    /// Return an iterator over trigger events related to the given index
    #[inline]
    pub fn trigger_events(&self, index: Index) -> TriggerIter<'_> {
        TriggerIter {
            target_index: index,
            iter: self.on_trigger.iter(),
        }
    }

    /// Return an iterator over collision events related to the given index
    #[inline]
    pub fn collision_events(&self, index: Index) -> CollisionIter<'_> {
        CollisionIter {
            target_index: index,
            iter: self.on_collision.iter(),
        }
    }
}

/// Iterator over trigger events
pub struct TriggerIter<'i> {
    /// Index to look for
    target_index: Index,

    /// Internal iterator
    iter: Iter<'i, OnContact>,
}

/// Trigger event
pub struct TriggerEvent {
    /// Identifier of the other object encountered
    pub other_ident: Ident,
}

/// Iterator over collision events
pub struct CollisionIter<'i> {
    /// Index to look for
    target_index: Index,

    /// Internal iterator
    iter: Iter<'i, OnContact<ShapeCastHit>>,
}

/// Collision event
pub struct CollisionEvent {
    /// Identifier of the other object encountered
    pub other_ident: Ident,

    /// Shape cast hit data of requested Index compared to other object
    pub hit: ShapeCastHit,
}

/// Implement iterator trait for trigger event iterator
impl Iterator for TriggerIter<'_> {
    type Item = TriggerEvent;

    fn next(&mut self) -> Option<Self::Item> {
        let index = self.target_index;

        // Keep looping until we find a relevant event
        while let Some(e) = self.iter.next() {
            if index == e.index1 {
                return Some(TriggerEvent {
                    other_ident: e.ident2,
                });
            } else if index == e.ident2.index {
                return Some(TriggerEvent {
                    other_ident: Ident::new(e.index1, true),
                });
            }
        }
        None
    }
}

/// Implement iterator trait for collision event iterator
impl Iterator for CollisionIter<'_> {
    type Item = CollisionEvent;

    fn next(&mut self) -> Option<Self::Item> {
        let index = self.target_index;

        // Keep looping until we find a relevant event
        while let Some(e) = self.iter.next() {
            if index == e.index1 {
                return Some(CollisionEvent {
                    other_ident: e.ident2,
                    hit: e.data,
                });
            } else if index == e.ident2.index {
                return Some(CollisionEvent {
                    other_ident: Ident::new(e.index1, true),
                    hit: e.data.swapped(),
                });
            }
        }
        None
    }
}
