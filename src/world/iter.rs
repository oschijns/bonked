//! Iterators for events raised during a physics update

use crate::world::{OnContact, World, set::Id};
use core::slice::Iter;
use parry::query::ShapeCastHit;

impl World {
    /// Return an iterator over trigger events related to the given ID
    #[inline]
    pub fn trigger_events(&self, id: Id) -> TriggerIter<'_> {
        TriggerIter {
            target_id: id,
            iter: self.on_trigger.iter(),
        }
    }

    /// Return an iterator over collision events related to the given ID
    #[inline]
    pub fn collision_events(&self, id: Id) -> CollisionIter<'_> {
        CollisionIter {
            target_id: id,
            iter: self.on_collision.iter(),
        }
    }
}

/// Iterator over trigger events
pub struct TriggerIter<'i> {
    /// ID to look for
    target_id: Id,

    /// Internal iterator
    iter: Iter<'i, OnContact>,
}

/// Trigger event
pub struct TriggerEvent {
    /// Id of the other object encountered
    pub other_id: Id,

    /// Is the other object dynamic
    pub other_dynamic: bool,
}

/// Iterator over collision events
pub struct CollisionIter<'i> {
    /// ID to look for
    target_id: Id,

    /// Internal iterator
    iter: Iter<'i, OnContact<ShapeCastHit>>,
}

/// Collision event
pub struct CollisionEvent {
    /// Id of the other object encountered
    pub other_id: Id,

    /// Is the other object dynamic
    pub other_dynamic: bool,

    /// Shape cast hit data of requested ID compared to other object
    pub hit: ShapeCastHit,
}

/// Implement iterator trait for trigger event iterator
impl Iterator for TriggerIter<'_> {
    type Item = TriggerEvent;

    fn next(&mut self) -> Option<Self::Item> {
        let id = self.target_id;

        // Keep looping until we find a relevant event
        while let Some(e) = self.iter.next() {
            if id == e.id1 {
                return Some(TriggerEvent {
                    other_id: e.id2,
                    other_dynamic: e.dynamic2,
                });
            } else if id == e.id2 {
                return Some(TriggerEvent {
                    other_id: e.id1,
                    other_dynamic: true,
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
        let id = self.target_id;

        // Keep looping until we find a relevant event
        while let Some(e) = self.iter.next() {
            if id == e.id1 {
                return Some(CollisionEvent {
                    other_id: e.id2,
                    other_dynamic: e.dynamic2,
                    hit: e.data,
                });
            } else if id == e.id2 {
                return Some(CollisionEvent {
                    other_id: e.id1,
                    other_dynamic: true,
                    hit: e.data.swapped(),
                });
            }
        }
        None
    }
}
