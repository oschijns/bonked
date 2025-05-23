//! Storage for physics bodies of a given type.
//! Guarantee that the reference to the bodies are
//! maintained as long as they are part of the physics world.

use crate::object::{handle::Handle, Object};
use alloc::{sync::Arc, vec::Vec};

/// Store a set of elements
#[derive(Default)]
pub struct Set<O>(pub(crate) Vec<Arc<O>>);

impl<O> Set<O>
where
    O: Object + 'static,
    Handle: From<Arc<O>>,
{
    /// Add a new element to this set
    pub fn add(&mut self, object: Arc<O>) -> Handle {
        self.0.push(object.clone());
        Handle::from(object)
    }

    /// Remove an element from this set
    pub fn remove(&mut self, object: Arc<O>) -> Option<Handle> {
        // find the position of the object in the list
        for (index, value) in self.0.iter().enumerate() {
            if Arc::ptr_eq(&object, value) {
                // We found the index, create an handle and remove the object.
                self.0.swap_remove(index);
                return Some(Handle::from(object));
            }
        }

        // The object was not found
        None
    }
}
