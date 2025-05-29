//! Storage for physics bodies of a given type.
//! Guarantee that the reference to the bodies are
//! maintained as long as they are part of the physics world.

use crate::object::{handle::Handle, Object};
use alloc::{sync::Arc, vec::Vec};
use delegate::delegate;

/// Store a set of elements
pub struct Set<O>(pub(crate) Vec<Arc<O>>);

/// Create a new empty set
impl<O> Default for Set<O> {
    #[inline]
    fn default() -> Self {
        Self(Vec::default())
    }
}

impl<O> Set<O> {
    /// Create a new empty set with a predefined capacity
    #[inline]
    pub fn with_capacity(capacity: usize) -> Self {
        Self(Vec::with_capacity(capacity))
    }

    // Expose some methods from the underlying vector
    delegate! {
        to self.0 {
            pub fn len(&self) -> usize;
            pub fn is_empty(&self) -> bool;
            pub fn reserve(&mut self, additional: usize);
            pub fn reserve_exact(&mut self, additional: usize);
            pub fn shrink_to_fit(&mut self);
            pub fn shrink_to(&mut self, min_capacity: usize);
            pub fn iter(&self) -> impl Iterator<Item = &Arc<O>>;
            pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut Arc<O>>;
        }
    }
}

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
