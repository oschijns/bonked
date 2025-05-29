//! Storage for physics bodies of a given type.
//! Guarantee that the reference to the bodies are
//! maintained as long as they are part of the physics world.

use super::{aabb::Aabb, Shared};
use crate::object::Object;
use alloc::{sync::Arc, vec::Vec};
use bvh_arena::Bvh;
use delegate::delegate;
use spin::RwLock;

/// Store a set of elements
pub struct Set<O> {
    /// List of objects in the set
    pub(crate) objects: Vec<Shared<O>>,

    /// Partitionning of the objects in the set
    pub(crate) partition: Bvh<Shared<O>, Aabb>,
}

/// Create a new empty set
impl<O> Default for Set<O> {
    #[inline]
    fn default() -> Self {
        Self {
            objects: Vec::default(),
            partition: Bvh::default(),
        }
    }
}

impl<O> Set<O> {
    /// Create a new empty set with a predefined capacity
    #[inline]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            objects: Vec::with_capacity(capacity),
            partition: Bvh::default(),
        }
    }

    // Expose some methods from the underlying vector
    delegate! {
        to self.objects {
            pub fn len(&self) -> usize;
            pub fn is_empty(&self) -> bool;
            pub fn reserve(&mut self, additional: usize);
            pub fn reserve_exact(&mut self, additional: usize);
            pub fn shrink_to_fit(&mut self);
            pub fn shrink_to(&mut self, min_capacity: usize);
            pub fn iter(&self) -> impl Iterator<Item = &Arc<RwLock<O>>>;
            pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut Arc<RwLock<O>>>;
        }
    }

    /// Store a new element in this set
    pub fn store(&mut self, object: Shared<O>) {
        self.objects.push(object);
    }

    /// Remove an element from this set but don't look into the partition
    /// Prefer using `remove` instead.
    pub fn quick_remove(&mut self, object: &Shared<O>) -> bool {
        // find the position of the object in the list
        for (index, value) in self.objects.iter().enumerate() {
            if Arc::ptr_eq(&object, value) {
                // We found the index, create an handle and remove the object.
                self.objects.swap_remove(index);

                // once found, stop the iteration
                return true;
            }
        }
        false
    }

    /// Reset the partition but don't update the objects' handles
    /// Prefer using `repartition` instead.
    #[inline]
    pub fn quick_reset(&mut self) {
        self.partition.clear();
    }
}

impl<O> Set<O>
where
    O: Object,
{
    /// Store the element and add it to the partition too
    pub fn add(&mut self, object: Shared<O>) {
        // add the object to the list
        self.objects.push(object.clone());

        // lock the object with write access to add it to the partition
        let mut mut_obj = object.write();
        let handle = self.partition.insert(object.clone(), mut_obj.aabb());
        mut_obj.set_handle(handle);
    }

    /// Remove an element from this set
    pub fn clean_remove(&mut self, object: &Shared<O>) -> bool {
        // find the position of the object in the list
        for (index, value) in self.objects.iter().enumerate() {
            if Arc::ptr_eq(&object, value) {
                // We found the index, create an handle and remove the object.
                self.objects.swap_remove(index);

                // detach the handle from the object
                let mut mut_obj = object.write();
                let handle = mut_obj.handle();
                mut_obj.unset_handle();

                // use the handle to remove the object from the partition
                if let Some(handle) = handle {
                    self.partition.remove(handle);
                }

                // once found, stop the iteration
                return true;
            }
        }
        false
    }

    /// Compute a partitionning for the objects defined in this set
    pub fn repartition(&mut self) {
        self.partition.clear();
        for object in &self.objects {
            let mut mut_obj = object.write();
            let handle = self.partition.insert(object.clone(), mut_obj.aabb());
            mut_obj.set_handle(handle);
        }
    }
}
