//! Storage for physics bodies of a given type.
//! Guarantee that the reference to the bodies are
//! maintained as long as they are part of the physics world.

use crate::object::Object;
use core::cell::{Ref, RefCell, RefMut};
use delegate::delegate;
use hashbrown::{HashMap, hash_map::Iter};
use parry::{
    bounding_volume::Aabb,
    math::{Real, Vector},
    partitioning::{Bvh, BvhWorkspace},
    query::{PointProjection, Ray},
};

/// Identifier to find an object in the set
pub type Index = u32;

/// Set of physics objects
pub struct Set<O> {
    /// Next index to use for the next object
    pub(crate) next_index: Index,

    /// List of objects in this set
    pub(crate) objects: HashMap<Index, RefCell<O>>,

    /// Partitionning of the objects in the world
    pub(crate) bvh: Bvh,
}

impl<O> Default for Set<O> {
    fn default() -> Self {
        Self {
            next_index: 0,
            objects: HashMap::default(),
            bvh: Bvh::default(),
        }
    }
}

impl<O> Set<O> {
    /// Create a new empty set
    #[inline]
    pub fn new() -> Self {
        Self {
            next_index: 0,
            objects: HashMap::new(),
            bvh: Bvh::new(),
        }
    }

    /// Create a new empty set with a predefined capacity
    #[inline]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            next_index: 0,
            objects: HashMap::with_capacity(capacity),
            bvh: Bvh::new(),
        }
    }
}

// Re-export some Bvh methods
impl<O> Set<O> {
    #[inline]
    pub fn intersect_aabb<'a>(&'a self, aabb: &'a Aabb) -> impl Iterator<Item = u32> + 'a {
        self.bvh.intersect_aabb(aabb)
    }

    #[inline]
    pub fn project_point(
        &self,
        point: Vector,
        max_distance: Real,
        primitive_check: impl Fn(Index, Real) -> Option<PointProjection>,
    ) -> Option<(Index, (Real, PointProjection))> {
        self.bvh.project_point(point, max_distance, primitive_check)
    }

    #[inline]
    pub fn cast_ray(
        &self,
        ray: &Ray,
        max_time_of_impact: Real,
        primitive_check: impl Fn(Index, Real) -> Option<Real>,
    ) -> Option<(Index, Real)> {
        self.bvh.cast_ray(ray, max_time_of_impact, primitive_check)
    }
}

impl<O> Set<O> {
    delegate! {
        to self.bvh {
            #[inline] pub fn refit(&mut self, workspace: &mut BvhWorkspace);
            #[inline] pub fn refit_without_opt(&mut self);
            #[inline] pub fn root_aabb(&self) -> Aabb;
            #[inline] pub fn subtree_depth(&self, node_index: Index) -> u32;
            #[inline] pub fn leaf_count(&self) -> u32;
            #[inline] pub fn assert_well_formed(&self);
            #[inline] pub fn assert_well_formed_topology_only(&self);
            #[inline] pub fn assert_is_depth_first(&self);
        }
    }
    delegate! {
        to self.objects {
            #[inline] pub fn iter(&self) -> Iter<'_, Index, RefCell<O>>;
        }
    }
}

impl<O> Set<O>
where
    O: Object,
{
    /// Get the object for the given index
    pub fn get(&self, index: Index) -> Option<Ref<'_, O>> {
        self.objects.get(&index).map(|o| o.borrow())
    }

    /// Get the object for the given index
    pub fn get_mut(&mut self, index: Index) -> Option<RefMut<'_, O>> {
        self.objects.get(&index).map(|o| o.borrow_mut())
    }

    /// Add the object in the set, but this requires to call the refit method afterwards
    pub fn quick_add(&mut self, object: RefCell<O>, margin: Real) -> Index {
        let aabb = object.borrow().aabb();
        unsafe {
            self.objects
                .insert_unique_unchecked(self.next_index, object);
        }
        self.bvh
            .insert_or_update_partially(aabb, self.next_index, margin);
        self.next_index += 1;
        self.next_index
    }

    /// Add the object in the set
    pub fn clean_add(&mut self, object: RefCell<O>, margin: Real) -> Index {
        let aabb = object.borrow().aabb();
        unsafe {
            self.objects
                .insert_unique_unchecked(self.next_index, object);
        }
        self.bvh
            .insert_with_change_detection(aabb, self.next_index, margin);
        self.next_index += 1;
        self.next_index
    }

    /// Remove an object from this set
    pub fn remove(&mut self, index: Index) -> bool {
        if self.objects.remove(&index).is_some() {
            self.bvh.remove(index);
            true
        } else {
            false
        }
    }

    /// Compute a partitionning for the objects defined in this set
    pub fn repartition(&mut self) {}
}
