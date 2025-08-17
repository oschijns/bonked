//! Storage for physics bodies of a given type.
//! Guarantee that the reference to the bodies are
//! maintained as long as they are part of the physics world.

use crate::object::Object;
use core::cell::{Ref, RefCell, RefMut};
use delegate::delegate;
use parry::{
    bounding_volume::Aabb,
    math::{Point, Real},
    partitioning::{Bvh, BvhWorkspace},
    query::{PointProjection, Ray},
    utils::hashmap::HashMap,
};

/// Identifier to find an object in the set
pub type Id = u32;

/// Set of physics objects
#[derive(Default)]
pub struct Set<O> {
    /// Next ID to use for the next object
    pub(crate) next_id: Id,

    /// List of objects in this set
    pub(crate) objects: HashMap<Id, RefCell<O>>,

    /// Partitionning of the objects in the world
    pub(crate) bvh: Bvh,
}

impl<O> Set<O> {
    /// Create a new empty set
    #[inline]
    pub fn new() -> Self {
        Self {
            next_id: 0,
            objects: HashMap::new(),
            bvh: Bvh::new(),
        }
    }

    /// Create a new empty set with a predefined capacity
    #[inline]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            next_id: 0,
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
        point: &Point<Real>,
        max_distance: Real,
        primitive_check: impl Fn(Id, Real) -> Option<PointProjection>,
    ) -> Option<(Id, (Real, PointProjection))> {
        self.bvh.project_point(point, max_distance, primitive_check)
    }

    #[inline]
    pub fn cast_ray(
        &self,
        ray: &Ray,
        max_time_of_impact: Real,
        primitive_check: impl Fn(Id, Real) -> Option<Real>,
    ) -> Option<(Id, Real)> {
        self.bvh.cast_ray(ray, max_time_of_impact, primitive_check)
    }
}

impl<O> Set<O> {
    delegate! {
        to self.bvh {
            #[inline] pub fn refit(&mut self, workspace: &mut BvhWorkspace);
            #[inline] pub fn refit_without_opt(&mut self);
            #[inline] pub fn root_aabb(&self) -> Aabb;
            #[inline] pub fn subtree_depth(&self, node_id: Id) -> u32;
            #[inline] pub fn leaf_count(&self) -> u32;
            #[inline] pub fn assert_well_formed(&self);
            #[inline] pub fn assert_well_formed_topology_only(&self);
            #[inline] pub fn assert_is_depth_first(&self);
        }
    }
}

impl<O> Set<O>
where
    O: Object,
{
    /// Get the object for the given ID
    pub fn get(&self, id: Id) -> Option<Ref<'_, O>> {
        if let Some(o) = self.objects.get(&id) {
            Some(o.borrow())
        } else {
            None
        }
    }

    /// Get the object for the given ID
    pub fn get_mut(&mut self, id: Id) -> Option<RefMut<'_, O>> {
        if let Some(o) = self.objects.get(&id) {
            Some(o.borrow_mut())
        } else {
            None
        }
    }

    /// Add the object in the set, but this requires to call the refit method afterwards
    pub fn quick_add(&mut self, object: RefCell<O>, margin: Real) -> Id {
        let aabb = object.borrow().aabb();
        unsafe {
            self.objects.insert_unique_unchecked(self.next_id, object);
        }
        self.bvh
            .insert_or_update_partially(aabb, self.next_id, margin);
        self.next_id += 1;
        self.next_id
    }

    /// Add the object in the set
    pub fn clean_add(&mut self, object: RefCell<O>, margin: Real) -> Id {
        let aabb = object.borrow().aabb();
        unsafe {
            self.objects.insert_unique_unchecked(self.next_id, object);
        }
        self.bvh
            .insert_with_change_detection(aabb, self.next_id, margin);
        self.next_id += 1;
        self.next_id
    }

    /// Remove an object from this set
    pub fn remove(&mut self, id: Id) -> bool {
        if self.objects.remove(&id).is_some() {
            self.bvh.remove(id);
            true
        } else {
            false
        }
    }

    /// Compute a partitionning for the objects defined in this set
    pub fn repartition(&mut self) {}
}
