//! Simple physics engine for the game

/// Kinematic body
pub mod kinematic_body;

/// Static body
pub mod static_body;

/// Trigger area
pub mod trigger_area;

/// Hit result between solid objects
pub mod contact;

use super::Mask;
use crate::{
    object::{kinematic_body::KinematicBody, static_body::StaticBody},
    world::aabb::Aabb,
};
use alloc::sync::Arc;
use bvh_arena::VolumeHandle;
use parry::{
    math::{Isometry, Real, Vector},
    query::{self, Contact, ShapeCastHit, ShapeCastOptions},
    shape::Shape,
};

/// Mask where all bits are set to 1
const MASK_ALL: Mask = Mask::MAX;

/// Trait implemented for static and dynamic bodies
pub trait Object {
    type Payload;

    /// Store the handle of this object after it has been added to the world
    fn set_handle(&mut self, handle: VolumeHandle);

    /// Unset the handle of this object
    fn unset_handle(&mut self);

    /// Access the handle of this object
    fn handle(&self) -> Option<VolumeHandle>;

    /// Access the shape assigned to this body
    fn shape(&self) -> &dyn Shape;

    /// Access the isometry of this shape
    fn isometry(&self) -> &Isometry<Real>;

    /// Create an Axis-Aligned Bounding Box for this body
    fn aabb(&self) -> Aabb;

    /// Access the payload defined on this object
    fn payload(&self) -> &Self::Payload;

    /// Access the payload defined on this object
    fn payload_mut(&mut self) -> &mut Self::Payload;

    /// Get the layer(s) this body belongs to
    #[inline]
    fn layer(&self) -> Mask {
        MASK_ALL
    }

    /// Get the layers this body can interact with
    #[inline]
    fn mask(&self) -> Mask {
        MASK_ALL
    }

    /// Get the velocity of the body (if it has one)
    #[inline]
    fn velocity(&self) -> Vector<Real> {
        Vector::default()
    }

    /// Try to cast the object into a kinematic body
    #[inline]
    fn as_kinematic(&self) -> Option<&KinematicBody<Self::Payload>> {
        None
    }

    /// Try to cast the object into a static body
    #[inline]
    fn as_static(&self) -> Option<&StaticBody<Self::Payload>> {
        None
    }
}

/// Common data shared between static and dynamic bodies
struct CommonData<P> {
    /// Handle of this body in the world
    handle: Option<VolumeHandle>,

    /// Collision shape used by this zone
    shape: Arc<dyn Shape>,

    /// Isometry of this body
    isometry: Isometry<Real>,

    /// Arbitrary payload
    payload: P,
}

impl<P> CommonData<P> {
    /// Create a new common data instance
    #[inline]
    pub fn new(shape: Arc<dyn Shape>, isometry: Isometry<Real>, payload: P) -> Self {
        CommonData {
            handle: None,
            shape,
            isometry,
            payload,
        }
    }
}

impl<P> Object for CommonData<P> {
    type Payload = P;

    /// Store the handle of this object after it has been added to the world
    #[inline]
    fn set_handle(&mut self, handle: VolumeHandle) {
        self.handle = Some(handle);
    }

    /// Unset the handle of this object
    #[inline]
    fn unset_handle(&mut self) {
        self.handle = None;
    }

    /// Access the handle of this object
    #[inline]
    fn handle(&self) -> Option<VolumeHandle> {
        self.handle
    }

    /// Access the shape assigned to this body
    #[inline]
    fn shape(&self) -> &dyn Shape {
        self.shape.as_ref()
    }

    /// Access the isometry of this shape
    #[inline]
    fn isometry(&self) -> &Isometry<f32> {
        &self.isometry
    }

    /// Build a generic AABB for this body
    #[inline]
    fn aabb(&self) -> Aabb {
        Aabb::new(self.shape.compute_aabb(&self.isometry), MASK_ALL, MASK_ALL)
    }

    /// Access the payload defined on this object
    #[inline]
    fn payload(&self) -> &Self::Payload {
        &self.payload
    }

    /// Access the payload defined on this object
    #[inline]
    fn payload_mut(&mut self) -> &mut Self::Payload {
        &mut self.payload
    }
}

/// Check if two objects intersects
#[inline]
pub fn intersects<A, B>(a: &A, b: &B) -> bool
where
    A: Object,
    B: Object,
{
    query::intersection_test(a.isometry(), a.shape(), b.isometry(), b.shape()).unwrap_or(false)
}

/// Check if two objects are in contact
#[inline]
pub fn contacts<A, B>(a: &A, b: &B, prediction: Real) -> Option<Contact>
where
    A: Object,
    B: Object,
{
    query::contact(a.isometry(), a.shape(), b.isometry(), b.shape(), prediction).unwrap_or(None)
}

/// Check if two objects will collide
#[inline]
pub fn collides<A, B>(a: &A, b: &B, options: ShapeCastOptions) -> Option<ShapeCastHit>
where
    A: Object,
    B: Object,
{
    query::cast_shapes(
        a.isometry(),
        &a.velocity(),
        a.shape(),
        b.isometry(),
        &b.velocity(),
        b.shape(),
        options,
    )
    .unwrap_or(None)
}
