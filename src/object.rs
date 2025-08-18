//! Simple physics engine for the game

/// Common data to all objects
pub mod common_object;

/// Static objects
mod static_object;

/// Dynamic objects
mod dynamic_object;

use crate::mask::{LayerFilter, Mask};
use parry::{
    bounding_volume::Aabb,
    math::{Isometry, Real, Vector},
    query::{self, Contact, ShapeCastHit, ShapeCastOptions},
    shape::Shape,
};

// re-export types
pub use dynamic_object::DynamicObject;
pub use static_object::StaticObject;

/// Trait implemented for static and dynamic bodies
pub trait Object {
    /// Access the shape assigned to this body
    fn shape(&self) -> &dyn Shape;

    /// Access the isometry of this shape
    fn isometry(&self) -> &Isometry<Real>;

    /// Create an Axis-Aligned Bounding Box for this body
    fn aabb(&self) -> Aabb;

    /// Return true if the object is dynamic
    fn is_dynamic(&self) -> bool;

    /// Return true if the object is a trigger area
    fn is_trigger_area(&self) -> bool;

    /// Access the layer and filter of this object
    fn layer_filter(&self) -> &LayerFilter;

    /// Return true if the two objects can interact if their layer and filter match
    #[inline]
    fn can_interact(&self, other: &Self) -> bool {
        self.layer_filter().can_interact(*other.layer_filter())
    }

    /// Return true if this object pass the given filter
    #[inline]
    fn pass_filter(&self, filter: Mask) -> bool {
        (self.layer_filter().layer & filter) != 0
    }

    /// Get the velocity of the body (if it has one)
    #[inline]
    fn velocity(&self) -> Vector<Real> {
        Vector::default()
    }

    /// Try to cast the object into a static body
    #[inline]
    fn as_static(&self) -> Option<&StaticObject> {
        None
    }

    /// Try to cast the object into a kinematic body
    #[inline]
    fn as_dynamic(&self) -> Option<&DynamicObject> {
        None
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
