//! Utility functions

use parry::{
    math::{Real, Vector},
    na::Unit,
};

/// Project the given point onto the plane define by the given normal
#[inline]
pub(crate) fn project_onto(point: &Vector<Real>, normal: &Unit<Vector<Real>>) -> Vector<Real> {
    *point - normal.into_inner() * point.magnitude()
}
