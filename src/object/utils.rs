//! Utilities for the physics engine

use parry::{
    math::{Point, Real, Vector},
    na::Unit,
};

/// Accumulate collision results
#[derive(Debug, Default, Clone, Copy)]
pub struct Accumulator {
    /// Accumulated position
    position: Vector<Real>,

    /// Accumulated normal
    normal: Vector<Real>,

    /// Count the number of collisions
    count: usize,
}

impl Accumulator {
    /// Add an collision to this accumulator
    pub fn add_collision(
        &mut self,
        point: &Point<Real>,
        normal: &Unit<Vector<Real>>,
        distance: Real,
    ) {
        self.position += point.coords;
        self.normal += normal.into_inner();
        self.count += 1;
    }
}
