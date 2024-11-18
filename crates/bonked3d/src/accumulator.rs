use crate::math::is_null;
use parry3d::math::{Point, Real, UnitVector, Vector};

/// Collision accumulator
pub trait Accumulator<A>: Send + Sync {
    /// Add the contact point and normal to this accumulator
    fn add_contact(&mut self, point: &Point<Real>, normal: &UnitVector<Real>, attributes: &A);

    /// Add the contact point, the normal and the velocity
    /// of the other object to this accumulator.
    fn add_contact_with_velocity(
        &mut self,
        point: &Point<Real>,
        normal: &UnitVector<Real>,
        attributes: &A,
        velocity: &Vector<Real>,
    );
}

/// Default accumulator
pub struct DefaultAccumulator {
    /// Assuming the collider is a sphere this indicate
    /// how far the center point should be offset.
    radius: Real,

    /// Resolved average position
    position: Vector<Real>,

    /// Average normal direction
    normal: Vector<Real>,

    /// Count the number of contact that have been added
    count: usize,
}

impl DefaultAccumulator {
    /// Create a new accumulator with the provided shape radius
    pub fn new(radius: Real) -> Self {
        Self {
            radius,
            position: Default::default(),
            normal: Default::default(),
            count: 0,
        }
    }

    /// Get the averaged position
    pub fn get_position(&self) -> Option<Point<Real>> {
        if self.count > 0 {
            Some(Point::from(self.position / self.count as Real))
        } else {
            None
        }
    }

    /// Get the averaged normal
    pub fn get_normal(&self) -> Option<UnitVector<Real>> {
        if self.count > 0 && !is_null(&self.normal) {
            Some(UnitVector::new_normalize(self.normal))
        } else {
            None
        }
    }
}

impl<A> Accumulator<A> for DefaultAccumulator {
    fn add_contact(&mut self, point: &Point<Real>, normal: &UnitVector<Real>, _attributes: &A) {
        let normal = normal.into_inner();
        self.position += point.coords + normal * self.radius;
        self.normal += normal;
        self.count += 1;
    }

    fn add_contact_with_velocity(
        &mut self,
        point: &Point<Real>,
        normal: &UnitVector<Real>,
        attributes: &A,
        _velocity: &Vector<Real>,
    ) {
        // simply add the contact and ignore the velocity
        self.add_contact(point, normal, attributes);
    }
}

#[cfg(test)]
mod tests {}
