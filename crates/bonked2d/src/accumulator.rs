use crate::math::is_null;
use parry2d::{
    math::{Isometry, Point, Real, UnitVector, Vector},
    na::{Translation, UnitComplex},
};

/// Collision accumulator
pub trait Accumulator<A>: Send + Sync {
    /// Reset the accumulator for a new tick
    fn reset(&mut self);

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

    /// Get the position
    fn get_position(&self) -> Option<Isometry<Real>>;

    /// Get the normal
    fn get_normal(&self) -> Option<UnitVector<Real>>;

    /// Get the velocity
    fn get_velocity(&self) -> Option<Vector<Real>>;
}

/// Default accumulator
pub struct DefaultAccumulator {
    /// Assuming the collider is a sphere this indicate
    /// how far the center point should be offset.
    radius: Real,

    /// The rotation of the object
    rotation: UnitComplex<Real>,

    /// Resolved average position
    position: Vector<Real>,

    /// Average normal direction
    normal: Vector<Real>,

    /// Count the number of contact that have been added
    count: usize,
}

impl DefaultAccumulator {
    /// Create a new accumulator with the provided shape radius
    pub fn new(radius: Real, rotation: UnitComplex<Real>) -> Self {
        Self {
            radius,
            rotation,
            position: Default::default(),
            normal: Default::default(),
            count: 0,
        }
    }
}

impl<A> Accumulator<A> for DefaultAccumulator {
    /// Reset the accumulator
    fn reset(&mut self) {
        self.position = Default::default();
        self.normal = Default::default();
        self.count = 0;
    }

    /// Add the contact while ignoring the attributes
    fn add_contact(&mut self, point: &Point<Real>, normal: &UnitVector<Real>, _attributes: &A) {
        let normal = normal.into_inner();
        self.position += point.coords + normal * self.radius;
        self.normal += normal;
        self.count += 1;
    }

    /// Add the contact while ignoring the attributes and the velocity
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

    /// Get the averaged position
    fn get_position(&self) -> Option<Isometry<Real>> {
        if self.count > 0 {
            let pos = self.position / self.count as Real;
            Some(Isometry::from_parts(Translation::from(pos), self.rotation))
        } else {
            None
        }
    }

    /// Get the averaged normal
    fn get_normal(&self) -> Option<UnitVector<Real>> {
        if self.count > 0 && !is_null(&self.normal) {
            Some(UnitVector::new_normalize(self.normal))
        } else {
            None
        }
    }

    /// Return a null velocity
    fn get_velocity(&self) -> Option<Vector<Real>> {
        None
    }
}

#[cfg(test)]
mod tests {}
