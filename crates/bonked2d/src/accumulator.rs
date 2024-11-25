use parry2d::{
    math::{Isometry, Real, Vector},
    na::{Translation, UnitComplex},
    query::Contact,
};

/// Collision accumulator
pub trait Accumulator<A>: Send + Sync {
    /// Reset the accumulator for a new tick
    fn reset(&mut self, current_position: &Isometry<Real>, current_velocity: &Vector<Real>);

    /// Add the contact point, normal and velocity to this accumulator
    fn add_contact(&mut self, contact: &Contact, velocity: &Vector<Real>, attributes: &A);

    /// Get the position
    fn get_position(&self) -> Option<Isometry<Real>>;

    /// Get the velocity
    fn get_velocity(&self) -> Option<Vector<Real>>;
}

/// Example of implementation of an accumulator
#[derive(Default, Debug)]
pub struct DefaultAccumulator {
    /// The rotation of the object
    rotation: UnitComplex<Real>,

    /// Resolved average position
    position: Vector<Real>,

    /// Count the number of contact that have been added
    count: usize,
}

impl<A> Accumulator<A> for DefaultAccumulator {
    /// Reset the accumulator
    fn reset(&mut self, current_position: &Isometry<Real>, _current_velocity: &Vector<Real>) {
        self.rotation = current_position.rotation;
        self.position = Default::default();
        self.count = 0;
    }

    /// Add the contact while ignoring the attributes
    fn add_contact(&mut self, contact: &Contact, _velocity: &Vector<Real>, _attributes: &A) {
        self.position += contact.point2.coords + contact.normal2.into_inner() * contact.dist.abs();
        self.count += 1;
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

    /// Return a null velocity
    fn get_velocity(&self) -> Option<Vector<Real>> {
        None
    }
}
