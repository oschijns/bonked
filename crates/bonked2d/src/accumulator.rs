use parry2d::{
    math::{Isometry, Point, Real, UnitVector, Vector},
    na::{Translation, UnitComplex},
};

/// Collision accumulator
pub trait Accumulator<A>: Send + Sync {
    /// Reset the accumulator for a new tick
    fn reset(&mut self, current_position: &Isometry<Real>, current_velocity: &Vector<Real>);

    /// Add the contact point, normal and velocity to this accumulator
    fn add_contact(
        &mut self,
        point: &Point<Real>,
        normal: &UnitVector<Real>,
        velocity: &Vector<Real>,
        attributes: &A,
    );

    /// Get the position
    fn get_position(&self) -> Option<Isometry<Real>>;

    /// Get the velocity
    fn get_velocity(&self) -> Option<Vector<Real>>;
}

/// Example of implementation of an accumulator
pub struct DefaultAccumulator {
    /// Assuming the collider is a sphere this indicate
    /// how far the center point should be offset.
    radius: Real,

    /// The rotation of the object
    rotation: UnitComplex<Real>,

    /// Resolved average position
    position: Vector<Real>,

    /// Count the number of contact that have been added
    count: usize,
}

impl DefaultAccumulator {
    /// Create a new accumulator with the provided shape radius
    pub fn new(radius: Real) -> Self {
        Self {
            radius,
            rotation: Default::default(),
            position: Default::default(),
            count: 0,
        }
    }
}

impl<A> Accumulator<A> for DefaultAccumulator {
    /// Reset the accumulator
    fn reset(&mut self, current_position: &Isometry<Real>, _current_velocity: &Vector<Real>) {
        self.rotation = current_position.rotation;
        self.position = Default::default();
        self.count = 0;
    }

    /// Add the contact while ignoring the attributes
    fn add_contact(
        &mut self,
        point: &Point<Real>,
        normal: &UnitVector<Real>,
        _velocity: &Vector<Real>,
        _attributes: &A,
    ) {
        let normal = normal.into_inner();
        self.position += point.coords + normal * self.radius;
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
