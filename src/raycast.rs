use crate::{InternalRay, Mask, RayCaster, NULL_VEL};
use parry::{
    bounding_volume::Aabb,
    math::{Point, Real, Vector},
    na::ComplexField,
    query::Ray,
};

impl RayCaster {
    /// Create a raycaster from two points
    pub fn from_points(
        origin: &Point<Real>,
        target: &Point<Real>,
        solid: bool,
        mask: Mask,
    ) -> Self {
        Self {
            internal: InternalRay::from_points(origin, target),
            solid,
            mask,
        }
    }

    /// Create a raycaster from a point and a velocity
    pub fn from_velocity(
        origin: &Point<Real>,
        velocity: &Vector<Real>,
        delta_time: Real,
        solid: bool,
        mask: Mask,
    ) -> Self {
        Self {
            internal: InternalRay::from_velocity(origin, velocity, delta_time),
            solid,
            mask,
        }
    }

    /// Set the points of a raycaster
    pub fn set_points(&mut self, origin: &Point<Real>, target: &Point<Real>) -> &Self {
        self.internal = InternalRay::from_points(origin, target);
        self
    }

    /// Set the origin and the velocity of the raycaster
    pub fn set_point_and_velocity(
        &mut self,
        origin: &Point<Real>,
        velocity: &Vector<Real>,
        delta_time: Real,
    ) -> &Self {
        self.internal = InternalRay::from_velocity(origin, velocity, delta_time);
        self
    }
}

impl InternalRay {
    /// Create a raycast which is a single point
    #[inline]
    fn new_null(origin: &Point<Real>) -> Self {
        Self {
            ray: Ray::new(*origin, NULL_VEL),
            length: 0.0,
            aabb: Aabb::new(*origin, *origin),
        }
    }

    /// Create a raycaster from two points
    fn from_points(origin: &Point<Real>, target: &Point<Real>) -> Self {
        if *origin == *target {
            // if origin and target overlap, the raycast is a single point
            Self::new_null(origin)
        } else {
            // compute the distance from origin to target
            let diff = target - origin;
            let length = diff.magnitude();

            // compute the AABB around the ray
            let mut aabb = Aabb::new_invalid();
            aabb.take_point(*origin);
            aabb.take_point(*target);

            Self {
                ray: Ray::new(*origin, diff / length),
                length,
                aabb,
            }
        }
    }

    /// Create a raycaster from a point and a velocity
    fn from_velocity(origin: &Point<Real>, velocity: &Vector<Real>, delta_time: Real) -> Self {
        let sqr_speed = velocity.magnitude_squared();
        if sqr_speed == 0.0 {
            // if the velocity is null, the raycast is a single point
            Self::new_null(origin)
        } else {
            // apply square root now
            let speed = <Real as ComplexField>::sqrt(sqr_speed);

            // compute the AABB around the ray
            let mut aabb = Aabb::new_invalid();
            aabb.take_point(*origin);
            aabb.take_point(Point::from(origin + velocity * delta_time));

            Self {
                ray: Ray::new(*origin, velocity / speed),
                length: speed * delta_time,
                aabb,
            }
        }
    }
}
