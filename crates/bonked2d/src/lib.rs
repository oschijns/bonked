use parry2d::math::{Isometry, Real, Vector};

/// Current position of the object for this tick
pub struct Position(pub Isometry<Real>);

/// Position of the object for the next tick
pub struct NextPosition(pub Isometry<Real>);

/// Current velocity of the object for this tick
pub struct Velocity(pub Vector<Real>);

/// Velocity of the object for the next tick
pub struct NextVelocity(pub Vector<Real>);
