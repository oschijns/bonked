use parry3d::math::{Isometry, Real, Translation, Vector};

use crate::Position;

impl Position {
    /// Get the end-point of
    pub fn get_end_point(&self, displacement: Vector<Real>) -> Isometry<Real> {
        let mut end_pos = self.0;
        end_pos.append_translation_mut(&Translation::from(displacement));
        end_pos
    }
}
