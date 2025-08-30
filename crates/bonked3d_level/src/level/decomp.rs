//! Handle convex decomposition for an arbitrary mesh

use crate::level::{GenericMesh, Index, List, ToParryIndex, ToParryPoint, Vector};
use parry::{
    math::{self, Point},
    transformation::vhacd::{VHACD, VHACDParameters},
};

/// Error encountered when performing a convex decomposition
#[derive(Debug, thiserror::Error)]
pub enum DecompError {
    #[error("Could not find the point {0} in the initial buffer.")]
    UnknownPoint(Point<math::Real>),
}

impl<'m, V, I> GenericMesh<'m, V, I>
where
    V: ToParryPoint,
    I: ToParryIndex,
{
    /// Perform a convex decomposition of the mesh
    pub fn decompose(&self) -> Result<List<List<Vector<Index, 3>>>, DecompError> {
        let params = VHACDParameters::default();

        // convert mesh data into a format that can be used by the VHACD decomposition
        let points = self
            .vertices
            .iter()
            .map(|v| v.to_parry())
            .collect::<Vec<_>>();
        let indices = self
            .indices
            .iter()
            .map(|i| i.to_parry())
            .collect::<Vec<_>>();

        // compute the convex decomposition
        let vhacd = VHACD::decompose(&params, &points, &indices, true);
        let hulls = vhacd.compute_exact_convex_hulls(&points, &indices);

        // re-identify the vertices used to compose the convex hulls
        let mut result = Vec::with_capacity(hulls.len());

        // find the index of the point in the initial vertices buffer
        let find = |point: Point<math::Real>| -> Result<usize, DecompError> {
            for (i, pt) in points.iter().enumerate() {
                if point == *pt {
                    return Ok(i);
                }
            }
            Err(DecompError::UnknownPoint(point))
        };

        // for each convex hull generated find back the
        // indexes relative to the initial vertices buffer
        for (pts, tri) in hulls {
            let mut new_indices = Vec::with_capacity(tri.len());

            // iterate over each triplet of indexes
            for [ix, iy, iz] in tri {
                // indexes should be in the range of available points
                macro_rules! check {
                    ($i:ident) => {
                        pts.len() < $i as usize
                    };
                }
                assert!(
                    check![ix] && check![iy] && check![iz],
                    "Some indexes in [{}, {}, {}] are out of bound",
                    ix,
                    iy,
                    iz
                );

                // convert the indices to reference the initial mesh
                let jx = find(pts[ix as usize])? as Index;
                let jy = find(pts[iy as usize])? as Index;
                let jz = find(pts[iz as usize])? as Index;

                new_indices.push(Vector([jx, jy, jz]));
            }
            result.push(List(new_indices));
        }
        Ok(List(result))
    }
}
