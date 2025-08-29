//! Handle convex decomposition for an arbitrary mesh

use crate::level::{Index, List, Vector};
use parry::{
    math::{self, Point},
    transformation::vhacd::{VHACD, VHACDParameters},
};

/// Mesh data to convert
pub struct Mesh<'m, V, I>
where
    V: ToVertex,
    I: ToIndex,
{
    /// Vertices data to serialize
    vertices: &'m [V],

    /// Indices to compose the triangles
    indices: &'m [I],
}

pub trait ToVertex {
    /// convert to be used in VHACD
    fn to_vhacd(&self) -> Point<math::Real>;
}

pub trait ToIndex {
    /// convert to be used in VHACD
    fn to_vhacd(&self) -> [u32; 3];
}

impl<'m, V, I> Mesh<'m, V, I>
where
    V: ToVertex,
    I: ToIndex,
{
    /// Perform a convex decomposition of the mesh
    fn decompose(&self) -> Result<List<List<Vector<Index, 3>>>, ()> {
        let params = VHACDParameters::default();

        // convert mesh data into a format that can be used by the VHACD decomposition
        let points = self
            .vertices
            .iter()
            .map(|v| v.to_vhacd())
            .collect::<Vec<_>>();
        let indices = self
            .indices
            .iter()
            .map(|i| i.to_vhacd())
            .collect::<Vec<_>>();

        // compute the convex decomposition
        let vhacd = VHACD::decompose(&params, &points, &indices, true);
        let hulls = vhacd.compute_exact_convex_hulls(&points, &indices);

        // re-identify the vertices used to compose the convex hulls
        let mut result = Vec::with_capacity(hulls.len());

        // find the index of the point in the initial vertices buffer
        let find = |point: Point<math::Real>| {
            for (i, pt) in points.iter().enumerate() {
                if point == *pt {
                    return Ok(i);
                }
            }
            Err(())
        };

        // for each convex hull generated find back the
        // indexes relative to the initial vertices buffer
        for (pts, idx) in hulls {
            let mut new_indices = Vec::with_capacity(idx.len());

            // iterate over each triplet of indexes
            for [ix, iy, iz] in idx {
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
