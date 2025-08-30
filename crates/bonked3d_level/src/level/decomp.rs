//! Handle convex decomposition for an arbitrary mesh

use crate::level::{
    GenericMesh, Index, LevelPart, List, ToLevelIndex, ToLevelVertex, ToParryIndex, ToParryPoint,
    Vector,
};
use alloc::vec::Vec;
use parry::{
    math::{self, Point},
    transformation::vhacd::{VHACD, VHACDParameters},
};

/// Error encountered when performing a convex decomposition
#[derive(Debug, thiserror::Error)]
pub enum ConvertError {
    #[error("Number of normals {0} do not match the number of vertices {1}")]
    MismatchNormals(usize, usize),

    #[error("Number of colors {0} do not match the number of vertices {1}")]
    MismatchColors(usize, usize),

    #[error("Number of UV coordinates {0} do not match the number of vertices {1}")]
    MismatchUVs(usize, usize),

    #[error("Could not find the point {0} in the initial buffer when doing convex decomposition.")]
    UnknownPoint(Point<math::Real>),
}

impl<'m, V, I> GenericMesh<'m, V, I>
where
    V: ToParryPoint + ToLevelVertex,
    I: ToParryIndex + ToLevelIndex,
{
    /// Convert the mesh into a level part
    pub fn to_level_part(&self) -> Result<LevelPart, ConvertError> {
        // start with convex decomposition as it is the step most likely to fail
        let hull_indices = self.decompose()?;

        // allocate buffers to store the mesh data
        let count = self.vertices.len();
        let mut positions = Vec::with_capacity(count);
        let mut normals = Vec::with_capacity(count);
        let mut colors = Vec::with_capacity(count);
        let mut uvs = Vec::with_capacity(count);

        // fill the buffers with data
        for vertex in self.vertices {
            positions.push(vertex.get_position());
            if let Some(normal) = vertex.get_normal() {
                normals.push(normal);
            }
            if let Some(color) = vertex.get_color() {
                colors.push(color);
            }
            if let Some(uv) = vertex.get_uv() {
                uvs.push(uv);
            }
        }

        // check each list of extra vertex data
        macro_rules! check {
            ($list:ident, $err:ident) => {{
                let list_count = $list.len();
                if list_count == 0 {
                    None
                } else if list_count == count {
                    Some($list)
                } else {
                    return Err(ConvertError::$err(list_count, count));
                }
            }};
        }
        let normals = check!(normals, MismatchNormals);
        let colors = check!(colors, MismatchColors);
        let uvs = check!(uvs, MismatchUVs);

        // compose the main list of indices
        let mut indices = Vec::with_capacity(self.indices.len());
        for index in self.indices {
            indices.push(index.get_index());
        }

        Ok(LevelPart {
            positions: List(positions),
            normals,
            colors,
            uvs,
            indices: List(indices),
            hull_indices,
        })
    }
}

impl<'m, V, I> GenericMesh<'m, V, I>
where
    V: ToParryPoint,
    I: ToParryIndex,
{
    /// Perform a convex decomposition of the mesh
    fn decompose(&self) -> Result<List<List<Vector<Index, 3>>>, ConvertError> {
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
        let find = |point: Point<math::Real>| -> Result<usize, ConvertError> {
            for (i, pt) in points.iter().enumerate() {
                if point == *pt {
                    return Ok(i);
                }
            }
            Err(ConvertError::UnknownPoint(point))
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
                debug_assert!(
                    check![ix] && check![iy] && check![iz],
                    "Some indexes in [{ix}, {iy}, {iz}] are out of bound",
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
