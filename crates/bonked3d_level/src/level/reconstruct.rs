//! Reconstruct the convex hulls from the level part.
//! Since the indices of the hull are indexing a single buffer of vertices,
//! we have to extract the vertices used by each individual hull and reassign
//! a new index for each vertex.

use crate::level::{Index, Real, ToParryPoint, Vector};
use core::cmp::min;
use parry::shape::ConvexPolyhedron;

/// Store data for generating convex polyhedrons
struct ReIndexer<'v> {
    /// The initial vertex buffer
    points: &'v [Vector<Real, 3>],

    /// Temporary buffer for storing indexes re-assignment
    indexes_buffer: Vec<u32>,
}

impl<'v> ReIndexer<'v> {
    /// Allocate an index buffer for the provided set of points
    fn new(points: &'v [Vector<Real, 3>]) -> Self {
        // allocate the indexes buffer and fill it with an invalid index
        let mut indexes_buffer = Vec::with_capacity(points.len());
        indexes_buffer.resize(points.len(), u32::MAX);

        Self {
            points,
            indexes_buffer,
        }
    }

    /// Given the indices of a convex hull, generate a convex polyhedron.
    fn make_convex_hull(&mut self, hull_indices: &[Vector<Index, 3>]) -> Option<ConvexPolyhedron> {
        // reset the index buffer
        self.indexes_buffer.fill(u32::MAX);

        // allocate a buffer to store the points, necessarly capped
        // by the vertex buffer or the number of indices in the hull
        let count = min(self.points.len(), hull_indices.len() * 3);
        let mut points = Vec::with_capacity(count);
        let mut indices = Vec::with_capacity(hull_indices.len());

        // iterate over each index
        for tri in hull_indices {
            // compose a new triangle
            let mut new_tri = [u32::MAX; 3];
            for (i, &idx) in tri.0.iter().enumerate() {
                // is there a new index already assigned ?
                let new_idx = self.indexes_buffer[idx as usize];
                if new_idx == u32::MAX {
                    // no index assigned yet, create a new one
                    new_tri[i] = points.len() as u32;
                    let point = &self.points[idx as usize];
                    points.push(point.to_parry());
                } else {
                    new_tri[i] = new_idx;
                }
            }

            // normally, we should have identified all of the indexes
            macro_rules! check {
                ($i:literal) => {
                    new_tri[$i] != u32::MAX
                };
            }
            assert!(
                check![0] && check![1] && check![2],
                "Some vertices of the triangle [{}, {}, {}] could not be identified",
                new_tri[0],
                new_tri[1],
                new_tri[2]
            );

            indices.push(new_tri);
        }

        // Build the convex polyhedron
        ConvexPolyhedron::from_convex_mesh(points, &indices)
    }
}
