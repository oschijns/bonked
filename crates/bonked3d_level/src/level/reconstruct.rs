//! Reconstruct the convex hulls from the level part.
//! Since the indices of the hull are indexing a single buffer of vertices,
//! we have to extract the vertices used by each individual hull and reassign
//! a new index for each vertex.

use crate::level::{Index, Mesh, Real, Vector};
use alloc::vec::Vec;
use core::cmp::min;
use parry::{
    math::Isometry,
    shape::{Compound, ConvexPolyhedron, SharedShape},
};

impl Mesh {
    /// Use the provided convex hulls to build the physical level geometry
    pub fn build_collider(&self) -> Compound {
        // prepare data to generate the convex hulls
        let mut reindexer = ReIndexer::new(&self.positions.0);
        let mut shapes = Vec::with_capacity(self.hull_indices.0.len());

        // rebuild each convex hull from raw data
        for hull in &self.hull_indices.0 {
            if let Some(convex) = reindexer.make_convex_hull(&hull.0) {
                shapes.push((Isometry::identity(), SharedShape::new(convex)));
            } else {
                debug_assert!(
                    false,
                    "Convex hull at index {} could not be generated.",
                    shapes.len()
                );
            }
        }
        Compound::new(shapes)
    }
}

/// Store data for generating convex polyhedrons
struct ReIndexer<'v> {
    /// The initial vertex buffer
    vertices: &'v [Vector<Real, 3>],

    /// Temporary buffer for storing indexes re-assignment
    indexes_buffer: Vec<u32>,
}

impl<'v> ReIndexer<'v> {
    /// Allocate an index buffer for the provided set of points
    fn new(vertices: &'v [Vector<Real, 3>]) -> Self {
        let count = vertices.len();

        // allocate the indexes buffer and fill it with an invalid index
        let mut indexes_buffer = Vec::with_capacity(count);
        indexes_buffer.resize(count, u32::MAX);

        Self {
            vertices,
            indexes_buffer,
        }
    }

    /// Given the indices of a convex hull, generate a convex polyhedron.
    fn make_convex_hull(&mut self, hull_indices: &[Vector<Index, 3>]) -> Option<ConvexPolyhedron> {
        // reset the index buffer
        self.indexes_buffer.fill(u32::MAX);

        // allocate a buffer to store the points, necessarly capped
        // by the vertex buffer or the number of indices in the hull
        let count = min(self.vertices.len(), hull_indices.len() * 3);
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
                    points.push(self.vertices[idx as usize].parry_point());
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
            debug_assert!(
                check![0] && check![1] && check![2],
                "Some vertices of the triangle [{}, {}, {}] could not be identified",
                new_tri[0],
                new_tri[1],
                new_tri[2]
            );

            indices.push(new_tri);
        }

        // Build the convex polyhedron
        points.shrink_to_fit();
        ConvexPolyhedron::from_convex_mesh(points, &indices)
    }
}
