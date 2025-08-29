use obj::{Obj, load_obj};
use parry3d::{
    math::{Point, Real},
    transformation::vhacd::{VHACD, VHACDParameters},
};
use std::{fs::File, io::BufReader};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let input = BufReader::new(File::open("assets/test-mesh.obj")?);
    let model: Obj = load_obj(input)?;

    //println!("Vertices {:#?}", model.vertices);
    //println!("Indices  {:#?}", model.indices);

    // reformat vertices
    let vertices = model
        .vertices
        .iter()
        .map(|v| {
            Point::new(
                v.position[0] as Real,
                v.position[1] as Real,
                v.position[2] as Real,
            )
        })
        .collect::<Vec<_>>();

    // reformat indices
    let indices = model
        .indices
        .as_chunks::<3>()
        .0
        .iter()
        .map(|i| [i[0] as u32, i[1] as u32, i[2] as u32])
        .collect::<Vec<_>>();

    // Create a parry mesh
    let params = VHACDParameters::default();
    let vhacd = VHACD::decompose(&params, &vertices, &indices, true);
    let hulls = vhacd.compute_exact_convex_hulls(&vertices, &indices);
    for (points, indexes) in hulls {
        println!("Hull {:?} {:?} \n\n", points, indexes);
    }

    Ok(())
}
