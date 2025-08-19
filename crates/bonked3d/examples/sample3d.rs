use bonked3d::world::World;
use macroquad::prelude::*;
use parry3d::{
    math::{Isometry, Point, Real, Vector},
    query::ShapeCastOptions,
    shape::{Ball, Capsule, Cuboid, Cylinder, Shape},
};
use std::sync::Arc;

#[macroquad::main("3D")]
async fn main() {
    let camera_speed = 30.0f32.to_radians();
    let mut cam_ang = 0.0f32;
    let mut world = build_world();

    loop {
        let delta = get_frame_time();

        // turn the camera
        let input = Inputs::read();
        cam_ang += input.camera as f32 * camera_speed * delta;

        clear_background(LIGHTGRAY);

        // place a camera to look at the scene
        set_camera(&Camera3D {
            position: Quat::from_rotation_y(cam_ang) * vec3(-20.0, 15.0, 0.0),
            up: Vec3::Y,
            target: Vec3::ZERO,
            ..Default::default()
        });
        draw_grid(20, 1., BLACK, GRAY);

        macro_rules! draw {
            ($body:ident, $color:ident) => {
                let body = $body.read();
                let shape = AShape::new(body.shape());
                let pos = to_glam(body.isometry().translation.vector);
                shape.draw(pos, $color);
            };
        }

        const EPSILON: Real = 0.0001;
        world.update(ShapeCastOptions {
            max_time_of_impact: delta,
            target_distance: EPSILON,
            stop_at_penetration: true,
            compute_impact_geometry_on_penetration: false,
        });

        // quit the example
        if is_quit_requested() {
            break;
        }

        next_frame().await
    }
}

enum AShape<'s> {
    /// Could not identify the shape
    None,

    /// Box shape
    Box(&'s Cuboid),

    /// Ball shape
    Ball(&'s Ball),

    /// Capsule shape
    Capsule(&'s Capsule),

    /// Cylinder shape
    Cylinder(&'s Cylinder),
}

impl<'s> AShape<'s> {
    fn new(shape: &'s dyn Shape) -> Self {
        if let Some(shape) = shape.as_cuboid() {
            Self::Box(shape)
        } else if let Some(shape) = shape.as_ball() {
            Self::Ball(shape)
        } else if let Some(shape) = shape.as_capsule() {
            Self::Capsule(shape)
        } else if let Some(shape) = shape.as_cylinder() {
            Self::Cylinder(shape)
        } else {
            Self::None
        }
    }

    fn draw(&'s self, pos: Vec3, color: Color) {
        match self {
            Self::Box(shape) => {
                let size = to_glam(shape.half_extents) * 2.0;
                draw_cube_wires(pos, size, color);
            }
            Self::Ball(shape) => {
                draw_sphere_wires(pos, shape.radius, None, color);
            }
            Self::Cylinder(shape) => {
                draw_cylinder_wires(
                    pos,
                    shape.radius,
                    shape.radius,
                    shape.half_height * 2.0,
                    None,
                    color,
                );
            }
            Self::Capsule(shape) => {
                draw_sphere_wires(
                    pos + to_glam(shape.segment.a.coords),
                    shape.radius,
                    None,
                    color,
                );
                draw_sphere_wires(
                    pos + to_glam(shape.segment.b.coords),
                    shape.radius,
                    None,
                    color,
                );
            }
            _ => {}
        }
    }
}

fn build_world() -> World {
    let mut world = World::new();

    world
}

struct Inputs {
    motion: IVec2,
    jump: bool,
    camera: i8,
}

impl Inputs {
    fn read() -> Self {
        let mut motion = IVec2::ZERO;
        let mut camera = 0;

        // motion
        if is_key_down(KeyCode::Left) {
            motion.x -= 1;
        }
        if is_key_down(KeyCode::Right) {
            motion.x += 1;
        }
        if is_key_down(KeyCode::Up) {
            motion.y -= 1;
        }
        if is_key_down(KeyCode::Down) {
            motion.y += 1;
        }

        // camera control
        if is_key_down(KeyCode::Comma) {
            camera -= 1;
        }
        if is_key_down(KeyCode::Period) {
            camera += 1;
        }

        Inputs {
            motion,
            jump: is_key_pressed(KeyCode::Space),
            camera,
        }
    }
}

fn new_box(pos: V3, size: V3) -> (Arc<dyn Shape>, Isometry<Real>) {
    let shape = Arc::new(Cuboid::new(to_nalgebra(size) * 0.5));
    let pos = Isometry::new(to_nalgebra(pos), Vector::zeros());
    (shape, pos)
}

fn new_ball(pos: V3, diameter: f32) -> (Arc<dyn Shape>, Isometry<Real>) {
    let shape = Arc::new(Ball::new(diameter * 0.5));
    let pos = Isometry::new(to_nalgebra(pos), Vector::zeros());
    (shape, pos)
}

fn new_capsule(pos: V3, diameter: f32, height: f32) -> (Arc<dyn Shape>, Isometry<Real>) {
    let radius = diameter * 0.5;
    let half = (height * 0.5 - radius).max(0.0);
    let a = Point::new(0.0, half, 0.0);
    let b = Point::new(0.0, -half * 0.5, 0.0);
    let shape = Arc::new(Capsule::new(a, b, radius));
    let pos = Isometry::new(to_nalgebra(pos), Vector::zeros());
    (shape, pos)
}

fn new_cylinder(pos: V3, diameter: f32, height: f32) -> (Arc<dyn Shape>, Isometry<Real>) {
    let shape = Arc::new(Cylinder::new(height * 0.5, diameter * 0.5));
    let pos = Isometry::new(to_nalgebra(pos), Vector::zeros());
    (shape, pos)
}

type V3 = [Real; 3];

/// Converts a `Vector<Real>` to a `Vec3`.
#[inline]
fn to_glam(v: Vector<Real>) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

/// Converts a `[Real; 3]` to a `Vector<Real>`.
#[inline]
fn to_nalgebra(v: V3) -> Vector<Real> {
    Vector::new(v[0], v[1], v[2])
}
