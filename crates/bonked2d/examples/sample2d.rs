use bonked2d::{
    make_shared,
    object::{kinematic_body::KinematicBody, static_body::StaticBody, Object},
    world::World,
};
use macroquad::{miniquad::window, prelude::*};
use parry2d::{
    math::{Isometry, Point, Real, Vector},
    shape::{Ball, Capsule, Cuboid, Shape},
};
use spin::RwLock;
use std::{sync::Arc, u32};

#[macroquad::main("2D")]
async fn main() {
    const THICKNESS: f32 = 0.02;
    const ZOOM: f32 = 0.1;

    let (mut world, bodies) = build_world();

    loop {
        let delta = get_frame_time();

        // turn the camera
        let input = Inputs::read();
        clear_background(LIGHTGRAY);

        // place a camera to look at the scene
        let size = window::screen_size();
        let ratio = size.0 / size.1;
        set_camera(&Camera2D {
            target: Vec2::new(0.0, 3.0),
            zoom: Vec2::new(ZOOM, ZOOM * -ratio),
            ..Default::default()
        });

        for body in bodies.iter() {
            let b = body.ptr.read();
            let shape = b.shape();
            let pos = to_glam(b.isometry().translation.vector);

            match body.shape_type {
                ShapeType::Box => {
                    let shape = shape.as_cuboid().unwrap();
                    let half = to_glam(shape.half_extents);
                    let start = pos - half;
                    let size = half * 2.0;
                    draw_rectangle_lines(start.x, start.y, size.x, size.y, THICKNESS, body.color);
                }
                ShapeType::Ball => {
                    let shape = shape.as_ball().unwrap();
                    draw_circle_lines(pos.x, pos.y, shape.radius, THICKNESS, body.color);
                }
                ShapeType::Capsule => {
                    let shape = shape.as_capsule().unwrap();
                    let a = pos + to_glam(shape.segment.a.coords);
                    let b = pos + to_glam(shape.segment.b.coords);
                    draw_circle_lines(a.x, a.y, shape.radius, THICKNESS, body.color);
                    draw_circle_lines(b.x, b.y, shape.radius, THICKNESS, body.color);
                }
            }
        }

        world.update(delta);

        // quit the example
        if is_quit_requested() {
            break;
        }

        next_frame().await
    }
}

fn build_world() -> (World, Vec<Body>) {
    const EPSILON: Real = 0.0001;
    let mut world = World::with_capacity(EPSILON, 1, 1, 0);

    let bodies = vec![
        Body::new_box([0.0, -0.5], BLUE, None, [20.0, 1.0]),
        {
            let mut body = Body::new_capsule([0.0, 10.0], RED, Some(1.0), 1.0, 2.0);
            body.set_velocity(&to_nalgebra([0.0, -1.0]));
            body
        },
        {
            let mut body = Body::new_capsule([0.5, 15.0], ORANGE, Some(1.0), 1.0, 2.0);
            body.set_velocity(&to_nalgebra([0.0, -1.1]));
            body
        },
    ];

    for body in bodies.iter() {
        match body.body_type.clone() {
            BodyType::Static(body) => world.add_static(body),
            BodyType::Kinematic(body) => world.add_kinematic(body),
        }
    }
    (world, bodies)
}

struct Inputs {
    motion: IVec2,
    jump: bool,
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

        Inputs {
            motion,
            jump: is_key_pressed(KeyCode::Space),
        }
    }
}

/// A physics body that can be rendered with a color
struct Body {
    body_type: BodyType,
    ptr: Arc<RwLock<dyn Object>>,
    shape_type: ShapeType,
    color: Color,
}

#[derive(Clone)]
enum BodyType {
    Static(Arc<RwLock<StaticBody>>),
    Kinematic(Arc<RwLock<KinematicBody>>),
}

#[repr(C)]
#[derive(Clone, Copy)]
enum ShapeType {
    Box,
    Ball,
    Capsule,
}

type V2 = [f32; 2];

impl Body {
    fn new_box(pos: V2, color: Color, weight: Option<f32>, size: V2) -> Self {
        let shape = Arc::new(Cuboid::new(to_nalgebra(size) * 0.5));
        let (body_type, ptr) = make_body(pos, weight, shape);
        Self {
            body_type,
            ptr,
            shape_type: ShapeType::Box,
            color,
        }
    }

    fn new_ball(pos: V2, color: Color, weight: Option<f32>, diameter: f32) -> Self {
        let shape = Arc::new(Ball::new(diameter * 0.5));
        let (body_type, ptr) = make_body(pos, weight, shape);
        Self {
            body_type,
            ptr,
            shape_type: ShapeType::Ball,
            color,
        }
    }

    fn new_capsule(pos: V2, color: Color, weight: Option<f32>, diameter: f32, height: f32) -> Self {
        let radius = diameter * 0.5;
        let half = (height * 0.5 - radius).max(0.0);
        let a = Point::new(0.0, half);
        let b = Point::new(0.0, -half * 0.5);
        let shape = Arc::new(Capsule::new(a, b, radius));
        let (body_type, ptr) = make_body(pos, weight, shape);
        Self {
            body_type,
            ptr,
            shape_type: ShapeType::Capsule,
            color,
        }
    }

    fn set_velocity(&mut self, vel: &Vector<Real>) {
        if let BodyType::Kinematic(body) = &mut self.body_type {
            body.write().velocity = *vel;
        }
    }
}

fn make_body(
    pos: V2,
    weight: Option<f32>,
    shape: Arc<dyn Shape>,
) -> (BodyType, Arc<RwLock<dyn Object>>) {
    let pos = Isometry::new(to_nalgebra(pos), 0.0);
    if let Some(weight) = weight {
        let ptr = make_shared(KinematicBody::new(shape, pos, weight, u32::MAX, u32::MAX));
        (BodyType::Kinematic(ptr.clone()), ptr)
    } else {
        let ptr = make_shared(StaticBody::new(shape, pos, u32::MAX));
        (BodyType::Static(ptr.clone()), ptr)
    }
}

/// Converts a `Vector<f32>` to a `Vec3`.
#[inline]
fn to_glam(v: Vector<f32>) -> Vec2 {
    Vec2::new(v.x, v.y)
}

/// Converts a `[f32; 2]` to a `Vector<f32>`.
#[inline]
fn to_nalgebra(v: V2) -> Vector<f32> {
    Vector::new(v[0], v[1])
}
