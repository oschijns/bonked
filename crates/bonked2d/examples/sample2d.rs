use bonked2d::{
    make_shared,
    object::{
        kinematic_body::KinematicBody, static_body::StaticBody, trigger_area::TriggerArea, Object,
    },
    world::World,
    Mask,
};
use macroquad::{miniquad::window, prelude::*};
use parry2d::{
    math::{Isometry, Point, Real, Vector},
    shape::{Ball, Capsule, Cuboid, Shape},
};
use std::sync::Arc;

#[macroquad::main("2D")]
async fn main() {
    const ZOOM: f32 = 0.1;
    let mut world = build_world();

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

        macro_rules! draw {
            ($body:ident, $color:ident) => {
                let body = $body.read();
                let shape = AShape::new(body.shape());
                let pos = to_glam(body.isometry().translation.vector);
                shape.draw(pos, $color);
            };
        }

        for body in world.statics().iter() {
            draw!(body, BLUE);
        }

        for body in world.kinematics().iter() {
            draw!(body, RED);
        }

        for body in world.triggers().iter() {
            draw!(body, GREEN);
        }

        world.update(delta);

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
}

impl<'s> AShape<'s> {
    fn new(shape: &'s dyn Shape) -> Self {
        if let Some(shape) = shape.as_cuboid() {
            Self::Box(shape)
        } else if let Some(shape) = shape.as_ball() {
            Self::Ball(shape)
        } else if let Some(shape) = shape.as_capsule() {
            Self::Capsule(shape)
        } else {
            Self::None
        }
    }

    fn draw(&'s self, pos: Vec2, color: Color) {
        const THICKNESS: f32 = 0.05;
        match self {
            Self::Box(shape) => {
                let half = to_glam(shape.half_extents);
                let start = pos - half;
                let size = half * 2.0;
                draw_rectangle_lines(start.x, start.y, size.x, size.y, THICKNESS, color);
            }
            Self::Ball(shape) => {
                draw_circle_lines(pos.x, pos.y, shape.radius, THICKNESS, color);
            }
            Self::Capsule(shape) => {
                let a = pos + to_glam(shape.segment.a.coords);
                let b = pos + to_glam(shape.segment.b.coords);
                draw_circle_lines(a.x, a.y, shape.radius, THICKNESS, color);
                draw_circle_lines(b.x, b.y, shape.radius, THICKNESS, color);
            }
            _ => {}
        }
    }
}

fn build_world() -> World<bool> {
    const EPSILON: Real = 0.0001;
    let mut world = World::with_capacity(EPSILON, 2, 1, 1);

    world.add_static({
        let (shape, isometry) = new_box([0.0, -0.5], [20.0, 1.0]);
        make_shared(StaticBody::new(shape, isometry, (), Mask::MAX))
    });

    world.add_kinematic({
        let (shape, isometry) = new_capsule([0.0, 10.0], 1.0, 2.0);
        let mut body = KinematicBody::new(shape, isometry, (), Mask::MAX, Mask::MAX, 1.0, false);
        body.velocity.y = -1.0;
        make_shared(body)
    });

    world.add_kinematic({
        let (shape, isometry) = new_capsule([0.5, 15.0], 1.0, 2.0);
        let mut body = KinematicBody::new(shape, isometry, (), Mask::MAX, Mask::MAX, 1.0, false);
        body.velocity.y = -1.5;
        make_shared(body)
    });

    world.add_trigger({
        let (shape, isometry) = new_box([5.0, 2.5], [5.0, 5.0]);
        make_shared(TriggerArea::new(
            shape,
            isometry,
            false,
            Mask::MAX,
            |trigger: &mut TriggerArea<bool>, _| {
                let flag = trigger.payload_mut();
                if !*flag {
                    *flag = true;
                    println!("Object entered area");
                }
            },
        ))
    });

    world
}

struct Inputs {
    motion: IVec2,
    jump: bool,
}

impl Inputs {
    fn read() -> Self {
        let mut motion = IVec2::ZERO;

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

fn new_box(pos: V2, size: V2) -> (Arc<dyn Shape>, Isometry<Real>) {
    let shape = Arc::new(Cuboid::new(to_nalgebra(size) * 0.5));
    let pos = Isometry::new(to_nalgebra(pos), 0.0);
    (shape, pos)
}

fn new_ball(pos: V2, diameter: f32) -> (Arc<dyn Shape>, Isometry<Real>) {
    let shape = Arc::new(Ball::new(diameter * 0.5));
    let pos = Isometry::new(to_nalgebra(pos), 0.0);
    (shape, pos)
}

fn new_capsule(pos: V2, diameter: f32, height: f32) -> (Arc<dyn Shape>, Isometry<Real>) {
    let radius = diameter * 0.5;
    let half = (height * 0.5 - radius).max(0.0);
    let a = Point::new(0.0, half);
    let b = Point::new(0.0, -half * 0.5);
    let shape = Arc::new(Capsule::new(a, b, radius));
    let pos = Isometry::new(to_nalgebra(pos), 0.0);
    (shape, pos)
}

type V2 = [Real; 2];

/// Converts a `Vector<Real>` to a `Vec2`.
#[inline]
fn to_glam(v: Vector<Real>) -> Vec2 {
    Vec2::new(v.x as f32, v.y as f32)
}

/// Converts a `[Real; 2]` to a `Vector<Real>`.
#[inline]
fn to_nalgebra(v: V2) -> Vector<Real> {
    Vector::new(v[0], v[1])
}
