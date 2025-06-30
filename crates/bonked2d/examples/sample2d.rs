use bonked2d::{
    make_shared,
    object::{
        kinematic_body::KinematicBody,
        payload::{Payload, SharedPayload, WeakPayload},
        static_body::StaticBody,
        trigger_area::TriggerArea,
        Object,
    },
    world::World,
};
use macroquad::{miniquad::window, prelude::*};
use parry2d::{
    math::{Isometry, Point, Real, Vector},
    shape::{Ball, Capsule, Cuboid, Shape},
};
use spin::RwLock;
use std::{any::Any, sync::Arc, u32};

#[macroquad::main("2D")]
async fn main() {
    const THICKNESS: f32 = 0.05;
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
        Body::new_box([0.0, -0.5], BLUE, BodyData::Static, [20.0, 1.0]),
        {
            let mut body = Body::new_capsule([0.0, 10.0], RED, BodyData::Kinematic(1.0), 1.0, 2.0);
            body.set_velocity(&to_nalgebra([0.0, -1.0]));
            body
        },
        {
            let mut body =
                Body::new_capsule([0.5, 15.0], ORANGE, BodyData::Kinematic(1.0), 1.0, 2.0);
            body.set_velocity(&to_nalgebra([0.0, -1.5]));
            body
        },
        Body::new_box(
            [5.0, 2.5],
            MAGENTA,
            BodyData::Trigger(|flag: WeakPayload, _| {
                if let Some(flag) = flag.get() {
                    let mut flag_guard = flag.write();
                    if let Some(flag) = flag_guard.as_any_mut().downcast_mut::<Flag>() {
                        if !flag.0 {
                            flag.0 = true;
                            println!("Object hit trigger !");
                        }
                    }
                }
            }),
            [5.0, 5.0],
        ),
    ];

    for body in bodies.iter() {
        match body.body_type.clone() {
            BodyType::Static(body) => world.add_static(body),
            BodyType::Kinematic(body) => world.add_kinematic(body),
            BodyType::Trigger(trigger) => world.add_trigger(trigger),
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
    Trigger(Arc<RwLock<TriggerArea>>),
}

#[repr(C)]
#[derive(Clone, Copy)]
enum ShapeType {
    Box,
    Ball,
    Capsule,
}

enum BodyData {
    Static,
    Kinematic(f32),
    Trigger(fn(WeakPayload, &mut KinematicBody)),
}

type V2 = [f32; 2];

impl Body {
    fn new_box(pos: V2, color: Color, data: BodyData, size: V2) -> Self {
        let shape = Arc::new(Cuboid::new(to_nalgebra(size) * 0.5));
        let (body_type, ptr) = make_body(pos, data, shape);
        Self {
            body_type,
            ptr,
            shape_type: ShapeType::Box,
            color,
        }
    }

    fn new_ball(pos: V2, color: Color, data: BodyData, diameter: f32) -> Self {
        let shape = Arc::new(Ball::new(diameter * 0.5));
        let (body_type, ptr) = make_body(pos, data, shape);
        Self {
            body_type,
            ptr,
            shape_type: ShapeType::Ball,
            color,
        }
    }

    fn new_capsule(pos: V2, color: Color, data: BodyData, diameter: f32, height: f32) -> Self {
        let radius = diameter * 0.5;
        let half = (height * 0.5 - radius).max(0.0);
        let a = Point::new(0.0, half);
        let b = Point::new(0.0, -half * 0.5);
        let shape = Arc::new(Capsule::new(a, b, radius));
        let (body_type, ptr) = make_body(pos, data, shape);
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
    data: BodyData,
    shape: Arc<dyn Shape>,
) -> (BodyType, Arc<RwLock<dyn Object>>) {
    let pos = Isometry::new(to_nalgebra(pos), 0.0);
    match data {
        BodyData::Static => {
            let ptr = make_shared(StaticBody::new(shape, pos, SharedPayload::None, u32::MAX));
            (BodyType::Static(ptr.clone()), ptr)
        }
        BodyData::Kinematic(weight) => {
            let ptr = make_shared(KinematicBody::new(
                shape,
                pos,
                SharedPayload::None,
                u32::MAX,
                u32::MAX,
                weight,
                false,
            ));
            (BodyType::Kinematic(ptr.clone()), ptr)
        }
        BodyData::Trigger(callback) => {
            let ptr = make_shared(TriggerArea::new(
                shape,
                pos,
                SharedPayload::new(Flag::default()),
                u32::MAX,
                callback,
            ));
            (BodyType::Trigger(ptr.clone()), ptr)
        }
    }
}

#[derive(Default)]
struct Flag(bool);

impl Payload for Flag {
    fn payload_type(&self) -> usize {
        0
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
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
