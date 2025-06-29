use bonked3d::{
    make_shared,
    object::{
        kinematic_body::KinematicBody, static_body::StaticBody, trigger_area::TriggerArea, Object,
    },
    world::World,
};
use macroquad::prelude::*;
use parry3d::{
    math::{Isometry, Point, Real, Vector},
    shape::{Ball, Capsule, Cuboid, Cylinder, Shape},
};
use spin::RwLock;
use std::{sync::Arc, u32};

#[macroquad::main("3D")]
async fn main() {
    let camera_speed = 30.0f32.to_radians();
    let mut cam_ang = 0.0f32;

    let (mut world, bodies) = build_world();

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

        for body in bodies.iter() {
            let b = body.ptr.read();
            let shape = b.shape();
            let pos = to_glam(b.isometry().translation.vector);

            match body.shape_type {
                ShapeType::Box => {
                    let shape = shape.as_cuboid().unwrap();
                    let size = to_glam(shape.half_extents) * 2.0;
                    draw_cube_wires(pos, size, body.color);
                }
                ShapeType::Ball => {
                    let shape = shape.as_ball().unwrap();
                    draw_sphere_wires(pos, shape.radius, None, body.color);
                }
                ShapeType::Cylinder => {
                    let shape = shape.as_cylinder().unwrap();
                    draw_cylinder_wires(
                        pos,
                        shape.radius,
                        shape.radius,
                        shape.half_height * 2.0,
                        None,
                        body.color,
                    );
                }
                ShapeType::Capsule => {
                    let shape = shape.as_capsule().unwrap();
                    draw_sphere_wires(
                        pos + to_glam(shape.segment.a.coords),
                        shape.radius,
                        None,
                        body.color,
                    );
                    draw_sphere_wires(
                        pos + to_glam(shape.segment.b.coords),
                        shape.radius,
                        None,
                        body.color,
                    );
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
        Body::new_box([0.0, -0.5, 0.0], BLUE, BodyData::Static, [20.0, 1.0, 20.0]),
        {
            let mut body =
                Body::new_capsule([0.0, 10.0, 0.0], RED, BodyData::Kinematic(1.0), 1.0, 2.0);
            body.set_velocity(&to_nalgebra([0.0, -1.0, 0.0]));
            body
        },
        {
            let mut body =
                Body::new_capsule([0.5, 15.0, 0.5], ORANGE, BodyData::Kinematic(1.0), 1.0, 2.0);
            body.set_velocity(&to_nalgebra([0.0, -1.5, 0.0]));
            body
        },
        Body::new_box(
            [5.0, 2.5, 5.0],
            MAGENTA,
            BodyData::Trigger(|_, _| {
                println!("Object hit trigger !");
            }),
            [5.0, 5.0, 5.0],
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
    Cylinder,
}

enum BodyData {
    Static,
    Kinematic(f32),
    Trigger(fn(&mut (), &mut KinematicBody)),
}

type V3 = [f32; 3];

impl Body {
    fn new_box(pos: V3, color: Color, data: BodyData, size: V3) -> Self {
        let shape = Arc::new(Cuboid::new(to_nalgebra(size) * 0.5));
        let (body_type, ptr) = make_body(pos, data, shape);
        Self {
            body_type,
            ptr,
            shape_type: ShapeType::Box,
            color,
        }
    }

    fn new_ball(pos: V3, color: Color, data: BodyData, diameter: f32) -> Self {
        let shape = Arc::new(Ball::new(diameter * 0.5));
        let (body_type, ptr) = make_body(pos, data, shape);
        Self {
            body_type,
            ptr,
            shape_type: ShapeType::Ball,
            color,
        }
    }

    fn new_capsule(pos: V3, color: Color, data: BodyData, diameter: f32, height: f32) -> Self {
        let radius = diameter * 0.5;
        let half = (height * 0.5 - radius).max(0.0);
        let a = Point::new(0.0, half, 0.0);
        let b = Point::new(0.0, -half * 0.5, 0.0);
        let shape = Arc::new(Capsule::new(a, b, radius));
        let (body_type, ptr) = make_body(pos, data, shape);
        Self {
            body_type,
            ptr,
            shape_type: ShapeType::Capsule,
            color,
        }
    }

    fn new_cylinder(pos: V3, color: Color, data: BodyData, diameter: f32, height: f32) -> Self {
        let shape = Arc::new(Cylinder::new(height * 0.5, diameter * 0.5));
        let (body_type, ptr) = make_body(pos, data, shape);
        Self {
            body_type,
            ptr,
            shape_type: ShapeType::Cylinder,
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
    pos: V3,
    data: BodyData,
    shape: Arc<dyn Shape>,
) -> (BodyType, Arc<RwLock<dyn Object>>) {
    let pos = Isometry::new(to_nalgebra(pos), Vector::zeros());
    match data {
        BodyData::Static => {
            let ptr = make_shared(StaticBody::new(shape, pos, (), u32::MAX));
            (BodyType::Static(ptr.clone()), ptr)
        }
        BodyData::Kinematic(weight) => {
            let ptr = make_shared(KinematicBody::new(
                shape,
                pos,
                (),
                u32::MAX,
                u32::MAX,
                weight,
            ));
            (BodyType::Kinematic(ptr.clone()), ptr)
        }
        BodyData::Trigger(callback) => {
            let ptr = make_shared(TriggerArea::new(shape, pos, (), u32::MAX, callback));
            (BodyType::Trigger(ptr.clone()), ptr)
        }
    }
}

/// Converts a `Vector<f32>` to a `Vec3`.
#[inline]
fn to_glam(v: Vector<f32>) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}

/// Converts a `[f32; 3]` to a `Vector<f32>`.
#[inline]
fn to_nalgebra(v: V3) -> Vector<f32> {
    Vector::new(v[0], v[1], v[2])
}
