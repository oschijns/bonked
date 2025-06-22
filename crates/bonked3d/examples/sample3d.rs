use bonked3d::{
    make_shared,
    object::{kinematic_body::KinematicBody, static_body::StaticBody, Object},
    world::World,
};
use macroquad::prelude::*;
use parry3d::{
    math::{Isometry, Point, Real, Vector},
    shape::{Capsule, Cuboid, Shape},
};
use std::sync::Arc;

#[macroquad::main("3D")]
async fn main() {
    const EPSILON: Real = 0.0001;

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
        for astatic in world.statics().iter() {
            let s = astatic.read();
            let pos = to_vec3(s.isometry().translation.vector);
            let size = to_vec3(s.shape().as_cuboid().unwrap().half_extents) * 2.0;
            draw_cube_wires(pos, size, BLUE);
        }

        for kinematic in world.kinematics().iter() {
            let k = kinematic.read();
            let pos = to_vec3(k.isometry().translation.vector);
            let cap = k.shape().as_capsule().unwrap();
            draw_cylinder_wires(pos, cap.radius, cap.radius, cap.height(), None, RED);
        }

        world.update(delta, EPSILON);

        next_frame().await
    }
}

fn build_world() -> World {
    let mut world = World::with_capacity(1, 1, 0);
    world.add_static(make_shared(StaticBody::new(
        new_box(20.0, 1.0, 20.0),
        Isometry::new(Vector::new(0.0, 0.0, 0.0), Vector::zeros()),
        u32::MAX,
    )));
    world.add_kinematic(make_shared(KinematicBody::new(
        new_capsule(1.0, 2.0),
        Isometry::new(Vector::new(0.0, 1.0, 0.0), Vector::zeros()),
        1.0,
        1,
        u32::MAX,
    )));
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

fn new_box(x: f32, y: f32, z: f32) -> Arc<dyn Shape> {
    let size = Vector::new(x, y, z) * 0.5;
    Arc::new(Cuboid::new(size))
}

fn new_capsule(diameter: f32, height: f32) -> Arc<dyn Shape> {
    let radius = diameter * 0.5;
    let half = (height * 0.5 - radius).max(0.0);
    let a = Point::new(0.0, half, 0.0);
    let b = Point::new(0.0, -half * 0.5, 0.0);
    Arc::new(Capsule::new(a, b, radius))
}

/// Converts a `Vector<f32>` to a `Vec3`.
#[inline]
fn to_vec3(v: Vector<f32>) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}
