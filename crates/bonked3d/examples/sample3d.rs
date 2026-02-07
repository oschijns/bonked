use bonked3d::{
    mask::MASK_ALL,
    object::{DynamicObject, Object, StaticObject},
    world::World,
};
use macroquad::prelude::*;
use parry3d::{
    math::{Pose, Real, Rotation, Vector},
    query::ShapeCastOptions,
    shape::{Ball, Capsule, Cuboid, Cylinder, Shape, SharedShape},
};
use std::{
    cell::{Ref, RefCell},
    f32::consts::PI,
};

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

        const EPSILON: Real = 0.0001;
        world.update(
            ShapeCastOptions {
                max_time_of_impact: delta,
                target_distance: EPSILON,
                stop_at_penetration: true,
                compute_impact_geometry_on_penetration: false,
            },
            EPSILON,
        );

        fn draw(obj: Ref<'_, dyn Object>, color: Color) {
            let shape = AShape::new(obj.shape());
            shape.draw(obj.isometry(), color);
        }

        for (_idx, obj) in world.statics().iter() {
            draw(obj.borrow(), BLUE);
        }

        for (_idx, obj) in world.dynamics().iter() {
            draw(obj.borrow(), RED);
        }

        // quit the example
        if is_quit_requested() {
            break;
        }

        next_frame().await
    }
}

macro_rules! pose {
    () => {
        Pose::identity()
    };
    ($x:literal, $y:literal, $z:literal) => {
        Pose::from_translation(Vector::new($x as Real, $y as Real, $z as Real))
    };
    ([$x:literal, $y:literal, $z:literal], [$axis:ident, $ang:literal]) => {
        Pose::from_parts(
            Vector::new($x as Real, $y as Real, $z as Real),
            Rotation::from_axis_angle(Vector::$axis, $ang as Real * PI / 180.0),
        )
    };
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

    fn draw(&'s self, pose: &Pose, color: Color) {
        match self {
            Self::Box(shape) => {
                // TODO: missing rotation
                let t = to_macroquad(pose.translation);
                let size = to_macroquad(shape.half_extents * 2.0);
                draw_cube_wires(t, size, color);
            }
            Self::Ball(shape) => {
                let t = to_macroquad(pose.translation);
                draw_sphere_wires(t, shape.radius, None, color);
            }
            Self::Cylinder(shape) => {
                // TODO: missing rotation
                draw_cylinder_ex(
                    to_macroquad(pose.translation),
                    shape.radius,
                    shape.radius,
                    shape.half_height * 2.0,
                    None,
                    color,
                    DrawCylinderParams {
                        sides: 16,
                        draw_mode: DrawMode::Lines,
                    },
                );
            }
            Self::Capsule(shape) => {
                let t1 = to_macroquad(pose * shape.segment.a);
                let t2 = to_macroquad(pose * shape.segment.b);
                draw_sphere_wires(t1, shape.radius, None, color);
                draw_sphere_wires(t2, shape.radius, None, color);
            }
            _ => {}
        }
    }
}

fn build_world() -> World {
    const MARGIN: Real = 0.1;

    let mut world = World::new();
    world.statics_mut().quick_add(
        new_static(new_box(Vector::new(20.0, 1.0, 20.0)), pose!(0, -0.5, 0)),
        MARGIN,
    );
    world.dynamics_mut().quick_add(
        new_dynamic(new_capsule(1.0, 2.0), pose!([0, 10, 0], [Z, 10]), -1.0, 1.0),
        MARGIN,
    );
    world.dynamics_mut().quick_add(
        new_dynamic(
            new_capsule(1.0, 2.0),
            pose!([0.5, 15, 0.5], [X, 10]),
            -1.5,
            1.0,
        ),
        MARGIN,
    );
    world.statics_mut().quick_add(
        new_static(new_box(Vector::new(5.0, 5.0, 5.0)), pose!(4, 2.5, 5)),
        MARGIN,
    );
    world.start();

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

#[inline]
fn new_static(shape: SharedShape, pose: Pose) -> RefCell<StaticObject> {
    RefCell::new(StaticObject::new(shape, pose, MASK_ALL, false))
}

#[inline]
fn new_dynamic(
    shape: SharedShape,
    pose: Pose,
    fall_speed: Real,
    weight: Real,
) -> RefCell<DynamicObject> {
    let mut d = DynamicObject::new(shape, pose, MASK_ALL, false, weight);
    d.velocity.y = fall_speed;
    RefCell::new(d)
}

#[inline]
fn new_box(size: Vector) -> SharedShape {
    SharedShape::cuboid(size[0] * 0.5, size[1] * 0.5, size[2] * 0.5)
}

#[inline]
fn new_ball(diameter: Real) -> SharedShape {
    SharedShape::ball(diameter * 0.5)
}

#[inline]
fn new_capsule(diameter: Real, height: Real) -> SharedShape {
    let radius = diameter * 0.5;
    SharedShape::capsule_y(height * 0.5 - radius, radius)
}

#[inline]
fn new_cylinder(diameter: Real, height: Real) -> SharedShape {
    SharedShape::cylinder(height * 0.5, diameter * 0.5)
}

/// Converts a parry `Vector` to a macroquad `Vec3`.
#[inline]
fn to_macroquad(v: Vector) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}
