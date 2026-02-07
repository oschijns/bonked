use bonked2d::{
    mask::MASK_ALL,
    object::{DynamicObject, Object, StaticObject},
    world::World,
};
use macroquad::{miniquad::window, prelude::*};
use parry2d::{
    math::{Pose, Real, Rotation, Vector},
    query::ShapeCastOptions,
    shape::{Ball, Capsule, Cuboid, Shape, SharedShape},
};
use std::{
    cell::{Ref, RefCell},
    f32::consts::PI,
};

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
    ($x:literal, $y:literal) => {
        Pose::from_translation(Vector::new($x as Real, $y as Real))
    };
    ([$x:literal, $y:literal], $ang:literal) => {
        Pose::from_parts(
            Vector::new($x as Real, $y as Real),
            Rotation::from_angle($ang as Real * PI / 180.0),
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

    fn draw(&'s self, pose: &Pose, color: Color) {
        const THICKNESS: f32 = 0.05;
        match self {
            Self::Box(shape) => {
                let start = pose.translation;
                let size = shape.half_extents * 2.0;
                draw_rectangle_lines_ex(
                    start.x,
                    start.y,
                    size.x,
                    size.y,
                    THICKNESS,
                    DrawRectangleParams {
                        offset: Vec2::new(0.5, 0.5),
                        rotation: pose.rotation.angle(),
                        color,
                    },
                );
            }
            Self::Ball(shape) => {
                let t = pose.translation;
                draw_circle_lines(t.x, t.y, shape.radius, THICKNESS, color);
            }
            Self::Capsule(shape) => {
                let a = pose * shape.segment.a;
                let b = pose * shape.segment.b;
                draw_circle_lines(a.x, a.y, shape.radius, THICKNESS, color);
                draw_circle_lines(b.x, b.y, shape.radius, THICKNESS, color);
            }
            _ => {}
        }
    }
}

fn build_world() -> World {
    const MARGIN: Real = 0.1;

    let mut world = World::new();
    world.statics_mut().quick_add(
        new_static(new_box(Vector::new(20.0, 1.0)), pose!(0, -0.5)),
        MARGIN,
    );
    world.dynamics_mut().quick_add(
        new_dynamic(new_capsule(1.0, 2.0), pose!([0, 10], 45), -1.0, 1.0),
        MARGIN,
    );
    world.dynamics_mut().quick_add(
        new_dynamic(new_capsule(1.0, 2.0), pose!([0.5, 15], -45), -1.5, 1.0),
        MARGIN,
    );
    world.statics_mut().quick_add(
        new_static(new_box(Vector::new(5.0, 5.0)), pose!([5, 2.5], -10)),
        MARGIN,
    );
    world.start();

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
    SharedShape::cuboid(size[0] * 0.5, size[1] * 0.5)
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
