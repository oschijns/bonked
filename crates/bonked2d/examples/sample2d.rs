use bonked2d::{
    mask::MASK_ALL,
    object::{DynamicObject, Object, StaticObject},
    world::World,
};
use macroquad::{miniquad::window, prelude::*};
use parry2d::{
    math::{Isometry, Point, Real, Vector},
    query::ShapeCastOptions,
    shape::{Ball, Capsule, Cuboid, Shape, SharedShape},
};
use std::{
    cell::{Ref, RefCell},
    sync::Arc,
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
            let pos = to_glam(obj.isometry().translation.vector);
            shape.draw(pos, color);
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

fn build_world() -> World {
    const MARGIN: Real = 0.1;

    let mut world = World::new();
    world
        .statics_mut()
        .quick_add(new_static(new_box([0.0, -0.5], [20.0, 1.0])), MARGIN);
    world.dynamics_mut().quick_add(
        new_dynamic(new_capsule([0.0, 10.0], 1.0, 2.0), -1.0, 1.0),
        MARGIN,
    );
    world.dynamics_mut().quick_add(
        new_dynamic(new_capsule([0.5, 15.0], 1.0, 2.0), -1.5, 1.0),
        MARGIN,
    );
    world
        .statics_mut()
        .quick_add(new_static(new_box([5.0, 2.5], [5.0, 5.0])), MARGIN);
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

fn new_static(coll: (SharedShape, Isometry<Real>)) -> RefCell<StaticObject> {
    RefCell::new(StaticObject::new(coll.0, coll.1, MASK_ALL, false))
}

fn new_dynamic(
    coll: (SharedShape, Isometry<Real>),
    fall_speed: Real,
    weight: Real,
) -> RefCell<DynamicObject> {
    let mut d = DynamicObject::new(coll.0, coll.1, MASK_ALL, false, weight);
    d.velocity.y = fall_speed;
    RefCell::new(d)
}

fn new_box(pos: V2, size: V2) -> (SharedShape, Isometry<Real>) {
    let shape = SharedShape::cuboid(size[0] * 0.5, size[1] * 0.5);
    let pos = Isometry::new(to_nalgebra(pos), 0.0);
    (shape, pos)
}

fn new_ball(pos: V2, diameter: f32) -> (SharedShape, Isometry<Real>) {
    let shape = SharedShape::ball(diameter * 0.5);
    let pos = Isometry::new(to_nalgebra(pos), 0.0);
    (shape, pos)
}

fn new_capsule(pos: V2, diameter: f32, height: f32) -> (SharedShape, Isometry<Real>) {
    let radius = diameter * 0.5;
    let shape = SharedShape::capsule_y(height * 0.5 - radius, radius);
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
