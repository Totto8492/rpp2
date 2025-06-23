use core::f32::consts::PI;

use embedded_graphics::Drawable;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Bgr565;
use embedded_graphics::prelude::{Point, Primitive, RgbColor, WebColors};
use embedded_graphics::primitives::PrimitiveStyle;
use embedded_graphics::primitives::Triangle;
use embedded_graphics::text::Text;
use glam::{Mat4, Vec3};

#[derive(Debug)]
struct RenderQueue {
    polygon: (Point, Point, Point),
    color: Bgr565,
}

pub(crate) struct RenderState {
    fps: u32,
    queue: heapless::Vec<RenderQueue, 16>,
}

impl RenderState {
    pub(crate) fn new() -> Self {
        Self {
            fps: 0,
            queue: heapless::Vec::new(),
        }
    }
}

pub(crate) fn process(elapsed: f32, delta: f32, state: &mut RenderState) {
    state.queue.clear();

    state.fps = if delta < 0.001 {
        1000
    } else {
        (1.0 / delta) as u32
    };

    let index_buffer = [
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(-0.5, 0.0, -0.5),
        Vec3::new(-0.5, 0.0, 0.5),
        Vec3::new(0.5, 0.0, 0.5),
        Vec3::new(0.5, 0.0, -0.5),
    ];

    let vertex_buffer = [
        (0, 1, 2, Bgr565::RED),
        (0, 2, 3, Bgr565::GREEN),
        (0, 3, 4, Bgr565::BLUE),
        (0, 4, 1, Bgr565::CSS_GRAY),
        (4, 2, 1, Bgr565::CSS_AQUA),
        (4, 3, 2, Bgr565::CSS_AQUA),
    ];

    let model = Mat4::from_rotation_x(elapsed) * Mat4::from_rotation_y(elapsed * 3.0);
    let view = Mat4::look_at_rh(
        Vec3::new(0.0, 3.0, -5.0),
        Vec3::new(0.0, 0.5, 0.0),
        Vec3::NEG_Y,
    );
    let projection = Mat4::perspective_rh(
        PI / (libm::sinf(elapsed * 5.0).abs() * 1.2 + 5.0),
        4.0 / 3.0,
        0.1,
        100.0,
    );
    let mvp = projection * view * model;

    let scale = Vec3::new(160.0, 120.0, 1.0);
    let offset = Vec3::new(160.0, 120.0, 0.0);

    for v in &vertex_buffer {
        let a = mvp.project_point3(index_buffer[v.0]);
        let b = mvp.project_point3(index_buffer[v.1]);
        let c = mvp.project_point3(index_buffer[v.2]);
        let a_ss = a * scale + offset;
        let b_ss = b * scale + offset;
        let c_ss = c * scale + offset;

        if !cull(a, b, c) {
            state
                .queue
                .push(RenderQueue {
                    polygon: (
                        Point::new(a_ss.x as i32, a_ss.y as i32),
                        Point::new(b_ss.x as i32, b_ss.y as i32),
                        Point::new(c_ss.x as i32, c_ss.y as i32),
                    ),
                    color: v.3,
                })
                .unwrap();
        }
    }
}

fn cull(a: Vec3, b: Vec3, c: Vec3) -> bool {
    (b - a).cross(c - a).z < 0.0
}

pub(crate) fn render<D: DrawTarget<Color = Bgr565>>(
    framebuffer: &mut D,
    state: &RenderState,
) -> Result<(), D::Error> {
    framebuffer.clear(Bgr565::BLACK)?;

    let mut fps = heapless::String::<16>::new();
    core::fmt::write(&mut fps, format_args!("FPS: {}", state.fps)).unwrap();
    let text_style = MonoTextStyle::new(
        &embedded_graphics::mono_font::ascii::FONT_10X20,
        Bgr565::BLUE,
    );
    let fps_label = Text::with_baseline(
        &fps,
        Point::zero(),
        text_style,
        embedded_graphics::text::Baseline::Top,
    );
    fps_label.draw(framebuffer)?;

    for queue in &state.queue {
        let poly = Triangle::new(queue.polygon.0, queue.polygon.1, queue.polygon.2)
            .into_styled(PrimitiveStyle::with_fill(queue.color));
        poly.draw(framebuffer)?;
    }

    Ok(())
}
