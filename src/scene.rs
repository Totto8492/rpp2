use core::f32::consts::PI;
use core::fmt::Write;

use embedded_graphics::Drawable;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Bgr565;
use embedded_graphics::prelude::{Point, Primitive, RgbColor, WebColors};
use embedded_graphics::primitives::PrimitiveStyle;
use embedded_graphics::primitives::Triangle;
use embedded_graphics::text::{Baseline, Text};
use glam::{Mat4, Vec3};
use num_traits::cast;

pub(crate) struct Mesh {
    vertex_buffer: &'static [Vec3],
    index_buffer: &'static [(usize, usize, usize, Bgr565)],
}

#[derive(Debug, Clone, Copy)]
struct RenderQueue {
    polygon: (Point, Point, Point),
    color: Bgr565,
}

#[derive(Default, Clone)]
pub(crate) struct RenderState<const N: usize> {
    pub(crate) fps: u32,
    polygon_count: u32,
    culling_count: u32,
    queue: heapless::Vec<RenderQueue, N>,
}

pub(crate) fn process<const N: usize>(elapsed: f32, state: &mut RenderState<N>) {
    state.queue.clear();
    state.polygon_count = 0;
    state.culling_count = 0;

    const PYRAMID: Mesh = Mesh {
        vertex_buffer: &[
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(-0.5, 0.0, -0.5),
            Vec3::new(-0.5, 0.0, 0.5),
            Vec3::new(0.5, 0.0, 0.5),
            Vec3::new(0.5, 0.0, -0.5),
        ],
        index_buffer: &[
            (2, 1, 0, Bgr565::RED),
            (3, 2, 0, Bgr565::GREEN),
            (4, 3, 0, Bgr565::BLUE),
            (1, 4, 0, Bgr565::CSS_GRAY),
            (1, 2, 4, Bgr565::CSS_AQUA),
            (2, 3, 4, Bgr565::CSS_AQUA),
        ],
    };

    const CUBE: Mesh = Mesh {
        vertex_buffer: &[
            Vec3::new(-0.5, -0.5, -0.5), // 0
            Vec3::new(0.5, -0.5, -0.5),  // 1
            Vec3::new(0.5, 0.5, -0.5),   // 2
            Vec3::new(-0.5, 0.5, -0.5),  // 3
            Vec3::new(-0.5, -0.5, 0.5),  // 4
            Vec3::new(0.5, -0.5, 0.5),   // 5
            Vec3::new(0.5, 0.5, 0.5),    // 6
            Vec3::new(-0.5, 0.5, 0.5),   // 7
        ],
        index_buffer: &[
            // Front face (Z-)
            (0, 1, 2, Bgr565::RED),
            (0, 2, 3, Bgr565::RED),
            // Back face (Z+)
            (4, 7, 6, Bgr565::BLUE),
            (4, 6, 5, Bgr565::BLUE),
            // Right face (X+)
            (1, 5, 6, Bgr565::GREEN),
            (1, 6, 2, Bgr565::GREEN),
            // Left face (X-)
            (4, 0, 3, Bgr565::YELLOW),
            (4, 3, 7, Bgr565::YELLOW),
            // Top face (Y+)
            (3, 2, 6, Bgr565::CYAN),
            (3, 6, 7, Bgr565::CYAN),
            // Bottom face (Y-)
            (4, 5, 1, Bgr565::MAGENTA),
            (4, 1, 0, Bgr565::MAGENTA),
        ],
    };

    let view = Mat4::look_at_rh(Vec3::new(0.0, 3.0, 5.0), Vec3::new(0.0, 0.5, 0.0), Vec3::Y);
    let projection = Mat4::perspective_rh(
        PI / (libm::sinf(elapsed * 5.0).abs() * 1.2 + 5.0),
        4.0 / 3.0,
        0.1,
        100.0,
    );

    // PYRAMID
    let model_pyramid = Mat4::from_translation(Vec3::new(-1.0, -0.5, 0.0))
        * Mat4::from_rotation_x(elapsed)
        * Mat4::from_rotation_y(elapsed * 3.0);
    let mvp_pyramid = projection * view * model_pyramid;
    queue_mesh(&PYRAMID, &mvp_pyramid, state);

    // PYRAMID2
    let model_pyramid = Mat4::from_translation(Vec3::new(1.0, 1.0, 0.0))
        * Mat4::from_rotation_x(elapsed)
        * Mat4::from_rotation_y(elapsed * 3.0);
    let mvp_pyramid = projection * view * model_pyramid;
    queue_mesh(&PYRAMID, &mvp_pyramid, state);

    // CUBE
    let model_cube = Mat4::from_translation(Vec3::new(1.0, 0.0, 0.0))
        * Mat4::from_rotation_y(elapsed * 2.0)
        * Mat4::from_rotation_z(elapsed * 0.5);
    let mvp_cube = projection * view * model_cube;
    queue_mesh(&CUBE, &mvp_cube, state);

    // CUBE2
    let model_cube2 = Mat4::from_translation(Vec3::new(0.0, 1.0, 0.0))
        * Mat4::from_rotation_y(elapsed * 2.0)
        * Mat4::from_rotation_z(elapsed * 0.5);
    let mvp_cube2 = projection * view * model_cube2;
    queue_mesh(&CUBE, &mvp_cube2, state);

    // CUBE3
    let model_cube2 = Mat4::from_translation(Vec3::new(-1.0, 1.0, 0.0))
        * Mat4::from_rotation_y(elapsed * 2.0)
        * Mat4::from_rotation_z(elapsed * 0.5);
    let mvp_cube2 = projection * view * model_cube2;
    queue_mesh(&CUBE, &mvp_cube2, state);
}

pub(crate) fn render<D: DrawTarget<Color = Bgr565>, const N: usize>(
    framebuffer: &mut D,
    state: &RenderState<N>,
) -> Result<(), D::Error> {
    for queue in &state.queue {
        let poly = Triangle::new(queue.polygon.0, queue.polygon.1, queue.polygon.2)
            .into_styled(PrimitiveStyle::with_fill(queue.color));
        poly.draw(framebuffer)?;
    }

    let text_style = MonoTextStyle::new(
        &embedded_graphics::mono_font::ascii::FONT_10X20,
        Bgr565::CSS_AQUAMARINE,
    );

    render_text(
        framebuffer,
        "FPS",
        Some(state.fps),
        text_style,
        Point::new(0, 0),
    )?;
    render_text(
        framebuffer,
        "Polys",
        Some(state.polygon_count),
        text_style,
        Point::new(0, 20),
    )?;
    render_text(
        framebuffer,
        "Culls",
        Some(state.culling_count),
        text_style,
        Point::new(0, 40),
    )?;

    Ok(())
}

fn make_polygon(a: Vec3, b: Vec3, c: Vec3) -> Option<(Point, Point, Point)> {
    if (b - a).cross(c - a).z > 0.0 {
        return None;
    }

    if a.z < 0.0 || b.z < 0.0 || c.z < 0.0 {
        return None;
    }

    if a.z > 1.0 || b.z > 1.0 || c.z > 1.0 {
        return None;
    }

    let scale = Vec3::new(160.0, -120.0, 1.0);
    let offset = Vec3::new(160.0, 120.0, 0.0);

    let a_ss = a * scale + offset;
    let b_ss = b * scale + offset;
    let c_ss = c * scale + offset;

    Some((
        Point::new(cast(a_ss.x)?, cast(a_ss.y)?),
        Point::new(cast(b_ss.x)?, cast(b_ss.y)?),
        Point::new(cast(c_ss.x)?, cast(c_ss.y)?),
    ))
}

fn render_text<D: DrawTarget<Color = Bgr565>>(
    framebuffer: &mut D,
    label: &str,
    value: Option<u32>,
    style: MonoTextStyle<'_, Bgr565>,
    pos: Point,
) -> Result<(), D::Error> {
    let mut s = heapless::String::<16>::new();
    if let Some(v) = value {
        write!(&mut s, "{label}: {v}").unwrap();
    } else {
        write!(&mut s, "{label}").unwrap();
    }

    let mut shadow_style = style;
    shadow_style.text_color = Some(Bgr565::CSS_DARK_GRAY);

    let text_shadow = Text::with_baseline(&s, pos + Point::new(1, 1), shadow_style, Baseline::Top);
    text_shadow.draw(framebuffer)?;

    let text_label = Text::with_baseline(&s, pos, style, Baseline::Top);
    text_label.draw(framebuffer)?;

    Ok(())
}

fn queue_mesh<const N: usize>(mesh: &Mesh, mvp: &Mat4, state: &mut RenderState<N>) {
    for v in mesh.index_buffer {
        state.polygon_count += 1;
        let a = mvp.project_point3(mesh.vertex_buffer[v.0]);
        let b = mvp.project_point3(mesh.vertex_buffer[v.1]);
        let c = mvp.project_point3(mesh.vertex_buffer[v.2]);
        let color = v.3;

        let Some(polygon) = make_polygon(a, b, c) else {
            state.culling_count += 1;
            continue;
        };

        state
            .queue
            .push(RenderQueue { polygon, color })
            .expect("Render queue is full");
    }
}
