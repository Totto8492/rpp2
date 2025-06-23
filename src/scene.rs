use embedded_graphics::Drawable;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Bgr565;
use embedded_graphics::prelude::{Point, Primitive, RgbColor, Transform, WebColors};
use embedded_graphics::primitives::PrimitiveStyle;
use embedded_graphics::primitives::Triangle;
use embedded_graphics::text::Text;

pub(crate) struct RenderState {
    pub(crate) x: i32,
    pub(crate) y: i32,
    pub(crate) fps: u32,
}

impl RenderState {
    pub(crate) fn new() -> Self {
        Self { x: 0, y: 0, fps: 0 }
    }
}

pub(crate) fn process(elapsed: f32, delta: f32, state: &mut RenderState) {
    let (x, y) = libm::sincosf(elapsed * 4.0);
    state.x = (x * 100.0 + 150.0) as i32;
    state.y = (y * 100.0 + 100.0) as i32;
    state.fps = if delta < 0.001 {
        1000
    } else {
        (1.0 / delta) as u32
    };
}

pub(crate) fn render<D: DrawTarget<Color = Bgr565>>(
    framebuffer: &mut D,
    state: &RenderState,
) -> Result<(), D::Error> {
    framebuffer.clear(Bgr565::CSS_ORANGE)?;

    let triangle = Triangle::new(Point::zero(), Point::new(30, 50), Point::new(-30, 50))
        .into_styled(PrimitiveStyle::with_fill(Bgr565::BLACK));

    triangle
        .translate(Point::new(state.x, state.y))
        .draw(framebuffer)?;

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

    Ok(())
}
