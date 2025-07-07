#![no_std]
#![no_main]

use embassy_executor::Executor;
use embassy_rp::{
    Peri,
    clocks::{self, ClockConfig, CoreVoltage},
    config::Config,
    dma,
    gpio::{Level, Output},
    multicore::{Stack, spawn_core1},
    pac::dma::vals::TreqSel,
    peripherals::{DMA_CH1, DMA_CH2, SPI0},
    spi::{self, Phase, Polarity, Spi},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, zerocopy_channel};
use embassy_time::{Duration, Instant, Timer};
use embedded_graphics::prelude::*;
use embedded_graphics::{
    framebuffer::{Framebuffer, buffer_size},
    pixelcolor::{
        Bgr565,
        raw::{BigEndian, RawU16},
    },
};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use static_cell::StaticCell;

mod scene;
use scene::RenderState;

type FramebufferType =
    Framebuffer<Bgr565, RawU16, BigEndian, 320, 120, { buffer_size::<Bgr565>(320, 120) }>;

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

#[cortex_m_rt::entry]
fn main() -> ! {
    rtt_init_print!();

    let mut config = Config::new(ClockConfig::system_freq(300_000_000).unwrap());
    config.clocks.core_voltage = CoreVoltage::V1_30;

    let p = embassy_rp::init(config);

    let sys_freq = clocks::clk_sys_freq();
    rprintln!("System clock frequency: {} MHz", sys_freq / 1_000_000);

    let core_voltage = clocks::core_voltage().unwrap();
    rprintln!("Core voltage: {:?}", core_voltage);

    let _ = Output::new(p.PIN_23, Level::High);

    let reset = Output::new(p.PIN_1, Level::Low);
    let dc = Output::new(p.PIN_2, Level::Low);
    let mut spi_config = spi::Config::default();
    spi_config.polarity = Polarity::IdleHigh;
    spi_config.phase = Phase::CaptureOnSecondTransition;
    spi_config.frequency = 75_000_000;
    let spi = Spi::new_txonly(p.SPI0, p.PIN_6, p.PIN_3, p.DMA_CH0, spi_config);

    static BUFFER_A: StaticCell<[FramebufferType; 2]> = StaticCell::new();
    let buffer_a = BUFFER_A.init([FramebufferType::new(), FramebufferType::new()]);
    static FRAMEBUFFER_A: StaticCell<
        zerocopy_channel::Channel<'_, CriticalSectionRawMutex, FramebufferType>,
    > = StaticCell::new();
    let framebuffer_a = FRAMEBUFFER_A.init(zerocopy_channel::Channel::new(buffer_a));
    let (framebuffer_a_sender, framebuffer_a_receiver) = framebuffer_a.split();

    static BUFFER_B: StaticCell<[FramebufferType; 2]> = StaticCell::new();
    let buffer_b = BUFFER_B.init([FramebufferType::new(), FramebufferType::new()]);
    static FRAMEBUFFER_B: StaticCell<
        zerocopy_channel::Channel<'_, CriticalSectionRawMutex, FramebufferType>,
    > = StaticCell::new();
    let framebuffer_b = FRAMEBUFFER_B.init(zerocopy_channel::Channel::new(buffer_b));
    let (framebuffer_b_sender, framebuffer_b_receiver) = framebuffer_b.split();

    static STATE_A: StaticCell<[RenderState<256>; 2]> = StaticCell::new();
    let state_a = STATE_A.init([RenderState::<256>::default(), RenderState::<256>::default()]);
    static STATE_A_CHANNEL: StaticCell<
        zerocopy_channel::Channel<'_, CriticalSectionRawMutex, RenderState<256>>,
    > = StaticCell::new();
    let state_a_channel = STATE_A_CHANNEL.init(zerocopy_channel::Channel::new(state_a));
    let (state_a_sender, state_a_receiver) = state_a_channel.split();

    static STATE_B: StaticCell<[RenderState<256>; 2]> = StaticCell::new();
    let state_b = STATE_B.init([RenderState::<256>::default(), RenderState::<256>::default()]);
    static STATE_B_CHANNEL: StaticCell<
        zerocopy_channel::Channel<'_, CriticalSectionRawMutex, RenderState<256>>,
    > = StaticCell::new();
    let state_b_channel = STATE_B_CHANNEL.init(zerocopy_channel::Channel::new(state_b));
    let (state_b_sender, state_b_receiver) = state_b_channel.split();

    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                spawner.must_spawn(core1_task());
                spawner.must_spawn(core1_render_task(
                    state_b_receiver,
                    framebuffer_b_sender,
                    p.DMA_CH2,
                ));
            });
        },
    );

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.must_spawn(core0_task());
        spawner.must_spawn(display_task(
            framebuffer_a_receiver,
            framebuffer_b_receiver,
            reset,
            dc,
            spi,
        ));
        spawner.must_spawn(process_task(state_a_sender, state_b_sender));
        spawner.must_spawn(core0_render_task(
            state_a_receiver,
            framebuffer_a_sender,
            p.DMA_CH1,
        ));
    });
}

#[embassy_executor::task]
async fn core0_task() {
    loop {
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn core1_task() {
    loop {
        Timer::after_secs(1).await;
    }
}

async fn init_display(
    reset: &mut Output<'static>,
    dc: &mut Output<'static>,
    spi: &mut Spi<'static, SPI0, spi::Async>,
) -> Result<(), embassy_rp::spi::Error> {
    reset.set_low();
    Timer::after_millis(1).await;
    reset.set_high();
    Timer::after_millis(120).await;

    // Display initialization commands
    // (command, data, delay_ms)
    let commands: &[(u8, Option<&[u8]>, u64)] = &[
        (0x01, None, 5),                                                // Software Reset
        (0xCB, Some(&[0x39, 0x2C, 0x00, 0b0011_0101, 0b0000_0000]), 0), // Power Control A
        (0x36, Some(&[0b0011_0100]), 0),                                // Memory Access Control
        (0x3A, Some(&[0b0101_0101]), 0),                                // Pixel Format Set
        (0x2A, Some(&[0x00, 0x00, 0x01, 0x3F]), 0),                     // Column Address Set
        (0x2B, Some(&[0x00, 0x00, 0x00, 0xEF]), 0),                     // Page Address Set
        (0x11, None, 120),                                              // Sleep Out
        (0x29, None, 0),                                                // Display ON
    ];

    for (cmd, data, delay) in commands {
        dc.set_low();
        spi.write(&[*cmd]).await?;
        if let Some(d) = data {
            dc.set_high();
            spi.write(d).await?;
        }
        if *delay > 0 {
            Timer::after_millis(*delay).await;
        }
    }

    Ok(())
}

#[embassy_executor::task]
async fn display_task(
    mut framebuffer_a: zerocopy_channel::Receiver<
        'static,
        CriticalSectionRawMutex,
        FramebufferType,
    >,
    mut framebuffer_b: zerocopy_channel::Receiver<
        'static,
        CriticalSectionRawMutex,
        FramebufferType,
    >,
    mut reset: Output<'static>,
    mut dc: Output<'static>,
    mut spi: Spi<'static, SPI0, spi::Async>,
) {
    if let Err(e) = init_display(&mut reset, &mut dc, &mut spi).await {
        rprintln!("Display initialization error: {:?}", e);
        return;
    }

    dc.set_low();
    if let Err(e) = spi.write(&[0x2c]).await {
        rprintln!("Failed to write command 0x2c: {:?}", e);
    }
    dc.set_high();

    loop {
        let buffer = framebuffer_a.receive().await;
        spi.write(buffer.data()).await.unwrap();
        framebuffer_a.receive_done();

        let buffer = framebuffer_b.receive().await;
        spi.write(buffer.data()).await.unwrap();
        framebuffer_b.receive_done();
    }
}

#[embassy_executor::task]
async fn process_task(
    mut state_a_sender: zerocopy_channel::Sender<
        'static,
        CriticalSectionRawMutex,
        RenderState<256>,
    >,
    mut state_b_sender: zerocopy_channel::Sender<
        'static,
        CriticalSectionRawMutex,
        RenderState<256>,
    >,
) {
    let start_time = Instant::now();
    let mut next_watch = start_time + Duration::from_secs(1);
    let mut frame_count = 0;
    let mut fps = 0;
    loop {
        let elapsed = (start_time.elapsed().as_micros() as f64 / 1_000_000.0) as f32;

        let state_a = state_a_sender.send().await;
        let state_b = state_b_sender.send().await;
        scene::process(elapsed, state_a);

        frame_count += 1;
        if next_watch < Instant::now() {
            next_watch += Duration::from_secs(1);
            fps = frame_count;
            frame_count = 0;
        }
        state_a.fps = fps;

        state_b.clone_from(state_a);
        state_b_sender.send_done();
        state_a_sender.send_done();
    }
}

#[embassy_executor::task]
async fn core0_render_task(
    mut state_receiver: zerocopy_channel::Receiver<
        'static,
        CriticalSectionRawMutex,
        RenderState<256>,
    >,
    mut framebuffer: zerocopy_channel::Sender<'static, CriticalSectionRawMutex, FramebufferType>,
    mut dma_ch: Peri<'static, DMA_CH1>,
) {
    loop {
        let buffer = framebuffer.send().await;

        static ZERO: u16 = 0;
        let ptr = bytemuck::cast_slice_mut(buffer.data_mut());
        unsafe {
            dma::read(
                dma_ch.reborrow(),
                core::ptr::addr_of!(ZERO),
                core::ptr::addr_of_mut!(*ptr),
                TreqSel::PERMANENT,
            )
            .await;
        }

        let state = state_receiver.receive().await;
        scene::render(buffer, state).unwrap();
        state_receiver.receive_done();
        framebuffer.send_done();
    }
}

#[embassy_executor::task]
async fn core1_render_task(
    mut state_receiver: zerocopy_channel::Receiver<
        'static,
        CriticalSectionRawMutex,
        RenderState<256>,
    >,
    mut framebuffer: zerocopy_channel::Sender<'static, CriticalSectionRawMutex, FramebufferType>,
    mut dma_ch: Peri<'static, DMA_CH2>,
) {
    loop {
        let buffer = framebuffer.send().await;

        static ZERO: u16 = 0;
        let ptr = bytemuck::cast_slice_mut(buffer.data_mut());
        unsafe {
            dma::read(
                dma_ch.reborrow(),
                core::ptr::addr_of!(ZERO),
                core::ptr::addr_of_mut!(*ptr),
                TreqSel::PERMANENT,
            )
            .await;
        }

        let state = state_receiver.receive().await;
        let buffer = &mut buffer.translated(Point::new(0, -120));
        scene::render(buffer, state).unwrap();
        state_receiver.receive_done();
        framebuffer.send_done();
    }
}
