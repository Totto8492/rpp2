#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Executor;
use embassy_rp::clocks::{self, ClockConfig, CoreVoltage};
use embassy_rp::config::Config;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::pac::dma::vals::TreqSel;
use embassy_rp::peripherals::{DMA_CH1, DMA_CH2, SPI0};
use embassy_rp::spi::{self, Phase, Polarity, Spi};
use embassy_rp::{Peri, dma};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::rwlock::RwLock;
use embassy_sync::zerocopy_channel;
use embassy_time::{Duration, Instant, Timer};
use embedded_graphics::framebuffer::{Framebuffer, buffer_size};
use embedded_graphics::pixelcolor::Bgr565;
use embedded_graphics::pixelcolor::raw::{BigEndian, RawU16};
use embedded_graphics::prelude::{DrawTargetExt, Point};
use static_cell::StaticCell;

mod scene;
use scene::{RenderState, process, render};

use {defmt_rtt as _, panic_probe as _};

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

type Framebuffer320x120 =
    Framebuffer<Bgr565, RawU16, BigEndian, 320, 120, { buffer_size::<Bgr565>(320, 120) }>;

type RenderStateRwLock = RwLock<CriticalSectionRawMutex, RenderState<256>>;

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut config = Config::new(ClockConfig::system_freq(300_000_000).unwrap());
    config.clocks.core_voltage = CoreVoltage::V1_30;

    let p = embassy_rp::init(config);

    let sys_freq = clocks::clk_sys_freq();
    info!("System clock frequency: {} MHz", sys_freq / 1_000_000);

    let core_voltage = clocks::core_voltage().unwrap();
    info!("Core voltage: {}", core_voltage);

    let _ = Output::new(p.PIN_23, Level::High);

    let reset = Output::new(p.PIN_1, Level::Low);
    let dc = Output::new(p.PIN_2, Level::Low);
    let mut spi_config = spi::Config::default();
    spi_config.polarity = Polarity::IdleHigh;
    spi_config.phase = Phase::CaptureOnSecondTransition;
    spi_config.frequency = 75_000_000;
    let spi = Spi::new_txonly(p.SPI0, p.PIN_6, p.PIN_3, p.DMA_CH0, spi_config);

    static BUF_A: StaticCell<[Framebuffer320x120; 2]> = StaticCell::new();
    let buf_a = BUF_A.init([Framebuffer320x120::new(), Framebuffer320x120::new()]);
    static CHANNEL_A: StaticCell<
        zerocopy_channel::Channel<'_, CriticalSectionRawMutex, Framebuffer320x120>,
    > = StaticCell::new();
    let channel_a = CHANNEL_A.init(zerocopy_channel::Channel::new(buf_a));
    let (sender_a, receiver_a) = channel_a.split();

    static BUF_B: StaticCell<[Framebuffer320x120; 2]> = StaticCell::new();
    let buf_b = BUF_B.init([Framebuffer320x120::new(), Framebuffer320x120::new()]);
    static CHANNEL_B: StaticCell<
        zerocopy_channel::Channel<'_, CriticalSectionRawMutex, Framebuffer320x120>,
    > = StaticCell::new();
    let channel_b = CHANNEL_B.init(zerocopy_channel::Channel::new(buf_b));
    let (sender_b, receiver_b) = channel_b.split();

    static DO_RENDER_A: Channel<CriticalSectionRawMutex, (), 2> = Channel::new();
    static DO_RENDER_B: Channel<CriticalSectionRawMutex, (), 2> = Channel::new();

    static STATE: StaticCell<RenderStateRwLock> = StaticCell::new();
    let state = STATE.init(RwLock::new(RenderState::default()));

    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                spawner.must_spawn(render_task_b(&DO_RENDER_B, sender_b, p.DMA_CH2, state));
            });
        },
    );

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.must_spawn(process_task(&DO_RENDER_A, &DO_RENDER_B, state));
        spawner.must_spawn(render_task_a(&DO_RENDER_A, sender_a, p.DMA_CH1, state));
        spawner.must_spawn(display_task(receiver_a, receiver_b, reset, dc, spi));
    });
}

#[embassy_executor::task]
async fn process_task(
    channel_a: &'static Channel<CriticalSectionRawMutex, (), 2>,
    channel_b: &'static Channel<CriticalSectionRawMutex, (), 2>,
    state: &'static RenderStateRwLock,
) {
    let start_time = Instant::now();
    let mut next_watch = start_time + Duration::from_secs(1);
    let mut frame_count = 0;
    loop {
        let elapsed = (start_time.elapsed().as_micros() as f64 / 1_000_000.0) as f32;

        {
            let mut rw = state.write().await;
            process(elapsed, &mut rw);
            frame_count += 1;
            if next_watch < Instant::now() {
                next_watch += Duration::from_secs(1);
                rw.fps = frame_count;
                frame_count = 0;
            }
        }

        channel_a.send(()).await;
        channel_b.send(()).await;
    }
}

#[embassy_executor::task]
async fn render_task_a(
    channel: &'static Channel<CriticalSectionRawMutex, (), 2>,
    mut sender: zerocopy_channel::Sender<'static, CriticalSectionRawMutex, Framebuffer320x120>,
    mut dma_ch: Peri<'static, DMA_CH1>,
    state: &'static RenderStateRwLock,
) {
    loop {
        channel.receive().await;
        let framebuffer = sender.send().await;

        static ZERO: u8 = 0;
        let ptr = framebuffer.data_mut();
        unsafe {
            dma::read(
                dma_ch.reborrow(),
                core::ptr::addr_of!(ZERO),
                core::ptr::addr_of_mut!(*ptr),
                TreqSel::PERMANENT,
            )
            .await;
        }

        {
            let rw = state.read().await;
            render(framebuffer, &rw).unwrap();
        }

        sender.send_done();
    }
}

#[embassy_executor::task]
async fn render_task_b(
    channel: &'static Channel<CriticalSectionRawMutex, (), 2>,
    mut sender: zerocopy_channel::Sender<'static, CriticalSectionRawMutex, Framebuffer320x120>,
    mut dma_ch: Peri<'static, DMA_CH2>,
    state: &'static RenderStateRwLock,
) {
    loop {
        channel.receive().await;
        let framebuffer = sender.send().await;

        static ZERO: u8 = 0;
        let ptr = framebuffer.data_mut();
        unsafe {
            dma::read(
                dma_ch.reborrow(),
                core::ptr::addr_of!(ZERO),
                core::ptr::addr_of_mut!(*ptr),
                TreqSel::PERMANENT,
            )
            .await;
        }

        let mut framebuffer = framebuffer.translated(Point::new(0, -120));

        {
            let rw = state.read().await;
            render(&mut framebuffer, &rw).unwrap();
        }

        sender.send_done();
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
    mut receiver_a: zerocopy_channel::Receiver<
        'static,
        CriticalSectionRawMutex,
        Framebuffer320x120,
    >,
    mut receiver_b: zerocopy_channel::Receiver<
        'static,
        CriticalSectionRawMutex,
        Framebuffer320x120,
    >,
    mut reset: Output<'static>,
    mut dc: Output<'static>,
    mut spi: Spi<'static, SPI0, spi::Async>,
) {
    if let Err(e) = init_display(&mut reset, &mut dc, &mut spi).await {
        error!("Display initialization error: {:?}", e);
        return;
    }

    dc.set_low();
    if let Err(e) = spi.write(&[0x2c]).await {
        error!("Failed to write command 0x2c: {:?}", e);
    }
    dc.set_high();

    loop {
        let framebuffer = receiver_a.receive().await;
        if let Err(e) = spi.write(framebuffer.data()).await {
            error!("Failed to write framebuffer data: {:?}", e);
        }
        receiver_a.receive_done();

        let framebuffer = receiver_b.receive().await;
        if let Err(e) = spi.write(framebuffer.data()).await {
            error!("Failed to write framebuffer data: {:?}", e);
        }
        receiver_b.receive_done();
    }
}
