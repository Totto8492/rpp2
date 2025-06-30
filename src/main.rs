#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Executor;
use embassy_rp::clocks::{self, ClockConfig, CoreVoltage};
use embassy_rp::config::Config;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::pac::dma::vals::TreqSel;
use embassy_rp::peripherals::{DMA_CH1, SPI0};
use embassy_rp::spi::{self, Phase, Polarity, Spi};
use embassy_rp::{Peri, dma};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel;
use embassy_time::{Instant, Timer};
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

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut config = Config::new(ClockConfig::system_freq(300_000_000).unwrap());
    config.clocks.core_voltage = CoreVoltage::V1_30;

    let p = embassy_rp::init(config);

    let sys_freq = clocks::clk_sys_freq();
    info!("System clock frequency: {} MHz", sys_freq / 1_000_000);

    let core_voltage = clocks::core_voltage().unwrap();
    info!("Core voltage: {}", core_voltage);

    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| unwrap!(spawner.spawn(core1_task())));
        },
    );

    let _ = Output::new(p.PIN_23, Level::High);

    let reset = Output::new(p.PIN_1, Level::Low);
    let dc = Output::new(p.PIN_2, Level::Low);
    let mut spi_config = spi::Config::default();
    spi_config.polarity = Polarity::IdleHigh;
    spi_config.phase = Phase::CaptureOnSecondTransition;
    spi_config.frequency = 75_000_000;
    let spi = Spi::new_txonly(p.SPI0, p.PIN_6, p.PIN_3, p.DMA_CH0, spi_config);

    static BUF: StaticCell<[Framebuffer320x120; 2]> = StaticCell::new();
    let buf = BUF.init([Framebuffer320x120::new(), Framebuffer320x120::new()]);

    static CHANNEL: StaticCell<zerocopy_channel::Channel<'_, NoopRawMutex, Framebuffer320x120>> =
        StaticCell::new();
    let channel = CHANNEL.init(zerocopy_channel::Channel::new(buf));

    let (sender, receiver) = channel.split();

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.must_spawn(draw_task(sender, p.DMA_CH1));
        spawner.must_spawn(display_task(receiver, reset, dc, spi));
    });
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
    mut receiver: zerocopy_channel::Receiver<'static, NoopRawMutex, Framebuffer320x120>,
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
        let framebuffer = receiver.receive().await;
        if let Err(e) = spi.write(framebuffer.data()).await {
            error!("Failed to write framebuffer data: {:?}", e);
        }
        receiver.receive_done();
    }
}

#[embassy_executor::task]
async fn draw_task(
    mut sender: zerocopy_channel::Sender<'static, NoopRawMutex, Framebuffer320x120>,
    mut dma_ch: Peri<'static, DMA_CH1>,
) {
    let start_time = Instant::now();
    let mut half_frames = 0;
    let mut last_time = 0.0;
    let mut state = RenderState::<256>::default();
    loop {
        if half_frames % 2 == 0 {
            let elapsed = (start_time.elapsed().as_micros() as f64 / 1_000_000.0) as f32;
            let delta = elapsed - last_time;
            process(elapsed, delta, &mut state);
            last_time = elapsed;
        }

        let offset = match half_frames % 2 {
            0 => Point::zero(),
            1 => Point::new(0, 120),
            _ => crate::unreachable!(),
        };
        half_frames += 1;

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

        let mut framebuffer_translated = framebuffer.translated(-offset);
        render(&mut framebuffer_translated, &state).unwrap();
        sender.send_done();
    }
}

#[embassy_executor::task]
async fn core1_task() {
    loop {
        cortex_m::asm::wfe();
    }
}
