#![no_std]
#![no_main]

#[rtic::app(
    device = rp_pico::hal::pac,
    dispatchers = [TIMER_IRQ_1]
)]
mod app {
    use rp_pico::XOSC_CRYSTAL_FREQ;
    use rp_pico::hal::{
        clocks,
        clocks::ClockSource,
        gpio,
        gpio::pin::bank0::{Gpio2, Gpio3, Gpio25},
        gpio::pin::PushPullOutput,
        I2C,
        pac,
        sio::Sio,
        watchdog::Watchdog
    };
    use rp2040_monotonic::{
        Rp2040Monotonic,
        fugit::Duration,
    };

    use core::mem::MaybeUninit;
    use cortex_m::delay::Delay;
    use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
    use fugit::RateExtU32; // For .kHz() conversion funcs
    use lcd_i2c::{Backlight, Lcd};

    use defmt::*;
    use defmt_rtt as _;
    use panic_probe as _;

    const MONO_NUM: u32         = 1;
    const MONO_DENOM: u32       = 1000000;
    const ONE_SEC_TICKS: u64    = 1000000;

    const LCD_ADDRESS: u8 = 0x27;

    type I2CBus = I2C<pac::I2C1, (gpio::Pin<Gpio2, gpio::FunctionI2C>, gpio::Pin<Gpio3, gpio::FunctionI2C>)>;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Rp2040Mono = Rp2040Monotonic;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: gpio::Pin<Gpio25, PushPullOutput>,
        display: Lcd<'static, I2CBus, Delay>,
        delay: Delay
    }

    #[init(local=[
        i2c: MaybeUninit<I2CBus> = MaybeUninit::uninit()
    ])]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        debug!("Program init");

        // README: ctx.device is "periphs"
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);

        // Configure the clocks, delay - The default is to generate a 125 MHz system clock
        let clocks = clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let mut delay: Delay = Delay::new(ctx.core.SYST, clocks.system_clock.get_freq().to_Hz());

        // Init LED pin
        let sio = Sio::new(ctx.device.SIO);
        let gpioa = rp_pico::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );
        let mut led = gpioa.led.into_push_pull_output();
        led.set_low().unwrap();

        // Init I2C pins
        let sda_pin = gpioa.gpio2.into_mode::<gpio::FunctionI2C>();
        let scl_pin = gpioa.gpio3.into_mode::<gpio::FunctionI2C>();

        let i2c: &'static mut _ = ctx.local.i2c.write(
            I2C::i2c1(
                ctx.device.I2C1,
                sda_pin,
                scl_pin,
                100.kHz(),
                &mut ctx.device.RESETS,
                &clocks.system_clock
            )
        );
 
        // Init LCD
        let display = Lcd::new(i2c, Backlight::On)
            .address(LCD_ADDRESS)
            .cursor_on(true)
            .rows(4)
            .init(&mut delay)
            .unwrap();

        let mono = Rp2040Mono::new(ctx.device.TIMER);

        debug!("Spawning heartbeat");
        heartbeat::spawn().unwrap();

        debug!("Spawning display");
        display::spawn().unwrap();

        let shared = Shared {};
        let local = Local {
            led,
            display,
            delay,
        };

        (shared, local, init::Monotonics(mono))
    }

    #[task(local = [delay, display])]
    fn display(ctx: display::Context) {
        let mut delay = ctx.local.delay;
        let display = ctx.local.display;

        let one_second = Duration::<u64, MONO_NUM, MONO_DENOM>::from_ticks(ONE_SEC_TICKS);

        debug!("Display 1");
        display.clear().unwrap();
        display.return_home(&mut delay).unwrap();
        display.write_str("Display 1").unwrap();

        delay.delay_ms(1000);

        debug!("Display 2");
        display.clear().unwrap();
        display.return_home(&mut delay).unwrap();
        display.write_str("Display 2").unwrap();

        display::spawn_after(one_second).unwrap();
    }

    #[task(local = [led])]
    fn heartbeat(ctx: heartbeat::Context) {
        let one_second = Duration::<u64, MONO_NUM, MONO_DENOM>::from_ticks(ONE_SEC_TICKS);

        debug!("Heartbeat");
        _ = ctx.local.led.toggle();

        heartbeat::spawn_after(one_second).unwrap();
    }
}