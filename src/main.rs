#![no_std]
#![no_main]

#[rtic::app(
    device = rp_pico::hal::pac,
    dispatchers = [TIMER_IRQ_1]
)]
mod app {
    use cortex_m::delay::Delay;
    use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
    use fugit::RateExtU32; // For .kHz() conversion funcs

    use rp_pico::XOSC_CRYSTAL_FREQ;
    use rp_pico::hal::{
        clocks,
        clocks::ClockSource,
        gpio,
        gpio::pin::bank0::Gpio25,
        gpio::pin::PushPullOutput,
        I2C,
        sio::Sio,
        watchdog::Watchdog
    };
    use rp2040_monotonic::{
        Rp2040Monotonic,
        fugit::Duration,
    };
    use lcd_i2c::{Backlight, Lcd};

    use defmt::*;
    use defmt_rtt as _;
    use panic_probe as _;

    const MONO_NUM: u32         = 1;
    const MONO_DENOM: u32       = 1000000;
    const ONE_SEC_TICKS: u64    = 1000000;

    const LCD_ADDRESS: u8 = 0x27;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Rp2040Mono = Rp2040Monotonic;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: gpio::Pin<Gpio25, PushPullOutput>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        debug!("Program init");

        // README: ctx.device is "periphs"
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);

        // Configure the clocks - The default is to generate a 125 MHz system clock
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
        let pins = rp_pico::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );
        let mut led = pins.led.into_push_pull_output();
        led.set_low().unwrap();

        // Init I2C pins
        let sda_pin = pins.gpio2.into_mode::<gpio::FunctionI2C>();
        let scl_pin = pins.gpio3.into_mode::<gpio::FunctionI2C>();

        let mut i2c = I2C::i2c1(
            ctx.device.I2C1,
            sda_pin,
            scl_pin,
            100.kHz(),
            &mut ctx.device.RESETS,
            &clocks.system_clock,
        );

        // Init LCD, takes ownership of I2C
        let mut lcd = Lcd::new(&mut i2c, Backlight::Off)
            .address(LCD_ADDRESS)
            .cursor_on(true)
            .rows(4)
            .init(&mut delay)
            .unwrap();

        let mono = Rp2040Mono::new(ctx.device.TIMER);


        // DEBUG
        debug!("Starting print, backlight on");
        _ = lcd.backlight(Backlight::On);

        for _ in 1..1000 {
            _ = lcd.return_home(&mut delay);
            _ = delay.delay_ms(10);
            _ = lcd.write_str("we hurrr");
            _ = delay.delay_ms(2000);

            //_ = lcd.clear();
            _ = lcd.return_home(&mut delay);
            _ = delay.delay_ms(10);
            _ = lcd.write_str("we there");
            _ = delay.delay_ms(2000);
        }

        _ = lcd.backlight(Backlight::Off);
        debug!("Ending print, backlight off");
        // DEBUG

        debug!("Spawning heartbeat");
        heartbeat::spawn().unwrap();
        (
            Shared {},
            Local { led },
            init::Monotonics(mono),
        )
    }

    #[task(local = [led])]
    fn heartbeat(ctx: heartbeat::Context) {
        let one_second = Duration::<u64, MONO_NUM, MONO_DENOM>::from_ticks(ONE_SEC_TICKS);

        _ = ctx.local.led.toggle();
        debug!("Heartbeat");
        heartbeat::spawn_after(one_second).unwrap();
    }
}