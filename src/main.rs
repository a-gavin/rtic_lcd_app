#![no_std]
#![no_main]

#[rtic::app(
    device = rp_pico::hal::pac,
    dispatchers = [TIMER_IRQ_1]
)]
mod app {
    use rp2040_monotonic::{fugit::Duration, fugit::RateExtU32, Rp2040Monotonic};
    use rp_pico::hal::{
        clocks,
        clocks::ClockSource, // needed for .get_freq()
        gpio,
        gpio::pin::bank0::{Gpio13, Gpio2, Gpio25, Gpio3},
        gpio::pin::{Pin, PushPullOutput},
        pac,
        sio::Sio,
        spi::{self, Enabled, Spi},
        watchdog::Watchdog,
        Clock, // needed for .freq()
        I2C,
    };
    use rp_pico::XOSC_CRYSTAL_FREQ;

    use core::mem::MaybeUninit;
    use cortex_m::delay::Delay;
    use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};

    use lcd_2004_i2c::{Lcd, LcdUninit, RowMode};
    use ufmt::uwrite;

    use embedded_nal::TcpClientStack;
    use embedded_nal::{IpAddr, Ipv4Addr, SocketAddr};
    use w5500::{bus::FourWire, tcp::TcpSocket, Device, MacAddress, UninitializedDevice};

    use defmt::*;
    use defmt_rtt as _;
    use panic_probe as _;

    const MONO_NUM: u32 = 1;
    const MONO_DENOM: u32 = 1000000;
    const ONE_SEC_TICKS: u64 = 1000000;
    type RpiDuration = Duration<u64, MONO_NUM, MONO_DENOM>;

    const LCD_ADDRESS: u8 = 0x27;
    type I2CBus = I2C<
        pac::I2C1,
        (
            Pin<Gpio2, gpio::FunctionI2C>,
            Pin<Gpio3, gpio::FunctionI2C>,
        ),
    >;

    type EthDevice =
        Device<FourWire<Spi<Enabled, pac::SPI1, 8>, Pin<Gpio13, PushPullOutput>>, w5500::Manual>;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Rp2040Mono = Rp2040Monotonic;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: gpio::Pin<Gpio25, PushPullOutput>,
        lcd: Lcd<'static, I2CBus, Delay>,
        eth: EthDevice,
        socket: TcpSocket,
        sock_addr: SocketAddr,
    }

    #[init(local=[
        i2c: MaybeUninit<I2CBus> = MaybeUninit::uninit()
    ])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        debug!("Program init");
        let mut periphs = ctx.device;
        let mut watchdog = Watchdog::new(periphs.WATCHDOG);

        // The default is to generate a 125 MHz system clock
        let clocks = clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            periphs.XOSC,
            periphs.CLOCKS,
            periphs.PLL_SYS,
            periphs.PLL_USB,
            &mut periphs.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(periphs.SIO);
        let gpio = rp_pico::Pins::new(
            periphs.IO_BANK0,
            periphs.PADS_BANK0,
            sio.gpio_bank0,
            &mut periphs.RESETS,
        );

        // Init LED pin
        let mut led = gpio.led.into_push_pull_output();
        led.set_low().unwrap();

        // Init I2C pins for LCD
        info!("LCD init");
        let sda_pin = gpio.gpio2.into_mode::<gpio::FunctionI2C>();
        let scl_pin = gpio.gpio3.into_mode::<gpio::FunctionI2C>();

        let i2c: &'static mut _ = ctx.local.i2c.write(I2C::i2c1(
            periphs.I2C1,
            sda_pin,
            scl_pin,
            100.kHz(),
            &mut periphs.RESETS,
            &clocks.system_clock,
        ));

        // Init LCD
        let delay = Delay::new(ctx.core.SYST, clocks.system_clock.get_freq().to_Hz());
        let mut lcd = LcdUninit::new(i2c, LCD_ADDRESS, delay)
            .set_num_rows(RowMode::Four)
            .init()
            .unwrap();
        _ = lcd.set_cursor_blink(false);

        // Init SPI (SPI bank 1, SPI init takes care of rest)
        info!("Network init");
        let _rx = gpio.gpio12.into_mode::<gpio::FunctionSpi>();
        let _sck = gpio.gpio14.into_mode::<gpio::FunctionSpi>();
        let _tx = gpio.gpio15.into_mode::<gpio::FunctionSpi>();

        let mut cs = gpio.gpio13.into_push_pull_output();
        cs.set_high().unwrap();

        let spi = spi::Spi::<_, _, 8>::new(periphs.SPI1);
        let spi = spi.init(
            &mut periphs.RESETS,
            clocks.peripheral_clock.freq(),
            1_000_000u32.Hz(),
            &embedded_hal::spi::MODE_0,
        );

        // Init w5500 itself and respective socket
        let mut eth = UninitializedDevice::new(FourWire::new(spi, cs))
            .initialize_manual(
                MacAddress::new(0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE),
                Ipv4Addr::new(169, 254, 112, 188),
                w5500::Mode::default(),
            )
            .unwrap();

        let mut socket = eth.socket().unwrap();
        let sock_addr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(169, 254, 112, 189)), 14000);
        eth.connect(&mut socket, sock_addr).unwrap();

        let mono = Rp2040Mono::new(periphs.TIMER);

        debug!("Spawning heartbeat task");
        heartbeat::spawn().unwrap();

        debug!("Spawning networking task");
        network::spawn().unwrap();

        debug!("Spawning display task");
        display::spawn(0).unwrap();

        let shared = Shared {};
        let local = Local {
            led,
            lcd,
            eth,
            socket,
            sock_addr,
        };

        (shared, local, init::Monotonics(mono))
    }

    #[task(local = [lcd])]
    fn display(ctx: display::Context, iter: u32) {
        debug!("Display {}", iter);
        let lcd = ctx.local.lcd;

        lcd.clear().unwrap();
        lcd.return_home().unwrap();
        _ = uwrite!(lcd, "Display {}", iter);

        // Respawn task in a fifth of a second
        let one_fifth_second = Duration::<u64, MONO_NUM, MONO_DENOM>::from_ticks(ONE_SEC_TICKS / 5);
        display::spawn_after(one_fifth_second, iter + 1).unwrap();
    }

    #[task(local = [eth, socket, sock_addr])]
    fn network(ctx: network::Context) {
        debug!("Network");
        let eth = ctx.local.eth;
        let mut socket = ctx.local.socket;

        if eth.is_connected(&socket).unwrap() {
            debug!("Sending");
            let send_buf = [0x00, 0xFF, 0x00, 0xFF];
            eth.send(&mut socket, &send_buf).unwrap();
            debug!("Sent");
        }

        // Respawn task in a half second
        let three_second = RpiDuration::from_ticks(3 * ONE_SEC_TICKS);
        network::spawn_after(three_second).unwrap();
    }

    #[task(local = [led])]
    fn heartbeat(ctx: heartbeat::Context) {
        debug!("Heartbeat");
        _ = ctx.local.led.toggle();

        // Respawn task in a second
        let one_second = RpiDuration::from_ticks(ONE_SEC_TICKS);
        heartbeat::spawn_after(one_second).unwrap();
    }
}
