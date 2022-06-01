#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

use panic_semihosting as _;
use rtic::app;

#[app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [XIP_IRQ])]
mod app {
    use embedded_hal::digital::v2::OutputPin;
    use embedded_hal::digital::v2::ToggleableOutputPin;

    use rp2040_monotonic::*;
    use rp_pico::hal;

    type LedPin = hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Monotonic = Rp2040Monotonic;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: LedPin,
    }

    #[init]
    fn init(context: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = context.device.RESETS;
        let sio = hal::Sio::new(context.device.SIO);
        let pins = rp_pico::Pins::new(
            context.device.IO_BANK0,
            context.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let mut led = pins.led.into_push_pull_output();
        led.set_low().unwrap();

        toggle_led::spawn().ok();

        (
            Shared {},
            Local { led },
            init::Monotonics(Rp2040Monotonic::new(context.device.TIMER)),
        )
    }

    #[task(priority = 1, local = [led])]
    fn toggle_led(cx: toggle_led::Context) {
        toggle_led::spawn_after(1000.millis()).ok();
        cx.local.led.toggle().ok();
    }
}
