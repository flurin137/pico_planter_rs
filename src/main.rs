#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

use panic_semihosting as _;
use rtic::app;

#[app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [XIP_IRQ])]
mod app {

    use embedded_hal::adc::OneShot;
    use embedded_hal::digital::v2::OutputPin;

    use rp2040_hal::gpio::bank0::Gpio26;
    use rp2040_monotonic::*;
    use rp_pico::hal;

    type LedPin = hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>;
    type SwitchPin = hal::gpio::Pin<hal::gpio::bank0::Gpio1, hal::gpio::PullUpInput>;

    type AnalogPin = hal::gpio::Pin<Gpio26, hal::gpio::Input<hal::gpio::Floating>>;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Monotonic = Rp2040Monotonic;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: LedPin,
        switch: SwitchPin,
        adc: hal::Adc,
        sensor: AnalogPin,
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

        let switch = pins.gpio1.into_mode();
        switch.set_interrupt_enabled(hal::gpio::Interrupt::EdgeLow, true);

        let adc = hal::Adc::new(context.device.ADC, &mut resets);
        let sensor = pins.gpio26.into_floating_input();

        led.set_low().unwrap();

        (
            Shared {},
            Local {
                led,
                switch,
                adc,
                sensor,
            },
            init::Monotonics(Rp2040Monotonic::new(context.device.TIMER)),
        )
    }

    #[task(priority = 1, local = [led, switch, adc, sensor ])]
    fn toggle_led(context: toggle_led::Context) {
        let led = context.local.led;
        let adc = context.local.adc;
        let sensor = context.local.sensor;

        if let Ok::<u16, _>(value) = adc.read(sensor) {
            if value > 200 {
                led.set_high().ok();
            } else {
                led.set_low().ok();
            }
        };

        toggle_led::spawn_after(1000.millis()).ok();
    }
}
