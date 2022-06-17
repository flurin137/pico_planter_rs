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
    use embedded_hal::digital::v2::ToggleableOutputPin;

    use hal::gpio::*;
    use rp2040_hal::gpio::bank0::*;
    use rp2040_monotonic::*;
    use rp_pico::hal;

    type LedPin = Pin<Gpio25, PushPullOutput>;
    //type MotorPin = Pin<Gpio19, PushPullOutput>;
    type SwitchPin = Pin<Gpio1, PullUpInput>;
    type AnalogPin = Pin<Gpio26, Input<Floating>>;

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
        //let mut motor = pins.gpio19.into_push_pull_output();

        let switch = pins.gpio1.into_mode();
        switch.set_interrupt_enabled(hal::gpio::Interrupt::EdgeLow, true);

        let adc = hal::Adc::new(context.device.ADC, &mut resets);
        let sensor = pins.gpio26.into_floating_input();

        led.set_high().ok();

        toggle_motor::spawn().ok();

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
    fn toggle_motor(context: toggle_motor::Context) {
        toggle_motor::spawn_after(200.millis()).ok();

        let led = context.local.led;
        let adc = context.local.adc;
        let sensor = context.local.sensor;

        if let Ok::<u16, _>(value) = adc.read(sensor) {
            if value > 200 {
                led.set_high().ok();
            } else {
                led.toggle().ok();
            }
        };

        led.toggle().ok();
    }
}
