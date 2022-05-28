#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use panic_semihosting as _;
use rtic::app;

#[app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [XIP_IRQ])]
mod app {
    use cortex_m_semihosting::{debug, hprintln};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(context: init::Context) -> (Shared, Local, init::Monotonics) {

        let _core = context.core;
        let _device = context.device;

        hprintln!("init");

        debug::exit(debug::EXIT_SUCCESS);

        (Shared {}, Local {}, init::Monotonics())
    }
}