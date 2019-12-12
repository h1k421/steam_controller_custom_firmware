#![no_std]
#![no_main]

use cortex_m::asm;
use cortex_m_rt::{entry, exception};

mod led;
mod rt;
mod system;

#[exception]
fn DefaultHandler(_irq: i16) {
    unimplemented!()
}

#[entry]
fn main() -> ! {
    system::initialize();
    led::initialize();

    led::set_intensity(0x1000);

    loop {
        asm::wfi();
    }
}
