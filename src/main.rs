#![no_std]
#![no_main]

use cortex_m::asm;
use cortex_m_rt::entry;

mod led;
mod rt;
mod system;

#[entry]
fn main() -> ! {
    system::initialize();
    led::initialize();

    led::set_intensity(0x1000);

    loop {
        asm::wfi();
    }
}
