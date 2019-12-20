use core::panic::PanicInfo;
use embedded_hal::blocking::delay::DelayMs;
use lpc11uxx::CorePeripherals;
use lpc11uxx_hal::delay::Delay;

use crate::led;
use crate::system::{CRYSTAL_OSCILLATOR_CLOCK_RATE, SYSTEM_PPL_MSET};

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    led_panic()
}

fn led_panic() -> ! {
    let peripherals = unsafe { CorePeripherals::steal() };

    let mut delay = Delay::new(
        peripherals.SYST,
        CRYSTAL_OSCILLATOR_CLOCK_RATE * u32::from(SYSTEM_PPL_MSET),
    );

    let mut intensity = 0x1000;
    loop {
        led::set_intensity(intensity);

        if intensity == 0 {
            intensity = 0x1000;
        } else {
            intensity = 0;
        }
        delay.delay_ms(1000);
    }
}
