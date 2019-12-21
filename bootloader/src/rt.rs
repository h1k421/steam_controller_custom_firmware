use core::panic::PanicInfo;
use embedded_hal::blocking::delay::DelayMs;
use lpc11uxx::CorePeripherals;
use lpc11uxx_hal::delay::Delay;

use crate::led;
use crate::system::{CRYSTAL_OSCILLATOR_CLOCK_RATE, SYSTEM_PPL_MSET};

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    led_panic(0x1000, 1000)
}

pub fn led_panic(start_intensity: u16, delay_ms: u32) -> ! {
    led::initialize();

    let peripherals = unsafe { CorePeripherals::steal() };

    let mut delay = Delay::new(
        peripherals.SYST,
        CRYSTAL_OSCILLATOR_CLOCK_RATE * u32::from(SYSTEM_PPL_MSET),
    );

    let mut intensity = start_intensity;
    loop {
        led::set_intensity(intensity);

        if intensity == 0 {
            intensity = start_intensity;
        } else {
            intensity = 0;
        }
        delay.delay_ms(delay_ms);
    }
}

pub fn led_blink_n_times(start_intensity: u16, n: u32) {
    led::initialize();

    let peripherals = unsafe { CorePeripherals::steal() };

    let mut delay = Delay::new(
        peripherals.SYST,
        CRYSTAL_OSCILLATOR_CLOCK_RATE * u32::from(SYSTEM_PPL_MSET),
    );

    let mut intensity = start_intensity;
    for i in 0..n * 2 {
        led::set_intensity(intensity);

        if intensity == 0 {
            intensity = start_intensity;
        } else {
            intensity = 0;
        }
        delay.delay_ms(1000);
    }
}