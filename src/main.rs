#![no_std]
#![no_main]

// pick a panicking behavior
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

use cortex_m::asm;
use cortex_m_rt::entry;

use lpc11uxx::Peripherals;

fn setup_system_ppl(peripherals: &mut Peripherals, msel: u8, psel: u8) {
    unsafe {
        peripherals.SYSCON.syspllctrl.write(|writer| writer.msel().bits(msel).psel().bits(psel));
    }
}

#[inline]
pub fn modify_pdruncfg<F>(peripherals: &mut Peripherals, f: F) where F: FnOnce(&mut lpc11uxx::syscon::pdruncfg::W) -> &mut lpc11uxx::syscon::pdruncfg::W, {
    unsafe {
        peripherals.SYSCON.pdruncfg.modify(|reader, writer| f(writer.bits(reader.bits() | 0xE800)));
    }
}

fn initialize_system() {
    let mut peripherals = unsafe {
        Peripherals::steal()
    };

    // Power up Crystal oscillator
    modify_pdruncfg(&mut peripherals, |writer| writer.sysosc_pd().powered());

    // Select Crystal Oscillator as System PLL clock source
    peripherals.SYSCON.syspllclksel.write(|writer| writer.sel().crystal_oscillator());
    peripherals.SYSCON.syspllclkuen.write(|writer| writer.ena().no_change());
    peripherals.SYSCON.syspllclkuen.write(|writer| writer.ena().update_clock_source());

    // Power down System PLL clock, we are going to change divisor
    modify_pdruncfg(&mut peripherals, |writer| writer.syspll_pd().powered_down());

    // Setup System PPL (Division ration is 2 x 4, feedback divider value is 3 + 1)
    setup_system_ppl(&mut peripherals, 3, 1);

    // Power up System PLL clock again
    modify_pdruncfg(&mut peripherals, |writer| writer.syspll_pd().powered());

    // Wait until System PLL clock is ready
    while peripherals.SYSCON.syspllstat.read().lock().is_pll_locked() {}

    // Setup AHB clock divisor to 1.
    peripherals.SYSCON.sysahbclkdiv.write(|writer| {
        unsafe {
            writer.div().bits(1)
        }
    });

    // Select main clock source to System PLL output
    peripherals.SYSCON.mainclksel.write(|writer| writer.sel().pll_output());
    peripherals.SYSCON.mainclkuen.write(|writer| writer.ena().no_change());
    peripherals.SYSCON.mainclkuen.write(|writer| writer.ena().update_clock_source());

    // Enable IOCON clock
    peripherals.SYSCON.sysahbclkctrl.modify(|_, writer| writer.iocon().enabled());
}

fn led_init() {
    let peripherals = unsafe {
        Peripherals::steal()
    };

    // Enable LED clock (CT16B1)
    peripherals.SYSCON.sysahbclkctrl.modify(|_, writer| writer.ct16b1().enabled());

    // Clear prescale register
    unsafe {
        peripherals.CT16B1.pr.write(|writer| writer.pcval().bits(0));
    }

    // Enable PWM mode for CT16B1_MAT0
    peripherals.CT16B1.pwmc.write(|writer| writer.pwmen0().enabled());

    unsafe {
        peripherals.CT16B1.mr[3].write(|writer| writer.bits(0xFFF));
        peripherals.CT16B1.mr[0].write(|writer| writer.bits(0x1000));
    }

    peripherals.CT16B1.mcr.modify(|_, writer| writer.mr3r().enabled());

    // Reset timer
    let backup_tcr = peripherals.CT16B1.tcr.read().bits();

    peripherals.CT16B1.tcr.write(|writer| writer.cen().the_counters_are_dis().crst().do_nothing());

    unsafe {
        peripherals.CT16B1.tc.write(|writer| writer.tc().bits(1));
    }

    peripherals.CT16B1.tcr.write(|writer| writer.crst().reset());

    while peripherals.CT16B1.tc.read().bits() != 0 {}

    unsafe {
        peripherals.CT16B1.tcr.write(|writer| writer.bits(backup_tcr));
    }

    peripherals.CT16B1.tcr.modify(|_, writer| writer.cen().the_timer_counter_an());
    unsafe {
        // FIXME: Doesn't work?
        //peripherals.IOCON.pio0_21.write(|writer| writer.func().ct16b1_mat0().mode().inactive());
        peripherals.IOCON.pio0_21.write(|writer| writer.bits(0x1));
    }
}

#[entry]
fn main() -> ! {

    initialize_system();
    led_init();

    loop {
        // your code goes here
        asm::wfi();
    }
}
