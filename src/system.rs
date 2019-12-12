use lpc11uxx::Peripherals;

pub static CRYSTAL_OSCILLATOR_CLOCK_RATE: u32 = 12_000_000;
pub static SYSTEM_PPL_MSET: u8 = 3;
pub static SYSTEM_PPL_PSET: u8 = 1;

fn setup_system_ppl(peripherals: &mut Peripherals, msel: u8, psel: u8) {
    unsafe {
        peripherals
            .SYSCON
            .syspllctrl
            .write(|writer| writer.msel().bits(msel).psel().bits(psel));
    }
}

pub fn initialize() {
    let mut peripherals = unsafe { Peripherals::steal() };

    // Power up Crystal oscillator
    peripherals
        .SYSCON
        .pdruncfg
        .modify(|_, writer| writer.sysosc_pd().powered());

    // Select Crystal Oscillator as System PLL clock source
    peripherals
        .SYSCON
        .syspllclksel
        .write(|writer| writer.sel().crystal_oscillator());
    peripherals
        .SYSCON
        .syspllclkuen
        .write(|writer| writer.ena().no_change());
    peripherals
        .SYSCON
        .syspllclkuen
        .write(|writer| writer.ena().update_clock_source());

    // Power down System PLL clock, we are going to change divisor
    peripherals
        .SYSCON
        .pdruncfg
        .modify(|_, writer| writer.syspll_pd().powered_down());

    // Setup System PPL (Division ration is 2 x 4, feedback divider value is 3 + 1)
    setup_system_ppl(&mut peripherals, SYSTEM_PPL_MSET, SYSTEM_PPL_PSET);

    // Power up System PLL clock again
    peripherals
        .SYSCON
        .pdruncfg
        .modify(|_, writer| writer.syspll_pd().powered());

    // Wait until System PLL clock is ready
    while peripherals.SYSCON.syspllstat.read().lock().is_pll_locked() {}

    // Setup AHB clock divisor to 1.
    peripherals
        .SYSCON
        .sysahbclkdiv
        .write(|writer| unsafe { writer.div().bits(1) });

    // Select main clock source to System PLL output
    peripherals
        .SYSCON
        .mainclksel
        .write(|writer| writer.sel().pll_output());
    peripherals
        .SYSCON
        .mainclkuen
        .write(|writer| writer.ena().no_change());
    peripherals
        .SYSCON
        .mainclkuen
        .write(|writer| writer.ena().update_clock_source());

    // Enable IOCON clock
    peripherals
        .SYSCON
        .sysahbclkctrl
        .modify(|_, writer| writer.iocon().enabled());
}
