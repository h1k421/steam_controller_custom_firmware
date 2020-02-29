use lpc11uxx::Peripherals;

pub static CRYSTAL_OSCILLATOR_CLOCK_RATE: u32 = 12_000_000;
pub static SYSTEM_PPL_MSET: u8 = 3;
pub static SYSTEM_PPL_PSET: u8 = 1;

fn stage1() {
    let peripherals = unsafe { Peripherals::steal() };

    // Power up Crystal oscillator
    peripherals
        .SYSCON
        .pdruncfg
        .modify(|_, writer| writer.sysosc_pd().powered());

    // There's supposed to be an empty loop from 0 to 0x1600 here, but it
    // doesn't look necessary?

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
    unsafe {
        peripherals.SYSCON.syspllctrl.write(|writer| {
            writer
                .msel()
                .bits(SYSTEM_PPL_MSET)
                .psel()
                .bits(SYSTEM_PPL_PSET)
        });
    }

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

    // Setup FLASH to 50Mhz
    peripherals
        .FLASHCTRL
        .flashcfg
        .modify(|_, writer| writer.flashtim()._3_system_clocks_flas());

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

    // Select Crystal Oscillator as USB PLL clock source
    peripherals
        .SYSCON
        .usbpllclksel
        .write(|writer| writer.sel().system_oscillator());
    peripherals
        .SYSCON
        .usbpllclkuen
        .write(|writer| writer.ena().no_change());
    peripherals
        .SYSCON
        .usbpllclkuen
        .write(|writer| writer.ena().update_clock_source());

    // Setup USB PPL (Division ration is 2 x 4, feedback divider value is 3 + 1)
    unsafe {
        peripherals.SYSCON.usbpllctrl.write(|writer| {
            writer
                .msel()
                .bits(SYSTEM_PPL_MSET)
                .psel()
                .bits(SYSTEM_PPL_PSET)
        });
    }

    // Power up USB PLL clock and USB transceiver.
    peripherals.SYSCON.pdruncfg.modify(|_, writer| {
        writer
            .usbpll_pd()
            .powered()
            .usbpad_pd()
            .usb_transceiver_poweered()
    });

    // Wait until USB PLL clock is ready
    while peripherals.SYSCON.usbpllstat.read().lock().is_pll_locked() {}

    // Enable IOCON clock
    peripherals
        .SYSCON
        .sysahbclkctrl
        .modify(|_, writer| writer.iocon().enabled());

}

pub fn initialize() {
    stage1();
    //crate::led::initialize();
}