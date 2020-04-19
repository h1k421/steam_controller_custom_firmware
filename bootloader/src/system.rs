use lpc11uxx::{SYSCON, FLASHCTRL};

pub static CRYSTAL_OSCILLATOR_CLOCK_RATE: u32 = 12_000_000;
pub static SYSTEM_PPL_MSET: u8 = 3;
pub static SYSTEM_PPL_PSET: u8 = 1;

fn stage1(syscon: &mut SYSCON, flashctrl: &mut FLASHCTRL) {
    // Power up Crystal oscillator
    syscon
        .pdruncfg
        .modify(|_, writer| writer.sysosc_pd().powered());

    // TODO: Wait 200us.

    // Select Crystal Oscillator as System PLL clock source
    syscon
        .syspllclksel
        .write(|writer| writer.sel().crystal_oscillator());
    syscon
        .syspllclkuen
        .write(|writer| writer.ena().no_change());
    syscon
        .syspllclkuen
        .write(|writer| writer.ena().update_clock_source());

    // Power down System PLL clock, we are going to change divisor
    syscon
        .pdruncfg
        .modify(|_, writer| writer.syspll_pd().powered_down());

    // Setup System PPL (Division ration is 2 x 4, feedback divider value is 3 + 1)
    unsafe {
        syscon.syspllctrl.write(|writer| {
            writer
                .msel()
                .bits(SYSTEM_PPL_MSET)
                .psel()
                .bits(SYSTEM_PPL_PSET)
        });
    }

    // Power up System PLL clock again
    syscon
        .pdruncfg
        .modify(|_, writer| writer.syspll_pd().powered());

    // Wait until System PLL clock is ready
    while syscon.syspllstat.read().lock().is_pll_locked() {}

    // Setup AHB clock divisor to 1.
    syscon
        .sysahbclkdiv
        .write(|writer| unsafe { writer.div().bits(1) });

    // Setup FLASH to 50Mhz
    flashctrl
        .flashcfg
        .modify(|_, writer| writer.flashtim()._3_system_clocks_flas());

    // Select main clock source to System PLL output
    syscon
        .mainclksel
        .write(|writer| writer.sel().pll_output());
    syscon
        .mainclkuen
        .write(|writer| writer.ena().no_change());
    syscon
        .mainclkuen
        .write(|writer| writer.ena().update_clock_source());

    // Select Crystal Oscillator as USB PLL clock source
    syscon
        .usbpllclksel
        .write(|writer| writer.sel().system_oscillator());
    syscon
        .usbpllclkuen
        .write(|writer| writer.ena().no_change());
    syscon
        .usbpllclkuen
        .write(|writer| writer.ena().update_clock_source());

    // Setup USB PPL (Division ration is 2 x 4, feedback divider value is 3 + 1)
    unsafe {
        syscon.usbpllctrl.write(|writer| {
            writer
                .msel()
                .bits(SYSTEM_PPL_MSET)
                .psel()
                .bits(SYSTEM_PPL_PSET)
        });
    }

    // Power up USB PLL clock and USB transceiver.
    syscon.pdruncfg.modify(|_, writer| {
        writer
            .usbpll_pd()
            .powered()
            .usbpad_pd()
            .usb_transceiver_poweered()
    });

    // Wait until USB PLL clock is ready
    while syscon.usbpllstat.read().lock().is_pll_locked() {}

    // Enable IOCON clock
    syscon
        .sysahbclkctrl
        .modify(|_, writer| writer.iocon().enabled());

}

pub fn initialize(syscon: &mut SYSCON, flashctrl: &mut FLASHCTRL) {
    stage1(syscon, flashctrl);
    //crate::led::initialize();
}