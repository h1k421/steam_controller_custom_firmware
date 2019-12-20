use lpc11uxx::Peripherals;

pub fn initialize() {
    let peripherals = unsafe { Peripherals::steal() };

    // Enable LED clock (CT16B1)
    peripherals
        .SYSCON
        .sysahbclkctrl
        .modify(|_, writer| writer.ct16b1().enabled());

    // Clear prescale register
    unsafe {
        peripherals.CT16B1.pr.write(|writer| writer.pcval().bits(0));
    }

    // Enable PWM mode for CT16B1_MAT0
    peripherals
        .CT16B1
        .pwmc
        .write(|writer| writer.pwmen0().enabled());

    unsafe {
        peripherals.CT16B1.mr[3].write(|writer| writer.bits(0xFFF));
        peripherals.CT16B1.mr[0].write(|writer| writer.bits(0x0));
    }

    peripherals
        .CT16B1
        .mcr
        .modify(|_, writer| writer.mr3r().enabled());

    // Reset timer
    let backup_tcr = peripherals.CT16B1.tcr.read().bits();

    peripherals
        .CT16B1
        .tcr
        .write(|writer| writer.cen().the_counters_are_dis().crst().do_nothing());

    unsafe {
        peripherals.CT16B1.tc.write(|writer| writer.tc().bits(1));
    }

    peripherals.CT16B1.tcr.write(|writer| writer.crst().reset());

    while peripherals.CT16B1.tc.read().bits() != 0 {}

    unsafe {
        peripherals
            .CT16B1
            .tcr
            .write(|writer| writer.bits(backup_tcr));
    }

    peripherals
        .CT16B1
        .tcr
        .modify(|_, writer| writer.cen().the_timer_counter_an());
    unsafe {
        peripherals
            .IOCON
            .pio0_21
            .write(|writer| writer.func().ct16b1_mat0().mode().inactive());
    }
}

pub fn set_intensity(intensity: u16) {
    let peripherals = unsafe { Peripherals::steal() };

    unsafe {
        peripherals.CT16B1.mr[0].write(|writer| writer.bits(u32::from(intensity)));
    }
}
