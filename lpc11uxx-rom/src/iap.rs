#[inline(always)]
fn send_iap_command(cmd_in: &[u32; 5], cmd_out: &mut [u32; 4]) {
    let entryfn: extern "C" fn(&[u32; 5], &mut [u32; 4]) =
        unsafe { core::mem::transmute(0x1fff_1ff1) };

    cortex_m::interrupt::free(|_| {
        entryfn(cmd_in, cmd_out);
    });
}

#[repr(u32)]
enum IapCmd {
    PrepareSectorForWrite = 50,
    CopyRamToFlash = 51,
    EraseSectors = 52,
    BlankCheckSectors = 53,
    ReadPartID = 54,
    ReadBootCodeVersion = 55,
    Compare = 56,
    ReinvokeISP = 57,
    ReadUID = 58,
    ErasePage = 59,
    EepromWrite = 61,
    EepromRead = 62,
}

pub fn prepare_sector_for_write(start_sector_number: u32, end_sector_number: u32) -> i32 {
    let mut cmd_out = &mut [0; 4];
    send_iap_command(
        &[
            IapCmd::PrepareSectorForWrite as u32,
            start_sector_number,
            end_sector_number,
            0,
            0,
        ],
        &mut cmd_out,
    );

    return cmd_out[0] as i32;
}

pub fn copy_ram_to_flash(
    flash_dst: u32,
    ram_src: usize,
    byte_count: usize,
    system_clock_freq: u32,
) -> i32 {
    let mut cmd_out = &mut [0; 4];
    send_iap_command(
        &[
            IapCmd::CopyRamToFlash as u32,
            flash_dst,
            ram_src as u32,
            byte_count as u32,
            system_clock_freq,
        ],
        &mut cmd_out,
    );

    return cmd_out[0] as i32;
}

pub fn erase_sectors(
    start_sector_number: u32,
    end_sector_number: u32,
    system_clock_freq: u32,
) -> i32 {
    let mut cmd_out = &mut [0; 4];
    send_iap_command(
        &[
            IapCmd::EraseSectors as u32,
            start_sector_number,
            end_sector_number,
            system_clock_freq,
            0,
        ],
        &mut cmd_out,
    );

    return cmd_out[0] as i32;
}

//pub fn blank_check_sectors() -> i32 {}
//pub fn read_part_id() -> i32 {}
//pub fn read_boot_code_version() -> i32 {}
//pub fn compare() -> i32 {}
pub fn reinvoke_isp() -> ! {
    let mut cmd_out = &mut [0; 4];
    send_iap_command(
        &[
            IapCmd::ReinvokeISP as u32,
            0,
            0,
            0,
            0,
        ],
        &mut cmd_out
    );
    loop {
        cortex_m::asm::wfi();
    }
}
//pub fn read_uid() -> i32 {}
//pub fn erase_page() -> i32 {}

pub fn eeprom_write(eeprom_addr: u32, data: &[u8], system_clock_freq: u32) -> i32 {
    let mut cmd_out = &mut [0; 4];
    send_iap_command(
        &[
            IapCmd::EepromWrite as u32,
            eeprom_addr,
            data.as_ptr() as u32,
            data.len() as u32,
            system_clock_freq,
        ],
        &mut cmd_out,
    );

    return cmd_out[0] as i32;
}

pub fn eeprom_read(eeprom_addr: u32, data: &mut [u8], system_clock_freq: u32) -> i32 {
    let mut cmd_out = &mut [0; 4];
    send_iap_command(
        &[
            IapCmd::EepromRead as u32,
            eeprom_addr,
            data.as_ptr() as u32,
            data.len() as u32,
            system_clock_freq,
        ],
        &mut cmd_out,
    );

    return cmd_out[0] as i32;
}
