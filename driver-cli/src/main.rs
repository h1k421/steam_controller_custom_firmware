use std::convert::TryInto;
use std::io::{self, Seek, SeekFrom, Read};

use hidapi_rs::*;

fn reboot_to_bootloader(device: &HidDevice) {
    let mut data = vec![0; 0x40 + 1];
    data[0] = 0;
    data[1] = 0x95;
    data[2] = 4;
    data[3..3 + 4].copy_from_slice(&0xecaabac0_u32.to_le_bytes());

    device.send_feature_report(&data).unwrap();
}

#[derive(Debug)]
struct HardwareInfo {
    unk1: u8,
    unk2: u8,
    usb_pid: u32,
    unk3: u8,
    bootloader_version: u32,
    unk4: u8,
    eeprom_magic: u32
}

fn get_hardware_info(device: &HidDevice) -> Option<HardwareInfo> {
    let mut data = vec![0; 0x40 + 1];
    data[0] = 0;
    data[1] = 0x83;

    device.send_feature_report(&data).unwrap();

    let mut data = vec![0; 0x40 + 1];

    let data_len = get_feature_report_workaround(device, &mut data);
    let data = &data[..data_len];

    if data[0] != 0 {
        return None;
    }

    match data[1] {
        0x83 => {
            Some(HardwareInfo {
                unk1: data[2], // 0xf
                unk2: data[3], // 1
                usb_pid: u32::from_le_bytes(data[4..8].try_into().unwrap()),
                unk3: data[8], // 4
                bootloader_version: u32::from_le_bytes(data[9..13].try_into().unwrap()),
                unk4: data[13], // 9
                eeprom_magic: u32::from_le_bytes(data[14..18].try_into().unwrap()),
            })
        },
        _ => None
    }
}

/// Data should be less than 0x3E bytes in size.
fn flash_data(device: &HidDevice, to_flash: &[u8]) -> Option<()> {
    let mut data = vec![0; 0x40 + 1];
    data[0] = 0;
    data[1] = 0x92;
    data[2] = to_flash.len() as u8;
    data[3..3 + to_flash.len()].copy_from_slice(to_flash);

    device.send_feature_report(&data).unwrap();

    let data_len = get_feature_report_workaround(device, &mut data);
    let data = &data[..data_len];

    if data[0] != 0 {
        return None;
    }

    match data[1] {
        0x92 => Some(()),
        _ => None
    }
}

fn get_feature_report_workaround(device: &HidDevice, data: &mut [u8]) -> usize {
    let mut data_len;
    loop {
        data_len = device.get_feature_report(&mut data[..]).unwrap();
        if data_len != 1 {
            break
        }
    }
    data_len
}

fn verify_flash_data(device: &HidDevice, signature: &[u8]) -> Option<()> {
    let mut data = vec![0; 0x40 + 1];
    data[0] = 0;
    data[1] = 0x93;
    data[2] = signature.len() as u8;
    data[3..3 + signature.len()].copy_from_slice(signature);

    device.send_feature_report(&data).unwrap();

    let data_len = get_feature_report_workaround(device, &mut data);
    let data = &data[..data_len];

    if data[0] != 0 {
        return None;
    }

    let mut sig = [0; 4];
    for i in 0..4 {
        sig[i] = u32::from_le_bytes(data[5 + i * 4..5 + (1 + i) * 4].try_into().unwrap())
    }

    println!("{:x?}", sig);
    println!("flash_stop_idx = {:x}", u32::from_le_bytes(data[21..25].try_into().unwrap()));

    match data[1] {
        0x94 => Some(()),
        _ => None
    }
}

fn erase_program2(device: &HidDevice) -> Option<()> {
    let mut data = vec![0; 0x40 + 1];
    data[0] = 0;
    data[1] = 0x91;
    device.send_feature_report(&data).unwrap();

    let data_len = get_feature_report_workaround(device, &mut data);
    let data = &data[..data_len];

    if data[0] != 0 {
        return None;
    }

    match data[1] {
        0x94 => Some(()),
        _ => None
    }
}

fn find_bootloader_device(hidapi: &mut HidApi) -> Box<HidDevice> {
    loop {
        hidapi.refresh_devices().unwrap();
        if let Some(device) = hidapi.devices().iter().find(|v|
            v.vendor_id == 0x28de && v.product_id == 0x1102 && v.interface_number == 2)
        {
            reboot_to_bootloader(&device.open_device(&hidapi).unwrap());
            std::thread::sleep(std::time::Duration::from_secs(1));
        }
        if let Some(device) = hidapi.devices().iter().find(|v|
            v.vendor_id == 0x28de && v.product_id == 0x1002 && v.interface_number == 0)
        {
            return device.open_device(&hidapi).unwrap();
        }
        println!("Failed to find steam controller.");
        println!("Make sure your controller is plugged, and press enter");
        println!("Devices:");
        for device in hidapi.devices() {
            println!("- {:?}", device);
        }
        let _ = std::io::stdin().read_line(&mut String::new()).unwrap();
    };
}

fn main() {
    /*let mut hidapi = HidApi::new().unwrap();

    let device = find_bootloader_device(&mut hidapi);
    println!("{:?}", get_hardware_info(&device));*/

    let mut firmware_file = std::fs::File::open("firmware_from_flash.bin").unwrap();
    //firmware_file.seek(SeekFrom::Start(0x2000)).unwrap();

    //erase_program2(&device).unwrap();

    //let mut firmware_data = [0; 0x3e];
    //loop {
    //    let bytes_read = firmware_file.read(&mut firmware_data).unwrap();
    //    if bytes_read == 0 {
    //        break;
    //    }
    //    flash_data(&device, &firmware_data[..bytes_read]).unwrap();
    //}

    firmware_file.seek(SeekFrom::Start(0x30)).unwrap();

    let mut cur_word = [0u32; 4];
    let mut ref_signature = [0u32; 4];
    let mut next_signature = [0u32; 4];
    loop {
        let mut buf = [0; 0x200];
        match firmware_file.read_exact(&mut buf) {
            Err(err) if err.kind() == io::ErrorKind::UnexpectedEof => break,
            v => v.unwrap()
        }

        for buf in buf.chunks(16) {
            cur_word[0] = u32::from_le_bytes(buf[0..4].try_into().unwrap());
            cur_word[1] = u32::from_le_bytes(buf[4..8].try_into().unwrap());
            cur_word[2] = u32::from_le_bytes(buf[8..12].try_into().unwrap());
            cur_word[3] = u32::from_le_bytes(buf[12..16].try_into().unwrap());

            next_signature[0] = cur_word[0] ^ ref_signature[0] >> 1 ^ ref_signature[1] << 31;
            next_signature[1] = cur_word[1] ^ ref_signature[1] >> 1 ^ ref_signature[2] << 31;
            next_signature[2] = cur_word[2] ^ ref_signature[2] >> 1 ^ ref_signature[3] << 31;
            next_signature[3] = cur_word[3] ^ ref_signature[3] >> 1 ^
                (ref_signature[0] & (1 << 29)) << 02 ^
                (ref_signature[0] & (1 << 27)) << 04 ^
                (ref_signature[0] & (1 << 02)) << 29 ^
                (ref_signature[0] & (1 << 00)) << 31;

            ref_signature = next_signature;
        }
    }

    println!("Expecting signature {:x?}", ref_signature);

    //let mut signature = [0; 16];
    //signature[0..4].copy_from_slice(&ref_signature[0].to_le_bytes());
    //signature[4..8].copy_from_slice(&ref_signature[1].to_le_bytes());
    //signature[8..12].copy_from_slice(&ref_signature[2].to_le_bytes());
    //signature[12..16].copy_from_slice(&ref_signature[3].to_le_bytes());

    //verify_flash_data(&device, &signature).unwrap();
}