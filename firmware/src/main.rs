#![no_std]
#![no_main]

#[macro_use]
extern crate static_assertions;

#[macro_use]
extern crate enum_primitive;

use cortex_m::asm;
use cortex_m_rt::{entry, exception};

mod led;
mod rom;
mod rt;
mod system;
mod usbd;

use rom::usbd::{CoreDescriptors, DeviceDescriptor, InitParameter};
use rom::RomDriver;

use lpc11uxx::interrupt;

use usb_device::prelude::*;

use usbd_serial::*;

#[exception]
fn DefaultHandler(_irq: i16) {
    unimplemented!()
}

/*fn usb_init() {
    let rom_driver = RomDriver::get();

    let usb_api = rom_driver.usb_api();

    let mut configuration: InitParameter = InitParameter::default();

    configuration.usb_reg_base = 0x40080000;
    configuration.mem_base = 0x20004000;
    configuration.mem_size = 0x0800;
    configuration.max_num_ep = 3 + 1;

    unsafe {
        // Setup string descriptor
        let mut string_descriptor_writer = DescriptorWriter::new(&mut STRING_DESCRIPTOR);
        string_descriptor_writer.write(descriptor_type::STRING, &lang_id::ENGLISH_US.to_le_bytes()).unwrap();

        // Manufacturer
        string_descriptor_writer.string("Mary").unwrap();

        // Product
        string_descriptor_writer.string("Steam Controller").unwrap();

        // Serial Number
        string_descriptor_writer.string("Gay").unwrap();

        // Interface name
        string_descriptor_writer.string("UART").unwrap();

        let mut device_configuration_writer = DescriptorWriter::new(&mut DEVICE_CONFIGURATION);



        let core_descriptor = CoreDescriptors {
            device_descriptors: &DEVICE_DESCRIPTOR,
            string_descriptors: &STRING_DESCRIPTOR as *const _,
            full_speed_descriptors: &DEVICE_CONFIGURATION as *const _,
            high_speed_descriptors: &DEVICE_CONFIGURATION as *const _,
            device_qualifier: core::ptr::null(),
        };

        let result = (usb_api.hw().init)(&mut USB_HANDLE, &core_descriptor, &configuration);
    }
}*/

unsafe fn usb_init() {
    usbd::USB_BUS_ALLOCATOR = Some(usbd::UsbBus::new());

    if let Some(ref usb_bus) = usbd::USB_BUS_ALLOCATOR {
        let mut serial = SerialPort::new(usb_bus);
        usbd::USB_DEVICE = Some(
            UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
                .product("Serial port")
                .device_class(USB_CLASS_CDC)
                .build(),
        );

        if let Some(ref mut usb_device) = usbd::USB_DEVICE {
            loop {
                if !usb_device.poll(&mut [&mut serial]) {
                    continue;
                }

                let mut buf = [0u8; 64];

                match serial.read(&mut buf[..]) {
                    Ok(count) => {
                        // count bytes were read to &buf[..count]
                    }
                    //Err(UsbError::WouldBlock) => {}
                    Err(err) => {
                        //panic!()
                    }
                };

                match serial.write(&[0x3a, 0x29]) {
                    Ok(count) => {
                        // count bytes were written
                    }
                    //Err(UsbError::WouldBlock) => {}
                    Err(err) => {
                        //panic!()
                    }
                };

                // Wait for interruption (as polling is handled in the IRQ)
                asm::wfi();
            }
        }
    }
}

#[entry]
fn main() -> ! {
    system::initialize();
    led::initialize();

    led::set_intensity(0x1000);

    unsafe {
        usb_init();
    }

    loop {
        asm::wfi();
    }
}
