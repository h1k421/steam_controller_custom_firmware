//#![feature(const_loop)]
//#![feature(const_if_match)]
//#![feature(const_int_conversion)]

use bitflags::bitflags;
use bitfield::bitfield;

#[macro_export]
macro_rules! combine_descriptors {
    ($($descriptor:expr),* $(,)?) => {{
        let mut data = [0; 1 $(+ $descriptor.len())*];
        let mut idx = 0;

        $({
            let arr = $descriptor.to_arr();
            let mut i = 0;
            while i < $descriptor.len() {
                data[idx + i] = arr[i];
                i += 1;
            }
            idx += i;
        })*

        // Ensure the usb descriptor looks valid
        assert_configuration_valid(&data);

        data
    }};
}

bitflags! {
    pub struct ConfigurationAttributes: u8 {
        const BUS_POWERED   = 0b10000000;
        const SELF_POWERED  = 0b01000000;
        const REMOTE_WAKEUP = 0b00100000;
    }
}

pub struct ConfigurationDescriptor {
    pub total_length: u16,
    pub num_interfaces: u8,
    pub configuration_value: u8,
    pub configuration_name_idx: u8,
    pub attributes: ConfigurationAttributes,
    pub max_power: u8,
}

impl ConfigurationDescriptor {
    const LEN: usize = 9;

    pub const fn to_arr(&self) -> [u8; Self::LEN] {
        [
            self.len() as u8, 0x02, self.total_length.to_le_bytes()[0],
            self.total_length.to_le_bytes()[1], self.num_interfaces,
            self.configuration_value, self.configuration_name_idx,
            self.attributes.bits(), self.max_power
        ]
    }

    pub const fn len(&self) -> usize {
        Self::LEN
    }
}

pub struct InterfaceDescriptor {
    pub interface_num: u8,
    pub alternate_setting: u8,
    pub num_endpoints: u8,
    pub interface_class: u8,
    pub interface_subclass: u8,
    pub interface_protocol: u8,
    pub interface_name_idx: u8
}

impl InterfaceDescriptor {
    const LEN: usize = 9;

    pub const fn to_arr(&self) -> [u8; Self::LEN] {
        [
            self.len() as u8, 0x04, self.interface_num, self.alternate_setting,
            self.num_endpoints, self.interface_class, self.interface_subclass,
            self.interface_protocol, self.interface_name_idx
        ]
    }

    pub const fn len(&self) -> usize {
        Self::LEN
    }
}

bitfield! {
    pub struct EndpointAttributes(u8);
    impl Debug;
    pub transfer_type, set_transfer_type: 1, 0;
    pub synchronisation_type, set_synchronisation_type: 3, 2;
    pub usage_type, set_usage_type: 5, 4;
}

pub struct EndpointDescriptor {
    pub endpoint_addr: u8,
    pub attributes: u8,
    pub max_packet_size: u16,
    pub interval: u8
}

impl EndpointDescriptor {
    const LEN: usize = 7;

    pub const fn to_arr(&self) -> [u8; Self::LEN] {
        [
            self.len() as u8, 0x05, self.endpoint_addr, self.attributes,
            self.max_packet_size.to_le_bytes()[0],
            self.max_packet_size.to_le_bytes()[1],
            self.interval
        ]
    }

    pub const fn len(&self) -> usize {
        Self::LEN
    }
}

pub struct InterfaceAssociationDescriptor {
    pub first_interface: u8,
    pub interface_count: u8,
    pub function_class: u8,
    pub subfunction_class: u8,
    pub function_protocol: u8,
    pub function_name_idx: u8
}

impl InterfaceAssociationDescriptor {
    const LEN: usize = 8;

    pub const fn to_arr(&self) -> [u8; Self::LEN] {
        [
            self.len() as u8, 0x0b, self.first_interface, self.interface_count,
            self.function_class, self.subfunction_class, self.function_protocol,
            self.function_name_idx,
        ]
    }

    pub const fn len(&self) -> usize {
        Self::LEN
    }
}

pub const fn assert_configuration_valid(data: &[u8]) {
    let mut idx = 0;
    let mut interfaces_expected = 0;
    let mut interfaces_found = 0;
    let mut configuration_descriptor_found = false;
    let mut endpoints_found = 0;
    let mut endpoints_expected = 0;
    while idx < data.len() {
        let len = data[idx];
        if len == 0 && idx == data.len() - 1 {
            if interfaces_found != interfaces_expected {
                panic!("Number of interfaces mismatches between configuration descriptor and array");
            }
            if endpoints_found != endpoints_expected {
                panic!("Number of endpoints mismatches between interface descriptor and array");
            }
            return
        } else if len == 0 {
            panic!("Weird 0-length descriptor in the middle of the configuration");
        } else if len == 1 {
            panic!("Weird 1-length descriptor");
        }

        let ty = data[idx + 1];
        match ty {
            2 if !configuration_descriptor_found => {
                configuration_descriptor_found = true;
                if data.len() - 1 != u16::from_le_bytes([data[idx + 2], data[idx + 3]]) as usize {
                    panic!("Configuration Descriptor total length does not match array length.")
                }
                interfaces_expected = data[idx + 4];
            },
            2 => panic!("Multiple configuration descriptors found"),
            4 => {
                if endpoints_found != endpoints_expected {
                    panic!("Mismatch between expected endpoints and endpoints found");
                }
                interfaces_found += 1;
                endpoints_expected = data[idx + 4];
                endpoints_found = 0;
            },
            5 => {
                endpoints_found += 1;
            },
            _ => (),
        }

        idx += len as usize;
    }

    panic!("Configuration ended without terminator.")
}

mod hid;
mod cdc;

pub use hid::*;
pub use cdc::*;
