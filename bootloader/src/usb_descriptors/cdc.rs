const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;

const CS_HEADER: u8 = 0x00;
const CS_CALL_MANAGEMENT: u8 = 0x01;
const CS_ABSTRACT_CONTROL_MANAGEMENT: u8 = 0x02;
const CS_UNION: u8 = 0x06;

pub struct CdcHeaderFunctionalDescriptor {
    pub cdc_version: u16
}

impl CdcHeaderFunctionalDescriptor {
    const LEN: usize = 5;
    pub const fn to_arr(&self) -> [u8; Self::LEN] {
        [
            self.len() as u8, CS_INTERFACE, CS_HEADER,
            self.cdc_version.to_le_bytes()[0],
            self.cdc_version.to_le_bytes()[1],
        ]
    }

    pub const fn len(&self) -> usize {
        Self::LEN
    }
}

pub struct CdcCallManagementFunctionalDescriptor {
    pub capabilities: u8,
    pub data_interface: u8
}

impl CdcCallManagementFunctionalDescriptor {
    const LEN: usize = 5;
    pub const fn to_arr(&self) -> [u8; Self::LEN] {
        [
            self.len() as u8, CS_INTERFACE, CS_CALL_MANAGEMENT,
            self.capabilities, self.data_interface,
        ]
    }

    pub const fn len(&self) -> usize {
        Self::LEN
    }
}

pub struct CdcAbstractControlManagementFunctionalDescriptor {
    pub capabilities: u8,
}

impl CdcAbstractControlManagementFunctionalDescriptor {
    const LEN: usize = 4;
    pub const fn to_arr(&self) -> [u8; Self::LEN] {
        [
            self.len() as u8, CS_INTERFACE, CS_ABSTRACT_CONTROL_MANAGEMENT,
            self.capabilities
        ]
    }

    pub const fn len(&self) -> usize {
        Self::LEN
    }
}

pub struct CdcUnionSlaveInterface {
    pub interface_num: u8
}

impl CdcUnionSlaveInterface {
    const LEN: usize = 1;
    pub const fn to_arr(&self) -> [u8; Self::LEN] {
        [
            self.interface_num
        ]
    }

    pub const fn len(&self) -> usize {
        Self::LEN
    }
}

pub struct CdcUnionFunctionalDescriptor {
    pub master_interface: u8,
    pub num_slave_interfaces: u8,
}

impl CdcUnionFunctionalDescriptor {
    const LEN: usize = 4;
    pub const fn to_arr(&self) -> [u8; Self::LEN] {
        [
            self.len() as u8 + self.num_slave_interfaces, CS_INTERFACE,
            CS_UNION, self.master_interface,
        ]
    }

    pub const fn len(&self) -> usize {
        Self::LEN
    }
}