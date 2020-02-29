pub struct HidDescriptorListItem {
    pub descriptor_type: u8,
    pub descriptor_length: u16,
}

impl HidDescriptorListItem {
    const LEN: usize = 3;

    pub const fn to_arr(&self) -> [u8; Self::LEN] {
        [
            self.descriptor_type,
            self.descriptor_length.to_le_bytes()[0],
            self.descriptor_length.to_le_bytes()[1]
        ]
    }

    pub const fn len(&self) -> usize {
        Self::LEN
    }
}

pub struct HidDescriptor {
    pub hid_version: u16,
    pub country_code: u8,
    pub hid_descriptors_num: u8,
}

impl HidDescriptor {
    const LEN: usize = 6;

    pub const fn to_arr(&self) -> [u8; Self::LEN] {
        [
            self.len() as u8 + 3 * self.hid_descriptors_num,
            0x21,
            self.hid_version.to_le_bytes()[0],
            self.hid_version.to_le_bytes()[1],
            self.country_code,
            self.hid_descriptors_num,
        ]
    }

    pub const fn len(&self) -> usize {
        Self::LEN
    }
}