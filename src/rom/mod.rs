pub mod usbd;

#[repr(C)]
#[derive(Debug)]
pub struct RomDriver {
    pub usb_api: *const usbd::UsbRomDriver,
    reserved0: u32,
    pub can_api: u32,
    pub pwr_api: u32,
    reserved1: u32,
    reserved2: u32,
    reserved3: u32,
    reserved4: u32,
}

assert_eq_size!(RomDriver, [u8; 0x20]);

impl RomDriver {
    pub fn ptr() -> *const Self {
        let vtable_ptr = 0x1FFF_1FF8 as *const u32;

        unsafe {
            *vtable_ptr as *const _
        }
    }

    pub fn get() -> &'static Self {
        unsafe { &*Self::ptr() }
    }

    pub fn usb_api(&self) -> &usbd::UsbRomDriver {
        unsafe { &*self.usb_api }
    }
}
