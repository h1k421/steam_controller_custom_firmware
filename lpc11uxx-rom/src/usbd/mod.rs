use bitfield::bitfield;
use static_assertions::assert_eq_size;
use core::ptr::NonNull;

pub const MAX_IF_COUNT: usize = 8;
pub const MAX_EP_COUNT: usize = 5;
pub const MAX_PACKET0: usize = 0x40;
pub const FS_MAX_BULK_PACKET: usize = 0x40;
pub const HS_MAX_BULK_PACKET: usize = 0x200;

#[repr(packed)]
#[derive(Debug, Copy, Clone)]
pub struct WordByte {
    pub raw_value: u16,
}

impl WordByte {
    #[inline]
    pub fn low(self) -> u8 {
        u8::from_le_bytes([self.raw_value.to_le_bytes()[0]])
    }

    #[inline]
    pub fn high(self) -> u8 {
        u8::from_le_bytes([self.raw_value.to_le_bytes()[1]])
    }
}

assert_eq_size!(WordByte, u16);

#[repr(C)]
#[derive(Debug)]
pub struct UsbRomDriver {
    pub hw: *const HardwareApi,
    pub core: *const CoreApi,
    pub msc: *const u32,
    pub dfu: *const u32,
    pub hid: *const HidApi,
    pub cdc: *const CdcApi,
    reserved: u32,
    pub version: u32,
}

impl UsbRomDriver {
    pub fn hw(&self) -> &HardwareApi {
        unsafe { &*self.hw }
    }

    pub fn core(&self) -> &CoreApi {
        unsafe { &*self.core }
    }

    pub fn hid(&self) -> &HidApi {
        unsafe { &*self.hid }
    }

    pub fn cdc(&self) -> &CdcApi {
        unsafe { &*self.cdc }
    }
}

// SAFETY: We only have one core so no issues here :)
unsafe impl Sync for UsbRomDriver {}

assert_eq_size!(UsbRomDriver, [u8; 0x20]);

#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum Recipient {
    Device = 0,
    Interface = 1,
    Endpoint = 2,
    Other = 3,
}

impl From<u8> for Recipient {
    fn from(request_type: u8) -> Recipient {
        match request_type {
            0 => Recipient::Device,
            1 => Recipient::Interface,
            2 => Recipient::Endpoint,
            3 => Recipient::Other,
            n => unreachable!("Got unexpected Recipient: {}", n),
        }
    }
}

impl From<Recipient> for u8 {
    fn from(request_type: Recipient) -> u8 {
        request_type as u8
    }
}

assert_eq_size!(Recipient, u8);

#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum EventType {
    Setup = 1,
    Out = 2,
    In = 3,
    OutNotAcknowledged = 4,
    InNotAcknowledged = 5,
    OutStalled = 6,
    InStalled = 7,
    OutDmaOutOfTransfer = 8,
    InDmaOutOfTransfer = 9,
    OutDmaNewDescriptorRequest = 10,
    InDmaNewDescriptorRequest = 11,
    OutDmaError = 12,
    InDmaError = 13,
    Reset = 14,
    StarOfFrame = 15,
    DeviceStatus = 16,
    DeviceError = 17,
    Unknown = 0xFF,
}

impl From<u8> for EventType {
    fn from(event_type: u8) -> EventType {
        match event_type {
            1 => EventType::Setup,
            2 => EventType::Out,
            3 => EventType::In,
            4 => EventType::OutNotAcknowledged,
            5 => EventType::InNotAcknowledged,
            6 => EventType::OutStalled,
            7 => EventType::InStalled,
            8 => EventType::OutDmaOutOfTransfer,
            9 => EventType::InDmaOutOfTransfer,
            10 => EventType::OutDmaNewDescriptorRequest,
            11 => EventType::InDmaNewDescriptorRequest,
            12 => EventType::OutDmaError,
            13 => EventType::InDmaError,
            14 => EventType::Reset,
            15 => EventType::StarOfFrame,
            16 => EventType::DeviceStatus,
            17 => EventType::DeviceError,
            _ => EventType::Unknown,
        }
    }
}

impl From<EventType> for u8 {
    fn from(request_type: EventType) -> u8 {
        request_type as u8
    }
}

assert_eq_size!(EventType, u8);

#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum RequestCategory {
    Standard = 0,
    Class = 1,
    Vendor = 2,
    Reserved = 3,
}

impl From<u8> for RequestCategory {
    fn from(request_category: u8) -> RequestCategory {
        match request_category {
            0 => RequestCategory::Standard,
            1 => RequestCategory::Class,
            2 => RequestCategory::Vendor,
            3 => RequestCategory::Reserved,
            n => unreachable!("Got unexpected RequestCategory: {}", n),
        }
    }
}

impl From<RequestCategory> for u8 {
    fn from(request_category: RequestCategory) -> u8 {
        request_category as u8
    }
}

assert_eq_size!(RequestCategory, u8);

#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum RequestDirection {
    HostToDevice = 0,
    DeviceToHost = 1,
}

impl From<u8> for RequestDirection {
    fn from(request_direction: u8) -> RequestDirection {
        match request_direction {
            0 => RequestDirection::HostToDevice,
            1 => RequestDirection::DeviceToHost,
            n => unreachable!("Got unexpected RequestDirection: {}", n),
        }
    }
}

impl From<RequestDirection> for u8 {
    fn from(request_direction: RequestDirection) -> u8 {
        request_direction as u8
    }
}

assert_eq_size!(RequestDirection, u8);

#[repr(packed)]
#[derive(Debug)]
pub struct EndpointDescriptor {
    pub length: u8,
    pub descryptor_type: u8,
    pub endpoint_address: u8,
    pub attributes: u8,
    pub max_packet_size: u16,
    pub internal: u8,
}

assert_eq_size!(EndpointDescriptor, [u8; 7]);

#[repr(C)]
#[derive(Debug)]
pub struct SetupPacket {
    pub request_type: RequestType,
    pub request: u8,
    pub value: WordByte,
    pub index: WordByte,
    pub length: u16,
}

assert_eq_size!(SetupPacket, u64);

#[repr(transparent)]
#[derive(Debug, Clone, Copy)]
pub struct UsbHandle(u32);

impl UsbHandle {
    pub const fn null() -> UsbHandle {
        UsbHandle(0)
    }
}

pub type Callback = extern "C" fn(UsbHandle) -> i32;
pub type CallbackParameter = extern "C" fn(UsbHandle, u32) -> i32;
pub type EndpointHandler = extern "C" fn(UsbHandle, *mut u8, u32) -> i32;

#[repr(C)]
#[derive(Debug, Default)]
pub struct InitParameter {
    pub usb_reg_base: u32,
    pub mem_base: u32,
    pub mem_size: u32,
    pub max_num_ep: u8,
    pub padding: [u8; 3],
    pub reset_event: Option<Callback>,
    pub suspend_event: Option<Callback>,
    pub resume_event: Option<Callback>,
    pub reserved_sbz: Option<Callback>,
    pub sof_event: Option<CallbackParameter>,
    pub wakeup_config: Option<CallbackParameter>,
    pub power_event: Option<CallbackParameter>,
    pub error_event: Option<CallbackParameter>,
    pub configure_event: Option<Callback>,
    pub interface_event: Option<Callback>,
    pub feature_event: Option<Callback>,
    pub virt_to_phys: Option<fn(u32) -> u32>,
    pub cache_flush: Option<fn(*const u32, *const u32)>,
}

assert_eq_size!(InitParameter, [u8; 0x44]);

#[repr(C)]
#[derive(Debug)]
pub struct CoreApi {
    pub register_class_handler: extern "C" fn(UsbHandle, EndpointHandler, *mut u8) -> i32,
    pub register_ep_handler: extern "C" fn(UsbHandle, u32, EndpointHandler, *mut u8) -> i32,
    pub setup_stage: extern "C" fn(UsbHandle),
    pub data_in_stage: extern "C" fn(UsbHandle),
    pub data_out_stage: extern "C" fn(UsbHandle),
    pub status_in_stage: extern "C" fn(UsbHandle),
    pub status_out_stage: extern "C" fn(UsbHandle),
    pub stall_ep0: extern "C" fn(UsbHandle),
}

assert_eq_size!(CoreApi, [u8; 0x20]);

#[repr(C)]
#[derive(Debug, Default)]
pub struct CoreDescriptors {
    pub device_descriptors: Option<NonNull<DeviceDescriptor>>,
    pub string_descriptors: Option<NonNull<u8>>,
    pub full_speed_descriptors: Option<NonNull<u8>>,
    pub high_speed_descriptors: Option<NonNull<u8>>,
    pub device_qualifier: Option<NonNull<u8>>,
}

assert_eq_size!(CoreDescriptors, [u8; 0x14]);

#[repr(C)]
#[derive(Debug)]
pub struct HardwareApi {
    pub get_mem_size: extern "C" fn(*const InitParameter) -> u32,
    pub init: extern "C" fn(*mut UsbHandle, *const CoreDescriptors, *mut InitParameter) -> i32,
    pub connect: extern "C" fn(UsbHandle, u32),
    pub isr: extern "C" fn(UsbHandle),
    pub reset: extern "C" fn(UsbHandle),
    pub force_full_speed: extern "C" fn(UsbHandle, u32),
    pub wakeup_config: extern "C" fn(UsbHandle, u32),
    pub set_address: extern "C" fn(UsbHandle, u32),
    pub configure: extern "C" fn(UsbHandle, u32),
    pub configure_ep: extern "C" fn(UsbHandle, *const EndpointDescriptor),
    pub dir_ctrl_ep: extern "C" fn(UsbHandle, u32),
    pub enable_ep: extern "C" fn(UsbHandle, u32),
    pub disable_ep: extern "C" fn(UsbHandle, u32),
    pub reset_ep: extern "C" fn(UsbHandle, u32),
    pub set_stall_ep: extern "C" fn(UsbHandle, u32),
    pub clr_stall_ep: extern "C" fn(UsbHandle, u32),
    pub set_test_mode: extern "C" fn(UsbHandle, u8) -> i32,
    pub read_ep: extern "C" fn(UsbHandle, u32, *mut u8) -> u32,
    pub read_req_ep: extern "C" fn(UsbHandle, u32, *mut u8, u32) -> u32,
    pub read_setup_pkt: extern "C" fn(UsbHandle, u32, *mut u32) -> u32,
    pub write_ep: extern "C" fn(UsbHandle, u32, *const u8, u32) -> u32,
    pub wakeup: extern "C" fn(UsbHandle),
    pub enable_event: extern "C" fn(UsbHandle, u32, EventType, u32) -> i32,
}

assert_eq_size!(HardwareApi, [u8; 0x5c]);

#[repr(C)]
#[derive(Debug)]
pub struct DeviceDescriptor {
    pub length: u8,
    pub descriptor_type: u8,
    pub bcd_usb: u16,
    pub device_class: u8,
    pub device_sub_class: u8,
    pub device_protocol: u8,
    pub max_packet_size: u8,
    pub id_vendor: u16,
    pub id_product: u16,
    pub bcd_device: u16,
    pub manufacturer_str_index: u8,
    pub product_str_index: u8,
    pub serial_number_str_index: u8,
    pub num_configurations: u8,
}

assert_eq_size!(DeviceDescriptor, [u8; 0x12]);

bitfield! {
  pub struct RequestType(u8);
  impl Debug;
  pub from into Recipient, recipient, set_recipient: 4, 0;
  pub from into RequestCategory, request_category, _ : 6, 5;
  pub from into RequestDirection, direction, _: 7;
}

#[repr(C)]
pub struct HidApi {
    pub get_mem_size: extern "C" fn(*const HidInitParameter) -> u32,
    // Mutable reference to init parameter is very much *on purpose*. The
    // mem_base and mem_size will be updated after the function returns to point
    // to the rest of the available memory.
    pub init: extern "C" fn(UsbHandle, *mut HidInitParameter) -> i32,
}
assert_eq_size!(HidApi, [u8; 8]);

#[repr(transparent)]
pub struct HidHandle(u32);

#[repr(C)]
#[derive(Debug, Default)]
pub struct HidInitParameter<'a> {
    pub mem_base: u32,
    pub mem_size: u32,
    pub max_reports: u8,
    pub pad: [u8; 3],
    pub intf_desc: Option<NonNull<u8>>,
    pub report_data: Option<&'a HidReport>,
    pub get_report: Option<extern "C" fn(HidHandle, *const SetupPacket, *mut *mut u8, *mut u16) -> i32>,
    pub set_report: Option<extern "C" fn(HidHandle, *const SetupPacket, *const *const u8, u16) -> i32>,
    pub get_phys_desc: Option<extern "C" fn(HidHandle, *const SetupPacket, *mut *mut u8, *mut u16) -> i32>,
    pub set_idle: Option<extern "C" fn(HidHandle, *const SetupPacket, u8) -> i32>,
    pub set_protocol: Option<extern "C" fn(HidHandle, *const SetupPacket, u8) -> i32>,
    pub ep_in_hdlr: Option<extern "C" fn(EndpointHandler) -> i32>,
    pub ep_out_hdlr: Option<extern "C" fn(EndpointHandler) -> i32>,
    pub get_report_desc: Option<extern "C" fn(HidHandle, *const SetupPacket, *mut *mut u8, *mut u16) -> i32>,
    pub ep0_hdlr: Option<EndpointHandler>,
}
assert_eq_size!(HidInitParameter, [u8; 0x38]);

#[repr(C)]
#[derive(Debug, Default)]
pub struct HidReport {
    pub len: u16,
    pub idle_time: u8,
    pub pad: u8,
    pub desc: Option<NonNull<u8>>
}
assert_eq_size!(HidReport, [u8; 8]);

#[repr(C)]
pub struct CdcControl {
    // Technically USB_CORE_CTRL
    pub usb_ctrl: UsbHandle,
    pub notice_buf: [u8; 12],
    pub line_coding: CdcLineCoding,
    pub pad0: u8,

    pub cif_num: u8,
    pub dif_num: u8,
    pub epin_num: u8,
    pub epout_num: u8,
    pub epint_num: u8,
    pub pad: [u8; 3],

    // User-defined functions
    pub send_encps_cmd: Option<extern "C" fn(CdcHandle, *mut u8, u16) -> i32>,
    pub get_encps_resp: Option<extern "C" fn(CdcHandle, *mut *mut u8, *mut u16) -> i32>,
    pub set_comm_feature: Option<extern "C" fn(CdcHandle, u16, *mut u8, u16) -> i32>,
    pub get_comm_feature: Option<extern "C" fn(CdcHandle, u16, *mut *mut u8, *mut u16) -> i32>,
    pub clr_comm_feature: Option<extern "C" fn(CdcHandle, u16) -> i32>,
    pub set_ctrl_line_state: Option<extern "C" fn(CdcHandle, u16) -> i32>,
    pub send_break: Option<extern "C" fn(CdcHandle, u16) -> i32>,
    pub set_line_code: Option<extern "C" fn(CdcHandle, &mut CdcLineCoding) -> i32>,

    /* virtual functions */
    pub cic_get_request: Option<extern "C" fn(CdcHandle, *const SetupPacket, *mut *mut u8, *mut u16) -> i32>,
    pub cic_set_request: Option<extern "C" fn(CdcHandle, *const SetupPacket, *const *const u8, u16) -> i32>,
}

#[repr(transparent)]
#[derive(PartialEq, Eq)]
pub struct CdcHandle(*mut CdcControl);

impl CdcHandle {
    pub const fn null() -> CdcHandle {
        CdcHandle(core::ptr::null_mut())
    }

    pub unsafe fn inner_ctrl(&mut self) -> &mut CdcControl {
        &mut *self.0
    }
}

#[repr(C)]
#[derive(Default)]
pub struct CdcInitParameter {
    pub mem_base: u32,
    pub mem_size: u32,
    pub cif_intf_desc: Option<NonNull<u8>>,
    pub dif_intf_desc: Option<NonNull<u8>>,
    pub cic_get_request: Option<extern "C" fn(CdcHandle, *const SetupPacket, *mut *mut u8, *mut u16) -> i32>,
    pub cic_set_request: Option<extern "C" fn(CdcHandle, *const SetupPacket, *const *const u8, u16) -> i32>,
    pub cdc_bulkin_hdlr: Option<extern "C" fn(UsbHandle, *mut (), u32) -> i32>,
    pub cdc_bulkout_hdlr: Option<extern "C" fn(UsbHandle, *mut (), u32) -> i32>,
    pub send_encps_cmd: Option<extern "C" fn(CdcHandle, *mut u8, u16) -> i32>,
    pub get_encps_resp: Option<extern "C" fn(CdcHandle, *mut *mut u8, *mut u16) -> i32>,
    pub set_comm_feature: Option<extern "C" fn(CdcHandle, u16, *mut u8, u16) -> i32>,
    pub get_comm_feature: Option<extern "C" fn(CdcHandle, u16, *mut *mut u8, *mut u16) -> i32>,
    pub clr_comm_feature: Option<extern "C" fn(CdcHandle, u16) -> i32>,
    pub set_ctrl_line_state: Option<extern "C" fn(CdcHandle, u16) -> i32>,
    pub send_break: Option<extern "C" fn(CdcHandle, u16) -> i32>,
    pub set_line_code: Option<extern "C" fn(CdcHandle, &mut CdcLineCoding) -> i32>,
    pub cdc_interruptep_hdlr: Option<extern "C" fn(UsbHandle, *mut (), u32) -> i32>,
    pub cdc_ep0_hdlr: Option<extern "C" fn(UsbHandle, *mut (), u32) -> i32>,
}
assert_eq_size!(CdcInitParameter, [u8; 0x48]);

#[repr(C)]
pub struct CdcApi {
    pub get_mem_size: extern "C" fn(*const CdcInitParameter) -> u32,
    pub init: extern "C" fn(UsbHandle, *mut CdcInitParameter, *mut CdcHandle) -> i32,
    pub send_notification: extern "C" fn(CdcHandle, u8, u16) -> i32,
}
assert_eq_size!(CdcApi, [u8; 12]);


#[repr(C)]
pub struct CdcLineCoding {
    pub dte_rate: u32,
    pub char_format: u8,
    pub parity_type: u8,
    pub data_bits: u8
}