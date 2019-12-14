use arrayvec::ArrayVec;

use lpc11uxx::interrupt;

use crate::rom::usbd;
use crate::rom::RomDriver;

use core::cell::Cell;

use usb_device::bus::{PollResult, UsbBusAllocator};
use usb_device::device::UsbDevice;
use usb_device::endpoint::{EndpointAddress, EndpointType};
use usb_device::{Result, UsbDirection, UsbError};

use crate::rom::usbd::{CoreDescriptors, DeviceDescriptor, EventType, InitParameter};

use cortex_m as cpu;

use cpu::peripheral::NVIC;
use cpu::Peripherals as ArmPeripherals;

use lpc11uxx::Interrupt;
use lpc11uxx::Peripherals;

pub struct Endpoint {
    address: EndpointAddress,
    endpoint_type: EndpointType,
    event_type: Cell<Option<EventType>>,
    is_stalled: Cell<bool>,
}

// SAFETY: We only have one core so no issues here :)
unsafe impl Sync for Endpoint {}

pub static mut USB_BUS_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
pub static mut USB_DEVICE: Option<UsbDevice<'static, UsbBus>> = None;

// FIXME: REMOVE ME
pub static mut USB_HANDLE: Option<usbd::Handle> = None;

impl Endpoint {
    pub fn new(address: EndpointAddress, endpoint_type: EndpointType) -> Self {
        Endpoint {
            address,
            endpoint_type,
            event_type: Cell::new(None),
            is_stalled: Cell::new(false),
        }
    }

    pub fn get_address(&self) -> EndpointAddress {
        self.address
    }

    pub fn get_poll_data(&self) -> PollResult {
        let mut ep_out: u16 = 0;
        let mut ep_in_complete: u16 = 0;
        let mut ep_setup: u16 = 0;

        match self.event_type.get() {
            Some(EventType::In) => ep_in_complete = 1 << self.address.index(),
            Some(EventType::Out) => ep_out = 1 << self.address.index(),
            Some(EventType::Setup) => ep_setup = 1 << self.address.index(),
            Some(EventType::Reset) => {
                self.event_type.set(None);
                return PollResult::Reset;
            }
            _ => return PollResult::None,
        };

        PollResult::Data {
            ep_out,
            ep_in_complete,
            ep_setup,
        }
    }

    pub fn read(
        &self,
        usb_api: &usbd::UsbRomDriver,
        usb_handle: usbd::Handle,
        buf: &mut [u8],
    ) -> Result<usize> {
        let res = match self.event_type.get() {
            // TODO: THIS IS TOTALLY UNSAFE
            Some(EventType::Setup) => Ok((usb_api.hw().read_setup_pkt)(
                usb_handle,
                u8::from(self.get_address()) as u32,
                buf as *mut _ as *mut u32,
            )),
            Some(EventType::Out) => Ok((usb_api.hw().read_req_ep)(
                usb_handle,
                u8::from(self.get_address()) as u32,
                buf as *mut _ as *mut u8,
                buf.len() as u32,
            )),
            Some(_) => unimplemented!(),
            None => Err(UsbError::WouldBlock),
        };

        self.event_type.set(None);

        res.map(|value| value as usize)
    }

    pub fn write(
        &self,
        usb_api: &usbd::UsbRomDriver,
        usb_handle: usbd::Handle,
        buf: &[u8],
    ) -> Result<usize> {
        let value = (usb_api.hw().write_ep)(
            usb_handle,
            u8::from(self.get_address()) as u32,
            buf as *const _ as *const u8,
            buf.len() as u32,
        );

        self.event_type.set(None);

        Ok(value as usize)
    }

    pub fn is_stalled(&self) -> bool {
        self.is_stalled.get()
    }

    pub fn set_stalled(
        &self,
        usb_api: &usbd::UsbRomDriver,
        usb_handle: usbd::Handle,
        is_stalled: bool,
    ) {
        if self.is_stalled() != is_stalled {
            if is_stalled {
                (usb_api.hw().set_stall_ep)(usb_handle, u8::from(self.get_address()) as u32);
            } else {
                (usb_api.hw().clr_stall_ep)(usb_handle, u8::from(self.get_address()) as u32);
            }
        }

        self.is_stalled.set(is_stalled);
    }

    extern "C" fn handle_event(_: usbd::Handle, instance: *mut u8, raw_event_type: u32) -> i32 {
        let endpoint = unsafe { &mut *(instance as *mut Endpoint) };

        let event_type = EventType::from(raw_event_type as u8);

        match event_type {
            EventType::In | EventType::Out | EventType::Setup => {
                endpoint.event_type.set(Some(event_type));
            }
            _ => {}
        }

        // We don't handle anything, this permits for the ClassHandler of both control interface to be called if needed.
        0x00040002
    }

    pub fn register_handler(&mut self, usb_api: &usbd::UsbRomDriver, usb_handle: usbd::Handle) {
        let result = if self.address.index() == 0 {
            (usb_api.core().register_class_handler)(
                usb_handle,
                Self::handle_event,
                self as *mut _ as *mut u8,
            )
        } else {
            (usb_api.core().register_ep_handler)(
                usb_handle,
                self.address.index() as u32,
                Self::handle_event,
                self as *mut _ as *mut u8,
            )
        };

        if result != 0 {
            panic!("Error while registering handler");
        }
    }
}

pub struct UsbBus {
    usb_api: &'static usbd::UsbRomDriver,
    endpoints: ArrayVec<[Endpoint; usbd::MAX_EP_COUNT]>,
    max_endpoint: usize,
    usb_handle: Option<usbd::Handle>,
}

impl UsbBus {
    pub fn new() -> UsbBusAllocator<Self> {
        let bus = UsbBus {
            usb_api: RomDriver::get().usb_api(),
            endpoints: ArrayVec::new(),
            max_endpoint: 0,
            usb_handle: None,
        };

        UsbBusAllocator::new(bus)
    }

    fn get_usb_handle(&self) -> Result<usbd::Handle> {
        self.usb_handle.ok_or(UsbError::InvalidState)
    }

    fn get_endpoint(&self, address: EndpointAddress) -> Result<&Endpoint> {
        for endpoint in &self.endpoints {
            if endpoint.get_address() == address {
                return Ok(endpoint);
            }
        }

        Err(UsbError::InvalidEndpoint)
    }
}

impl usb_device::bus::UsbBus for UsbBus {
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> Result<EndpointAddress> {
        if self.endpoints.is_full() {
            return Err(UsbError::EndpointOverflow);
        }

        let ep_addr = ep_addr.unwrap_or(EndpointAddress::from_parts(self.endpoints.len(), ep_dir));
        let mut endpoint = Endpoint::new(ep_addr, ep_type);

        self.endpoints.push(endpoint);

        Ok(ep_addr)
    }

    fn enable(&mut self) {
        self.max_endpoint = self.endpoints.len() - 1;

        let mut configuration: InitParameter = InitParameter::default();

        configuration.usb_reg_base = 0x40080000;
        configuration.mem_base = 0x20004000;
        configuration.mem_size = 0x0800;
        configuration.max_num_ep = 3 + 1;

        let core_desc = CoreDescriptors {
            device_descriptors: core::ptr::null(),
            string_descriptors: core::ptr::null(),
            full_speed_descriptors: core::ptr::null(),
            high_speed_descriptors: core::ptr::null(),
            device_qualifier: core::ptr::null(),
        };

        let mut usb_handle: usbd::Handle = 0;

        let res = (self.usb_api.hw().init)(
            &mut usb_handle as *mut _,
            &core_desc as *const _,
            &configuration,
        );

        self.usb_handle = Some(usb_handle);

        unsafe {
            USB_HANDLE = Some(usb_handle);
        }

        if res != 0 {
            panic!("Error while init");
        }

        for endpoint in &mut self.endpoints {
            endpoint.register_handler(self.usb_api, usb_handle);
        }

        unsafe {
            let mut peripheral = ArmPeripherals::steal();
            peripheral.NVIC.set_priority(Interrupt::USB_IRQ, 1);
            NVIC::unmask(Interrupt::USB_IRQ);
        }

        (self.usb_api.hw().connect)(usb_handle, 1);
    }

    fn reset(&self) {
        if let Some(usb_handle) = self.usb_handle {
            (self.usb_api.hw().reset)(usb_handle);
        }
    }

    fn poll(&self) -> PollResult {
        let mut ep_out: u16 = 0;
        let mut ep_in_complete: u16 = 0;
        let mut ep_setup: u16 = 0;

        for endpoint in &self.endpoints {
            match endpoint.get_poll_data() {
                PollResult::Data {
                    ep_out: out,
                    ep_in_complete: in_complete,
                    ep_setup: setup,
                } => {
                    ep_out += out;
                    ep_in_complete += in_complete;
                    ep_setup += setup;
                }
                PollResult::Reset => return PollResult::Reset,
                _ => {}
            }
        }

        if ep_in_complete == 0 && ep_out == 0 && ep_setup == 0 {
            return PollResult::None;
        }

        PollResult::Data {
            ep_out,
            ep_in_complete,
            ep_setup,
        }
    }

    fn set_device_address(&self, addr: u8) {
        if let Some(usb_handle) = self.usb_handle {
            (self.usb_api.hw().set_address)(usb_handle, u32::from(addr));
        }
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        if !ep_addr.is_in() {
            return Err(UsbError::InvalidEndpoint);
        }

        match self.get_endpoint(ep_addr) {
            Ok(endpoint) => endpoint.write(self.usb_api, self.get_usb_handle()?, buf),
            Err(error) => Err(error),
        }
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        if !ep_addr.is_out() {
            return Err(UsbError::InvalidEndpoint);
        }

        match self.get_endpoint(ep_addr) {
            Ok(endpoint) => {
                if self.get_usb_handle().is_err() {
                    unimplemented!()
                }
                endpoint.read(self.usb_api, self.get_usb_handle()?, buf)
            }
            Err(error) => Err(error),
        }
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        match self.get_endpoint(ep_addr) {
            Ok(endpoint) => endpoint.is_stalled(),
            Err(_) => false,
        }
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        if let Ok(usb_handle) = self.get_usb_handle() {
            match self.get_endpoint(ep_addr) {
                Ok(endpoint) => endpoint.set_stalled(self.usb_api, usb_handle, stalled),
                Err(_) => {}
            }
        }
    }

    fn suspend(&self) {
        unimplemented!()
    }

    fn resume(&self) {
        unimplemented!()
    }
}

#[interrupt]
fn USB_IRQ() {
    unsafe {
        let peripheral = Peripherals::steal();

        if peripheral.USB.devcmdstat.read().setup().bit() {
            let epliststart_address = peripheral.USB.epliststart.read().bits() as *mut u32;

            *epliststart_address &= !(1 << 29);
            *(epliststart_address.offset(2)) &= !(1 << 29);
        }

        if let Some(usb_handle) = USB_HANDLE {
            (RomDriver::get().usb_api().hw().isr)(usb_handle);
        }
    }
}
