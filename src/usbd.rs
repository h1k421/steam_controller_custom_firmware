use arrayvec::ArrayVec;

use lpc11uxx::interrupt;

use crate::rom::usbd;
use crate::rom::RomDriver;

use core::cell::Cell;

use usb_device::bus::{PollResult, UsbBusAllocator};
use usb_device::device::UsbDevice;
use usb_device::endpoint::{EndpointAddress, EndpointType};
use usb_device::{Result, UsbDirection, UsbError};

use crate::rom::usbd::{CoreDescriptors, EventType, InitParameter};

use cortex_m as cpu;

use cpu::peripheral::NVIC;
use cpu::Peripherals as ArmPeripherals;

use lpc11uxx::Interrupt;
use lpc11uxx::Peripherals;

#[derive(Default, Clone)]
pub struct Endpoint {
    ep_type: Option<EndpointType>,
    ep_dir: Option<UsbDirection>,
    index: u8,
    event_type: Option<EventType>,
    is_stalled: Cell<bool>,
}

// SAFETY: We only have one core so no issues here :)
unsafe impl Sync for Endpoint {}

pub static mut USB_BUS_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
pub static mut USB_DEVICE: Option<UsbDevice<'static, UsbBus>> = None;
pub static mut USB_HANDLE: Option<usbd::Handle> = None;

impl Endpoint {
    pub fn new(index: u8) -> Self {
        Endpoint {
            ep_type: None,
            ep_dir: None,
            index,
            event_type: None,
            is_stalled: Cell::new(false),
        }
    }

    pub fn ep_type(&self) -> Option<EndpointType> {
        self.ep_type
    }

    pub fn set_ep_type(&mut self, ep_type: EndpointType) {
        self.ep_type = Some(ep_type);
    }

    pub fn set_ep_dir(&mut self, ep_dir: UsbDirection) {
        self.ep_dir = Some(ep_dir);
    }

    pub fn get_address(&self) -> EndpointAddress {
        EndpointAddress::from_parts(self.index as usize, self.ep_dir.unwrap())
    }

    pub fn get_poll_data(&self) -> PollResult {
        let mut ep_out: u16 = 0;
        let mut ep_in_complete: u16 = 0;
        let mut ep_setup: u16 = 0;

        match self.event_type {
            Some(EventType::In) => ep_in_complete = 1 << self.index,
            Some(EventType::Out) => ep_out = 1 << self.index,
            Some(EventType::Setup) => ep_setup = 1 << self.index,
            _ => return PollResult::None
        };

        PollResult::Data { ep_out, ep_in_complete, ep_setup }
    }

    pub fn read(
        &self,
        usb_api: &usbd::UsbRomDriver,
        usb_handle: usbd::Handle,
        buf: &mut [u8],
    ) -> Result<usize> {
        let value = if let Some(EventType::Setup) = self.event_type {
            unimplemented!()
        } else {
            (usb_api.hw().read_req_ep)(
                usb_handle,
                u8::from(self.get_address()) as u32,
                buf as *mut _ as *mut u8,
                buf.len() as u32,
            )
        };

        Ok(value as usize)
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
                endpoint.event_type = Some(event_type);
                unimplemented!()
            }
            EventType::Reset => {
            }
            _ => {
                unimplemented!();
            }
        }
        0
    }

    pub fn register_handler(&mut self, usb_api: &usbd::UsbRomDriver, usb_handle: usbd::Handle) {
        let result = if self.index == 0 {
            (usb_api.core().register_class_handler)(
                usb_handle,
                Self::handle_event,
                self as *mut _ as *mut u8,
            )
        } else {
            (usb_api.core().register_ep_handler)(
                usb_handle,
                u32::from(self.index),
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
        let mut endpoints = ArrayVec::new();

        for i in 0..usbd::MAX_EP_COUNT {
            endpoints.push(Endpoint::new(i as u8));
        }

        let bus = UsbBus {
            usb_api: RomDriver::get().usb_api(),
            endpoints,
            max_endpoint: 0,
            usb_handle: None,
        };

        UsbBusAllocator::new(bus)
    }

    fn get_usb_handle(&self) -> Result<usbd::Handle> {
        self.usb_handle.ok_or(UsbError::InvalidState)
    }

    /*pub fn handle_interrupt(&self) {
        if let Some(usb_handle) = self.usb_handle {
            (self.usb_api.hw().isr)(usb_handle);
        }
    }*/
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
        for index in ep_addr
            .map(|a| a.index()..a.index() + 1)
            .unwrap_or(1..usbd::MAX_EP_COUNT)
        {
            let ep = &mut self.endpoints[index];

            match ep.ep_type() {
                None => {
                    ep.set_ep_type(ep_type);
                    ep.set_ep_dir(ep_dir);
                }
                Some(t) if t != ep_type => {
                    continue;
                }
                _ => {}
            };

            return Ok(ep.get_address());
        }

        Err(match ep_addr {
            Some(_) => UsbError::InvalidEndpoint,
            None => UsbError::EndpointOverflow,
        })
    }

    fn enable(&mut self) {
        let mut max = 0;
        for (index, ep) in self.endpoints.iter().enumerate() {
            if ep.ep_type().is_some() {
                max = index;
            }
        }

        self.max_endpoint = max;

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

        unsafe {
            USB_HANDLE = Some(usb_handle);
        }

        if res != 0 {
            panic!("Error while init");
        }

        for i in 0..=max {
            self.endpoints[i].register_handler(self.usb_api, usb_handle);
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

        for i in 0..=self.max_endpoint {
            match self.endpoints[i].get_poll_data() {
                PollResult::Data { ep_out: out, ep_in_complete: in_complete, ep_setup: setup } => {
                    ep_out += out;
                    ep_in_complete += in_complete;
                    ep_setup += setup;
                }
                _ => {}
            }
        }

        if ep_in_complete == 0 && ep_out == 0 && ep_setup == 0 {
            return PollResult::None;
        }

        PollResult::Data { ep_out, ep_in_complete, ep_setup }        
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

        if ep_addr.index() > self.max_endpoint {
            return Err(UsbError::InvalidEndpoint);
        }

        self.endpoints[ep_addr.index()].write(self.usb_api, self.get_usb_handle()?, buf)
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        if !ep_addr.is_out() {
            return Err(UsbError::InvalidEndpoint);
        }

        if ep_addr.index() > self.max_endpoint {
            return Err(UsbError::InvalidEndpoint);
        }

        self.endpoints[ep_addr.index()].read(self.usb_api, self.get_usb_handle()?, buf)
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        if ep_addr.index() > self.endpoints.len() {
            return false;
        }

        self.endpoints[ep_addr.index()].is_stalled()
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        if ep_addr.index() > self.endpoints.len() {
            return;
        }

        if let Ok(ubs_handle) = self.get_usb_handle() {
            self.endpoints[ep_addr.index()].set_stalled(self.usb_api, ubs_handle, stalled);
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
