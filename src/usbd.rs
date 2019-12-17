use arrayvec::ArrayVec;

use core::cell::Cell;

use usb_device::bus::{PollResult, UsbBusAllocator};
use usb_device::device::UsbDevice;
use usb_device::endpoint::{EndpointAddress, EndpointType};
use usb_device::{Result, UsbDirection, UsbError};

use cortex_m::peripheral::NVIC;
use lpc11uxx::{CorePeripherals, Interrupt, Peripherals};

use bitfield::bitfield;

pub const MAX_IF_COUNT: usize = 8;
pub const MAX_EP_LOGICAL_COUNT: usize = 5;
pub const MAX_EP_PHYSICAL_COUNT: usize = 10;
pub const MAX_PACKET0: usize = 0x40;
pub const FS_MAX_BULK_PACKET: usize = 0x40;
pub const HS_MAX_BULK_PACKET: usize = 0x200;

pub struct Endpoint {
    address: EndpointAddress,
    endpoint_type: EndpointType,
    endpoint_entry: Cell<*mut HardwareEndpoint>,
    registers: lpc11uxx::USB,
    // TODO: remove options and use MaybeUninit?
    buffer_address: Cell<*mut u8>,
    buffer_size: Cell<usize>,
    is_active: Cell<bool>,
}

// SAFETY: We only have one core so no issues here :)
unsafe impl Sync for Endpoint {}

pub static mut USB_BUS_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
pub static mut USB_DEVICE: Option<UsbDevice<'static, UsbBus>> = None;

impl Endpoint {
    pub fn new(
        address: EndpointAddress,
        endpoint_type: EndpointType,
        registers: lpc11uxx::USB,
    ) -> Self {
        Endpoint {
            address,
            endpoint_type,
            endpoint_entry: Cell::new(core::ptr::null_mut()),
            registers,
            buffer_address: Cell::new(core::ptr::null_mut()),
            buffer_size: Cell::new(0),
            is_active: Cell::new(false),
        }
    }

    pub fn get_address(&self) -> EndpointAddress {
        self.address
    }

    pub unsafe fn initialize(&self, usb_memory_base: *mut u8) {
        let endpoint_list = usb_memory_base as *mut HardwareEndpoint;
        let endpoint_buffer = usb_memory_base.offset(0x100);
        let address = self.address;

        let buffer_index = UsbBus::get_buffer_offset_with_address(address);

        let target_buffer_address = endpoint_buffer.offset(MAX_PACKET0 as isize * buffer_index);

        (*endpoint_list.offset(buffer_index)).set_address(target_buffer_address as u32);
        (*endpoint_list.offset(buffer_index)).set_size(MAX_PACKET0 as u32);

        // EP0 is always active
        if address.index() != 0 {
            (*endpoint_list.offset(buffer_index)).set_disabled(true);
        }

        // Seems like even in single buffering mode you need this???
        // This is fine as EP0 OUT buffer 1 is special and used for setup...
        let target_double_buffer_address =
            endpoint_buffer.offset(MAX_PACKET0 as isize * buffer_index + MAX_PACKET0 as isize);
        (*endpoint_list.offset(buffer_index + 1)).set_address(target_double_buffer_address as u32);
        (*endpoint_list.offset(buffer_index + 1)).set_size(MAX_PACKET0 as u32);

        // EP0 buffer 1 is always active and is special (OUT: setup, IN: reserved)
        if address.index() != 0 {
            (*endpoint_list.offset(buffer_index + 1)).set_disabled(true);
        }

        self.endpoint_entry.set(endpoint_list.offset(buffer_index));
        self.set_buffer(target_buffer_address, MAX_PACKET0);

        self.is_active.set(false);
    }

    pub fn get_buffer(&mut self) -> &mut [u8] {
        let buffer_address = self.buffer_address.get();
        unsafe { core::slice::from_raw_parts_mut(buffer_address, self.buffer_size.get()) }
    }

    pub fn set_buffer(&self, buffer_address: *mut u8, buffer_size: usize) {
        self.buffer_address.set(buffer_address);
        self.buffer_size.set(buffer_size);
    }

    fn disable(&self) {
        let mask_to_apply = 1 << UsbBus::get_buffer_offset_with_address(self.address);

        self.registers
            .epskip
            .modify(|_, writer| unsafe { writer.skip().bits(mask_to_apply) });

        while self.registers.epskip.read().bits() & mask_to_apply != 0 {}

        // Clear ep interrupt
        self.registers
            .intstat
            .write(|writer| unsafe { writer.bits(mask_to_apply) });
    }

    pub fn is_stalled(&self) -> bool {
        let endpoint_entry = self.endpoint_entry.get();

        unsafe { (*endpoint_entry).is_stalled() }
    }

    pub fn set_stalled(&self, stalled: bool) {
        let endpoint_entry = self.endpoint_entry.get();
        let double_buffer_mask = 1 << UsbBus::get_buffer_offset_with_address(self.address);

        if !stalled {
            if self.address.index() == 0 {
                // If we are on EP0, we just need to clear the stall bit.
                unsafe { (*endpoint_entry).set_stalled(false) };
            } else {
                // If we are on a non zero endpoint, we need to clear the stall bit on our double buffer and choose the appropriate one to reset.

                unsafe {
                    (*endpoint_entry).set_stalled(false);
                    (*endpoint_entry.offset(1)).set_stalled(false);
                }

                let target_endpoint_entry;

                if (self.registers.epinuse.read().bits() & double_buffer_mask) != 0 {
                    // Secondary buffer in use.
                    target_endpoint_entry = unsafe { endpoint_entry.offset(1) };
                } else {
                    // Primary buffer in use.
                    target_endpoint_entry = endpoint_entry;
                }

                unsafe { (*target_endpoint_entry).set_reset(true) };

                if self.address.direction() == UsbDirection::In {
                    // if the IN EP was set active, reflect it in the hardware endpoint entry.
                    if self.is_active.get() {
                        unsafe { (*target_endpoint_entry).set_active(true) };

                        self.is_active.set(false)
                    }
                } else {
                    // For OUT EP, we reactivate and reset the size to the previous value.
                    unsafe { (*target_endpoint_entry).set_active(true) };
                    unsafe { (*target_endpoint_entry).set_size(self.buffer_size.get() as u32) };
                }
            }
        } else {
            // If the active bit is set, we need to get ride of it before setting the stalled bit
            let is_active = unsafe { (*endpoint_entry).is_active() };
            if is_active {
                self.disable();
            }

            // Set stall bit
            unsafe { (*endpoint_entry).set_stalled(true) };

            // If we aren't on EP0 and double buffering is active, we need to set the stall bit on the double buffer.
            if self.address.index() != 0
                && (self.registers.epbufcfg.read().bits() & double_buffer_mask) != 0
            {
                // If the active bit is set, we need to get ride of it before setting the stalled bit
                let is_active = unsafe { (*endpoint_entry.offset(1)).is_active() };
                if is_active {
                    self.disable();
                }

                // Set stall bit
                unsafe { (*endpoint_entry.offset(0)).set_stalled(true) };
            }
        }
    }

    pub fn write(&self, buf: &[u8]) -> Result<usize> {
        unimplemented!()
    }

    pub fn read(&self, buf: &mut [u8]) -> Result<usize> {
        unimplemented!()
    }
}

// Control structure that MUST be on the USB RAM.
#[repr(C)]
pub struct UsbContext {}

bitfield! {
  pub struct HardwareEndpoint(u32);
  impl Debug;
  pub address, set_address: 15, 0;
  pub size, set_size : 24, 16;
  pub is_iso_type, set_iso_type: 25;
  // bit 26: ???
  pub is_reset, set_reset: 27;
  pub is_stalled, set_stalled: 28;
  pub is_disabled, set_disabled: 29;
  pub is_active, set_active: 30;
  //pub cs, set_cs: 30, 25;
}

pub struct UsbBus {
    endpoints: ArrayVec<[Endpoint; MAX_EP_PHYSICAL_COUNT]>,
    registers: lpc11uxx::USB,
    usb_memory_base: *mut u8,
    max_endpoint: usize,
}

unsafe impl Sync for UsbBus {}

impl UsbBus {
    pub fn new() -> UsbBusAllocator<Self> {
        let bus = UsbBus {
            endpoints: ArrayVec::new(),
            registers: unsafe { Peripherals::steal().USB },
            usb_memory_base: 0x20004000 as *mut u8,
            max_endpoint: 0,
        };

        UsbBusAllocator::new(bus)
    }

    fn get_endpoint(&self, address: EndpointAddress) -> Result<&Endpoint> {
        for endpoint in &self.endpoints {
            let ep_addr = endpoint.get_address();

            if address.index() == ep_addr.index() && address.direction() == ep_addr.direction() {
                return Ok(endpoint);
            }
        }

        Err(UsbError::InvalidEndpoint)
    }

    fn get_buffer_offset_with_address(endpoint_address: EndpointAddress) -> isize {
        let value = (endpoint_address.index()
            + if endpoint_address.direction() == UsbDirection::In {
                1
            } else {
                0
            }) as isize;

        value * 2
    }

    unsafe fn init_endpoints(&self) {
        self.registers
            .epliststart
            .write(|writer| writer.bits(self.usb_memory_base as u32));
        self.registers
            .databufstart
            .write(|writer| writer.bits((self.usb_memory_base.offset(0x100) as u32) & 0xFFC00000));

        for endpoint in &self.endpoints {
            endpoint.initialize(self.usb_memory_base);
        }
    }

    fn connect(&self) {
        self.registers
            .devcmdstat
            .modify(|_, writer| writer.dcon().set_bit())
    }

    fn disconnect(&self) {
        self.registers
            .devcmdstat
            .modify(|_, writer| writer.dcon().clear_bit())
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
        let endpoint = Endpoint::new(ep_addr, ep_type, unsafe { Peripherals::steal().USB });

        self.endpoints.push(endpoint);

        Ok(ep_addr)
    }

    fn enable(&mut self) {
        self.max_endpoint = self.endpoints.len() - 1;

        // TODO: move the usb clock init here?

        // Enable USB IRQ
        unsafe {
            let mut peripheral = CorePeripherals::steal();
            peripheral.NVIC.set_priority(Interrupt::USB_IRQ, 1);
            NVIC::unmask(Interrupt::USB_IRQ);
        }

        self.reset();
        self.connect();
    }

    fn reset(&self) {
        unsafe {
            self.init_endpoints();
        }

        // Clear EP usage registers
        self.registers
            .epinuse
            .write(|writer| unsafe { writer.buf().bits(0) });

        self.registers
            .epskip
            .write(|writer| unsafe { writer.skip().bits(0) });

        self.registers
            .devcmdstat
            .modify(|_, writer| writer.dev_en().set_bit());

        // Clear all EP interrupts, device status, and SOF interrupts.
        self.registers.intstat.write(|writer| {
            writer
                .ep0out()
                .set_bit()
                .ep0in()
                .set_bit()
                .ep1out()
                .set_bit()
                .ep1in()
                .set_bit()
                .ep2out()
                .set_bit()
                .ep2in()
                .set_bit()
                .ep3out()
                .set_bit()
                .ep3in()
                .set_bit()
                .ep4out()
                .set_bit()
                .ep4in()
                .set_bit()
                .frame_int()
                .set_bit()
                .dev_int()
                .set_bit()
        });

        self.registers.inten.write(|writer| unsafe {
            writer
                .ep_int_en()
                .bits(0x3FF)
                .frame_int_en()
                .set_bit()
                .dev_int_en()
                .set_bit()
        });
    }

    fn poll(&self) -> PollResult {
        unimplemented!()
    }

    fn set_device_address(&self, addr: u8) {
        // Clear device address.
        self.registers
            .devcmdstat
            .modify(|_, writer| unsafe { writer.dev_addr().bits(0x0) });

        // Set device address and enable.
        self.registers
            .devcmdstat
            .modify(|_, writer| unsafe { writer.dev_addr().bits(addr).dev_en().set_bit() });
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        if !ep_addr.is_in() {
            return Err(UsbError::InvalidEndpoint);
        }

        match self.get_endpoint(ep_addr) {
            Ok(endpoint) => endpoint.write(buf),
            Err(error) => Err(error),
        }
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        if !ep_addr.is_out() {
            return Err(UsbError::InvalidEndpoint);
        }

        match self.get_endpoint(ep_addr) {
            Ok(endpoint) => endpoint.read(buf),
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
        match self.get_endpoint(ep_addr) {
            Ok(endpoint) => endpoint.set_stalled(stalled),
            Err(_) => {}
        }
    }

    fn suspend(&self) {
        unimplemented!()
    }

    fn resume(&self) {
        unimplemented!()
    }
}
