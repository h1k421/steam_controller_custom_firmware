use arrayvec::ArrayVec;

use core::cell::{Cell, RefCell};
use core::ops::DerefMut;

use usb_device::bus::{PollResult, UsbBusAllocator};
use usb_device::device::UsbDevice;
use usb_device::endpoint::{EndpointAddress, EndpointType};
use usb_device::{Result, UsbDirection, UsbError};

use cortex_m::asm;
use cortex_m::peripheral::NVIC;
use lpc11uxx::interrupt;
use lpc11uxx::{CorePeripherals, Interrupt, Peripherals};

use bitfield::bitfield;

pub const MAX_IF_COUNT: usize = 8;
pub const MAX_EP_LOGICAL_COUNT: usize = 5;
pub const MAX_EP_PHYSICAL_COUNT: usize = 10;
pub const MAX_PACKET0: usize = 0x40;
pub const FS_MAX_BULK_PACKET: usize = 0x40;
pub const HS_MAX_BULK_PACKET: usize = 0x200;

pub struct EndpointBuffer {
    endpoint_entry: *mut HardwareEndpoint,
    buffer_address: *mut u8,
    buffer_size: u32,
    address: EndpointAddress,
    endpoint_type: EndpointType,
}

impl EndpointBuffer {
    pub fn new(address: EndpointAddress) -> Self {
        EndpointBuffer {
            endpoint_entry: core::ptr::null_mut(),
            buffer_address: core::ptr::null_mut(),
            buffer_size: 0,
            address,
            endpoint_type: EndpointType::Control,
        }
    }

    pub fn get_hw_mut(&mut self) -> &mut HardwareEndpoint {
        unsafe { &mut *self.endpoint_entry }
    }

    pub fn get_hw(&self) -> &HardwareEndpoint {
        unsafe { &*self.endpoint_entry }
    }

    pub fn configure(
        &mut self,
        endpoint_type: EndpointType,
        buffer_address: *mut u8,
        buffer_size: u32,
    ) {
        self.endpoint_type = endpoint_type;
        self.buffer_address = buffer_address;
        self.buffer_size = buffer_size;

        let hardware_endpoint = self.get_hw_mut();

        hardware_endpoint.set_size(buffer_size);
        hardware_endpoint.set_address(buffer_address as u32);
        hardware_endpoint.set_cs(0);

        if endpoint_type == EndpointType::Isochronous {
            hardware_endpoint.set_iso_type(true);
        }

        self.disable();
    }

    pub fn setup_for_output(&mut self) {
        let buffer_address = self.buffer_address as u32;

        let hardware_endpoint = self.get_hw_mut();

        hardware_endpoint.set_address(buffer_address);
    }

    pub fn setup_for_setup(&mut self) {
        let buffer_address = self.buffer_address as u32;
        let buffer_size = self.buffer_size;

        let hardware_endpoint = self.get_hw_mut();

        hardware_endpoint.set_address(buffer_address);
    }

    pub fn setup_for_input(&mut self, data_ptr: *const u8, count: u32) {
        debug_assert!(count > self.buffer_size);

        let address = self.address;

        let buffer_address = self.buffer_address as u32;

        unsafe {
            data_ptr.copy_to(self.buffer_address, count as usize);
        }

        let hardware_endpoint = self.get_hw_mut();

        hardware_endpoint.set_address(buffer_address);
        hardware_endpoint.set_size(count);

        // FIXME: NXP seems to clear bit 26 & 27???
        //hardware_endpoint.set_iso_type(false);

        if address.index() == 0 || !hardware_endpoint.is_stalled() {
            hardware_endpoint.set_active(true);
        }
    }

    pub fn reset_after_stall(&mut self, is_active: bool) {
        let address = self.address;
        let buffer_size = self.buffer_size;
        let hardware_endpoint = self.get_hw_mut();
        hardware_endpoint.set_reset(true);

        if address.direction() == UsbDirection::In {
            // if the IN EP was set active, reflect it in the hardware endpoint entry.
            if is_active {
                hardware_endpoint.set_active(true);
            }
        } else {
            // For OUT EP, we reactivate and reset the size to the previous value.
            hardware_endpoint.set_active(true);
            hardware_endpoint.set_size(buffer_size);
        }
    }

    pub fn enable(&mut self) {
        if self.address.index() != 0 {
            let address = self.address;
            let hardware_endpoint = self.get_hw_mut();

            hardware_endpoint.set_disabled(false);

            // EP OUT mut be active when enabled
            if address.direction() == UsbDirection::Out {
                hardware_endpoint.set_active(true);
            }
        }
    }

    pub fn disable(&mut self) {
        if self.address.index() != 0 {
            self.get_hw_mut().set_disabled(true);
        }
    }

    pub fn write(&mut self, buf: &[u8]) -> Result<(usize, bool)> {
        if self.address.index() == 0 {
            // EP0 OUT must be set active it seems?
            unsafe {
                (*self.endpoint_entry.offset(-2)).set_active(true);
            }
        }

        let count = core::cmp::min(self.buffer_size as usize, buf.len());

        self.setup_for_input(buf.as_ptr(), count as u32);

        Ok((count, self.get_hw().is_stalled()))
    }

    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        let buffer_size = self.buffer_size;

        let hardware_endpoint = self.get_hw_mut();
        let count = core::cmp::min(buf.len(), (buffer_size - hardware_endpoint.size()) as usize);

        unsafe {
            // Copy the data packet from the buffer
            buf.as_mut_ptr().copy_from(self.buffer_address, count);
        }

        unsafe { self.read_unsafe(self.buffer_address, count) }
    }

    pub fn read_setup(&self, buf: &mut [u8]) -> Result<usize> {
        // Disable stall if set for EP0 OUT/IN
        unsafe {
            if (*self.endpoint_entry).is_stalled() || (*self.endpoint_entry.offset(2)).is_stalled()
            {
                (*self.endpoint_entry).set_stalled(false);
                (*self.endpoint_entry.offset(2)).set_stalled(false);
            }
        }

        let count = core::cmp::min(buf.len(), 8);

        unsafe { self.read_unsafe(self.buffer_address, count) }
    }

    unsafe fn read_unsafe(&self, buf: *mut u8, count: usize) -> Result<usize> {
        buf.copy_from(self.buffer_address, count);

        Ok(count)
    }
}

pub struct Endpoint {
    address: EndpointAddress,
    endpoint_type: Option<EndpointType>,
    registers: lpc11uxx::USB,
    is_active: Cell<bool>,
    is_used: Cell<bool>,
    is_setup: Cell<bool>,
    first_buffer: RefCell<EndpointBuffer>,
    second_buffer: RefCell<EndpointBuffer>,
    max_packet_size: u32,
}

// SAFETY: We only have one core so no issues here :)
unsafe impl Sync for Endpoint {}

pub static mut USB_BUS_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
pub static mut USB_DEVICE: Option<UsbDevice<'static, UsbBus>> = None;

impl Endpoint {
    pub fn new(address: EndpointAddress, registers: lpc11uxx::USB) -> Self {
        Endpoint {
            address,
            endpoint_type: None,
            registers,
            first_buffer: RefCell::new(EndpointBuffer::new(address)),
            second_buffer: RefCell::new(EndpointBuffer::new(address)),
            is_active: Cell::new(false),
            is_used: Cell::new(false),
            is_setup: Cell::new(false),
            max_packet_size: MAX_PACKET0 as u32,
        }
    }

    pub fn get_address(&self) -> EndpointAddress {
        self.address
    }

    pub fn set_endpoint_type(&mut self, endpoint_type: EndpointType) {
        self.endpoint_type = Some(endpoint_type);
    }

    pub fn endpoint_type(&self) -> Option<EndpointType> {
        self.endpoint_type
    }

    pub fn initialize(&self, usb_memory_base: *mut u8) {
        let endpoint_buffer = unsafe { usb_memory_base.offset(0x100) };
        let address = self.address;

        let endpoint_index = UsbBus::get_buffer_offset_with_address(address);
        let buffer_index = endpoint_index * 2;

        let target_buffer_address =
            unsafe { endpoint_buffer.offset(self.max_packet_size as isize * buffer_index) };
        let target_double_buffer_address =
            unsafe { target_buffer_address.offset(self.max_packet_size as isize) };

        self.first_buffer.borrow_mut().configure(
            self.endpoint_type.unwrap_or(EndpointType::Bulk),
            target_buffer_address,
            self.max_packet_size,
        );
        self.second_buffer.borrow_mut().configure(
            self.endpoint_type.unwrap_or(EndpointType::Bulk),
            target_double_buffer_address,
            self.max_packet_size,
        );

        self.is_active.set(false);
        self.is_used.set(false);
    }

    fn deactivate(&self) {
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
        self.first_buffer.borrow().get_hw().is_stalled()
    }

    pub fn set_stalled(&self, stalled: bool) {
        let double_buffer_mask = 1 << UsbBus::get_buffer_offset_with_address(self.address);

        let mut first_buffer = self.first_buffer.borrow_mut();
        let mut second_buffer = self.second_buffer.borrow_mut();

        if !stalled {
            if self.address.index() == 0 {
                // If we are on EP0, we just need to clear the stall bit.
                first_buffer.get_hw_mut().set_stalled(false);
            } else {
                // If we are on a non zero endpoint, we need to clear the stall bit on our double buffer and choose the appropriate one to reset.
                first_buffer.get_hw_mut().set_stalled(false);
                second_buffer.get_hw_mut().set_stalled(false);

                let mut target_endpoint_entry;

                if (self.registers.epinuse.read().bits() & double_buffer_mask) != 0 {
                    // Secondary buffer in use.
                    target_endpoint_entry = second_buffer;
                } else {
                    // Primary buffer in use.
                    target_endpoint_entry = first_buffer;
                }

                target_endpoint_entry.reset_after_stall(self.is_active.get());

                self.is_active.set(false);
            }
        } else {
            let mut hardware_endpoint = first_buffer;
            // If the active bit is set, we need to get ride of it before setting the stalled bit
            let endpoint_entry = hardware_endpoint.get_hw_mut();

            let is_active = endpoint_entry.is_active();
            if is_active {
                self.deactivate();
            }

            // Set stall bit
            endpoint_entry.set_stalled(true);

            // If we aren't on EP0 and double buffering is active, we need to set the stall bit on the double buffer.
            if self.address.index() != 0
                && (self.registers.epbufcfg.read().bits() & double_buffer_mask) != 0
            {
                let mut hardware_endpoint = second_buffer;
                let endpoint_entry = hardware_endpoint.get_hw_mut();

                let is_active = endpoint_entry.is_active();
                if is_active {
                    self.deactivate();
                }

                // Set stall bit
                endpoint_entry.set_stalled(true);
            }
        }
    }

    pub fn write(&self, buf: &[u8]) -> Result<usize> {
        let buffer_index = UsbBus::get_buffer_offset_with_address(self.address);
        let double_buffer_mask = 1 << buffer_index;

        let mut target_buffer;

        if (self.registers.epinuse.read().bits() & double_buffer_mask) != 0
            && (self.registers.epbufcfg.read().bits() & double_buffer_mask) != 0
        {
            // Secondary buffer in use.
            target_buffer = self.second_buffer.borrow_mut();
        } else {
            // Primary Buffer in use.
            target_buffer = self.first_buffer.borrow_mut();
        }

        let (count, is_stalled) = target_buffer.write(buf)?;

        if self.address.index() != 0 && is_stalled {
            self.is_active.set(true);
        }

        Ok(count)
    }

    pub fn read(&self, buf: &mut [u8]) -> Result<usize> {
        if self.is_setup.get() {
            let res = self.read_setup(buf);

            self.is_setup.set(false);

            res
        } else {
            self.read_normal(buf)
        }
    }

    fn read_setup(&self, buf: &mut [u8]) -> Result<usize> {
        let mut buffer = self.second_buffer.borrow_mut();
        let res = buffer.read_setup(buf);

        buffer.setup_for_setup();

        res
    }

    fn read_normal(&self, buf: &mut [u8]) -> Result<usize> {
        let mut first_buffer = self.first_buffer.borrow_mut();
        let mut second_buffer = self.second_buffer.borrow_mut();
        let first_buffer_ptr = first_buffer.deref_mut() as *mut EndpointBuffer;
        let second_buffer_ptr = second_buffer.deref_mut() as *mut EndpointBuffer;

        let buffer_index = UsbBus::get_buffer_offset_with_address(self.address);
        let double_buffer_mask = 1 << buffer_index;

        let mut target_buffer_ptr;

        if self.address.index() != 0 && self.is_used.get() {
            // Secondary buffer
            target_buffer_ptr = second_buffer_ptr;
        } else {
            // Primary buffer
            target_buffer_ptr = first_buffer_ptr;
        }

        let res = unsafe { (*target_buffer_ptr).read(buf) };

        if self.address.index() != 0
            && (self.registers.epbufcfg.read().bits() & double_buffer_mask) != 0
        {
            self.is_used
                .set((self.registers.epinuse.read().bits() & double_buffer_mask) != 0);

            if self.is_used.get() {
                target_buffer_ptr = second_buffer_ptr;
            } else {
                target_buffer_ptr = first_buffer_ptr;
            }
        }

        // Reset to the original state and ready to receive buffers.
        unsafe {
            (*target_buffer_ptr).setup_for_output();
        }

        res
    }
}

bitfield! {
  pub struct HardwareEndpoint(u32);
  impl Debug;
  pub address, set_address: 15, 0;
  pub size, set_size : 25, 16;
  pub is_iso_type, set_iso_type: 26;
  // bit 27: ???
  pub is_reset, set_reset: 28;
  pub is_stalled, set_stalled: 29;
  pub is_disabled, set_disabled: 30;
  pub is_active, set_active: 31;
  pub cs, set_cs: 31, 26;
}

pub struct UsbBus {
    endpoints: ArrayVec<[Endpoint; MAX_EP_PHYSICAL_COUNT]>,
    registers: lpc11uxx::USB,
    usb_memory_base: *mut u8,
    interrupt_status: RefCell<Option<lpc11uxx::usb::intstat::R>>,
}

unsafe impl Sync for UsbBus {}

impl UsbBus {
    pub fn new() -> UsbBusAllocator<Self> {
        let mut bus = UsbBus {
            endpoints: ArrayVec::new(),
            registers: unsafe { Peripherals::steal().USB },
            usb_memory_base: 0x20004000 as *mut u8,
            interrupt_status: RefCell::new(None),
        };

        for i in 0..MAX_EP_LOGICAL_COUNT {
            let out_endpoint =
                Endpoint::new(EndpointAddress::from_parts(i, UsbDirection::Out), unsafe {
                    Peripherals::steal().USB
                });
            bus.endpoints.push(out_endpoint);

            let in_endpoint =
                Endpoint::new(EndpointAddress::from_parts(i, UsbDirection::In), unsafe {
                    Peripherals::steal().USB
                });
            bus.endpoints.push(in_endpoint);
        }

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
        (endpoint_address.index() * 2
            + if endpoint_address.direction() == UsbDirection::In {
                1
            } else {
                0
            }) as isize
    }

    unsafe fn init_endpoints(&self) {
        self.registers
            .epliststart
            .write(|writer| writer.bits(self.usb_memory_base as u32));
        self.registers
            .databufstart
            .write(|writer| writer.bits(self.usb_memory_base.offset(0x100) as u32));

        // Clear eplist
        core::ptr::write_bytes(self.usb_memory_base, 0, 0x100);

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

    pub fn handle_irq(&self) {
        // First we get USB interruption status and immediately clear the register.
        let interrupt_status = self.registers.intstat.read();
        self.registers
            .intstat
            .write(|writer| unsafe { writer.bits(interrupt_status.bits()) });
        self.interrupt_status.replace(Some(interrupt_status));
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
        for index in ep_addr
            .map(|a| {
                let index = Self::get_buffer_offset_with_address(a) as usize;
                index..index + 1
            })
            .unwrap_or(2..self.endpoints.len())
        {
            let ep = &mut self.endpoints[index];

            match ep.endpoint_type() {
                None => {}
                _ => continue,
            };

            match ep_dir {
                UsbDirection::Out if ep.get_address().direction() == UsbDirection::Out => {
                    ep.set_endpoint_type(ep_type);

                    return Ok(EndpointAddress::from_parts(index, ep_dir));
                }
                UsbDirection::In if ep.get_address().direction() == UsbDirection::In => {
                    ep.set_endpoint_type(ep_type);

                    return Ok(EndpointAddress::from_parts(index, ep_dir));
                }
                _ => {}
            }
        }

        Err(match ep_addr {
            Some(_) => UsbError::InvalidEndpoint,
            None => UsbError::EndpointOverflow,
        })
    }

    fn enable(&mut self) {
        // TODO: move the usb clock init here?

        // Enable USB IRQ
        unsafe {
            let mut peripheral = CorePeripherals::steal();
            peripheral.NVIC.set_priority(Interrupt::USB_IRQ, 1);
            NVIC::unmask(Interrupt::USB_IRQ);
        }

        // reset devcmdstat
        self.registers
            .devcmdstat
            .write(|writer| unsafe { writer.bits(0x0) });
        self.reset();

        self.set_device_address(0);
        self.connect();
    }

    fn reset(&self) {
        // Clear reset bit if it's set
        if self.registers.devcmdstat.read().dres_c().bit() {
            self.registers
                .devcmdstat
                .modify(|_, writer| writer.dres_c().set_bit());
        }

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

        // Enable double buffering
        self.registers
            .epbufcfg
            .write(|writer| unsafe { writer.bits(0x3FF) });

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
        let interrupt_status_opt = self.interrupt_status.borrow_mut().take();

        if interrupt_status_opt.is_none() {
            return PollResult::None;
        }

        let interrupt_status = interrupt_status_opt.unwrap();

        // FIXME: maybe we should clear not enable lines? it's not clear if we can't trust the hardware..
        //interrupt_status &= self.registers.inten.read();

        // Device Status interruption?
        if interrupt_status.dev_int().bit() {
            if self.registers.devcmdstat.read().dres_c().bit() {
                self.endpoints[0].is_setup.set(true);
                return PollResult::Reset;
            }

            // Clear connect bit if it's set
            if self.registers.devcmdstat.read().dcon_c().bit() {
                self.registers
                    .devcmdstat
                    .modify(|_, writer| writer.dcon_c().set_bit());
            }

            // Find suspension change bit and clear it if it's set
            if self.registers.devcmdstat.read().dsus_c().bit() {
                self.registers
                    .devcmdstat
                    .modify(|_, writer| writer.dsus_c().set_bit());

                // Report the right suspension event
                if self.registers.devcmdstat.read().dsus().bit() {
                    return PollResult::Suspend;
                } else {
                    return PollResult::Resume;
                }
            }
        }

        let mut ep_out: u16 = 0;
        let mut ep_in_complete: u16 = 0;
        let mut ep_setup: u16 = 0;

        // If there is any changes on any hardware EP.
        if (interrupt_status.bits() & 0x3FF) != 0 {
            for endpoint_index in 0..MAX_EP_PHYSICAL_COUNT {
                if (interrupt_status.bits() & 1 << endpoint_index) != 0 {
                    if (endpoint_index % 1) == 0 {
                        // If we are on OUT EP0, check for possible setup packet.
                        if endpoint_index == 0 {
                            // Find suspension change bit and clear it if it's set
                            if self.registers.devcmdstat.read().setup().bit() {
                                self.registers
                                    .devcmdstat
                                    .modify(|_, writer| writer.setup().set_bit());

                                ep_setup += 1;
                                continue;
                            }
                        }

                        // This is an OUT EP.
                        ep_out += 1;
                    } else {
                        // This is an IN EP.
                        ep_in_complete += 1;
                    }
                }
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
        // TODO: support suspension?
        //unimplemented!()
    }

    fn resume(&self) {
        // TODO: support suspension?
        //unimplemented!()
    }
}

#[interrupt]
fn USB_IRQ() {
    // This isn't used to handle USB interruptions because of how usb-device crate is designed. (around a polling user controlled)
    // As such, the USB interrupt is used to reduce polling cost by making the user code wait for an interruption.
    unsafe {
        if let Some(usb_device) = &USB_DEVICE {
            usb_device.bus().handle_irq();
        }
    }
}
