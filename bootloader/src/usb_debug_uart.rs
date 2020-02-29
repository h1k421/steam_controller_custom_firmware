use vcell::VolatileCell;
use lpc11uxx_rom::usbd::{UsbHandle, CdcHandle, CdcInitParameter, CdcLineCoding};
use lpc11uxx_rom::RomDriver;
use core::ptr::NonNull;

const UCOM_TX_BUF_SZ: usize = 256;
const UCOM_RX_BUF_SZ: usize = 256;

struct UcomData<'a> {
    connected: VolatileCell<bool>,
    /// Handle to USB device stack
    usb: UsbHandle,
    /// handle to CDC Controller
    cdc: CdcHandle,

    /// Buffer treated as a FIFO which stores character data that has been
    /// receive via USB CDC UART
    rx_fifo: &'a mut [u8],
    rx_rd_idx: usize,
    rx_wr_idx: usize,

    /// Flag that we did not have enough room for incoming data in rxFifo and
    /// did not call ReadEP. We will need to call ReadEP to get the buffered
    /// data and have interrupts start working again once room opens up in
    /// tx_fifo. Note that even though we do not call ReadEP, incoming USB
    /// packets may be dropped and into data lost. Still looking for solutions
    /// to fix this.
    rx_stalled: bool,
    /// Indicates transmission is in progress. This does not guarantee that
    /// tx_fifo will be drained (use usb_flush() for this).
    tx_busy: VolatileCell<bool>,

    tx_fifo: &'a mut [u8],
    tx_rd_idx: VolatileCell<usize>,
    tx_wr_idx: usize,
    tx_sent: usize,

    ep_in_idx: u32,
    ep_out_idx: u32,
}

static mut UCOM_DATA: UcomData<'static> = UcomData {
    connected: VolatileCell::new(false),
    usb: UsbHandle::null(),
    cdc: CdcHandle::null(),
    rx_fifo: &mut [],
    rx_rd_idx: 0,
    rx_wr_idx: 0,
    rx_stalled: false,
    tx_busy: VolatileCell::new(false),
    tx_fifo: &mut [],
    tx_rd_idx: VolatileCell::new(0),
    tx_wr_idx: 0,
    tx_sent: 0,

    ep_in_idx: 0,
    ep_out_idx: 0,
};

extern fn set_line_code(hnd: CdcHandle, line_coding: &mut CdcLineCoding) -> i32 {
    unsafe { UCOM_DATA.connected.set(true); }
    0
}

pub fn init_usb_cdc(usb_handle: UsbHandle, cif_intf_desc: &mut [u8],
    dif_intf_desc: &mut [u8], mem_base: &mut u32, mem_size: &mut u32,
    ep_in_idx: u32, ep_out_idx: u32) -> i32
{
    let usb_api = RomDriver::get().usb_api();

    unsafe {
        UCOM_DATA.usb = usb_handle;
        UCOM_DATA.ep_in_idx = ep_in_idx;
        UCOM_DATA.ep_out_idx = ep_out_idx;
    }

    let mut init_param = CdcInitParameter::default();
    init_param.mem_base = *mem_base;
    init_param.mem_size = *mem_size;
    init_param.cif_intf_desc = Some(NonNull::from(cif_intf_desc).cast());
    init_param.dif_intf_desc = Some(NonNull::from(dif_intf_desc).cast());
    init_param.set_line_code = Some(set_line_code);
    let err = (usb_api.cdc().init)(usb_handle, &mut init_param, unsafe { &mut UCOM_DATA.cdc });

    if err != 0 {
        return err
    }

    // Allocate transfer buffers.
    if init_param.mem_size < UCOM_TX_BUF_SZ as u32 {
        return 1;
    }
    unsafe {
        UCOM_DATA.tx_fifo = core::slice::from_raw_parts_mut(init_param.mem_base as *mut u8, UCOM_TX_BUF_SZ);
    }
    init_param.mem_base += UCOM_TX_BUF_SZ as u32;
    init_param.mem_size -= UCOM_TX_BUF_SZ as u32;

    if init_param.mem_size < UCOM_RX_BUF_SZ as u32 {
        return 1;
    }
    unsafe {
        UCOM_DATA.rx_fifo = core::slice::from_raw_parts_mut(init_param.mem_base as *mut u8, UCOM_RX_BUF_SZ);
    }
    init_param.mem_base += UCOM_RX_BUF_SZ as u32;
    init_param.mem_size -= UCOM_RX_BUF_SZ as u32;

    *mem_base = init_param.mem_base;
    *mem_size = init_param.mem_size;

    // Register Endpoint Interrupt Handler
    let ep_idx = ((ep_in_idx & 0x0F) << 1) + 1;
    let err = (usb_api.core().register_ep_handler)(usb_handle, ep_idx, ucom_bulk_hdlr, unsafe { &mut UCOM_DATA } as *mut _ as *mut u8);
    if err != 0 {
        return err;
    }

    let ep_idx = (ep_out_idx & 0x0F) << 1;
    let err = (usb_api.core().register_ep_handler)(usb_handle, ep_idx, ucom_bulk_hdlr, unsafe { &mut UCOM_DATA } as *mut _ as *mut u8);

    unsafe {
        UCOM_DATA.cdc.inner_ctrl().line_coding.dte_rate = 115200;
        UCOM_DATA.cdc.inner_ctrl().line_coding.data_bits = 8;
    }

    return err
}

fn usb_tx_fifo_num_bytes(ucom: &UcomData) -> usize {
    let tx_rd_idx = ucom.tx_rd_idx.get();
    if ucom.tx_wr_idx >= tx_rd_idx {
        return ucom.tx_wr_idx - tx_rd_idx;
    }

    return 1 + tx_rd_idx + (ucom.tx_fifo.len() - ucom.tx_wr_idx);
}

fn usb_uart_tx_start(ucom: &mut UcomData) {
    if ucom.tx_busy.get() {
        return
    }

    ucom.tx_busy.set(true);

    let mut bytes_to_send = usb_tx_fifo_num_bytes(ucom);

    if bytes_to_send == 0 {
        ucom.tx_busy.set(false);
        return;
    }

    // Seems we can enter an error state (i.e. interrupt does not fire or
    //  keeps firing continuously) if we tell WriteEP() that it should send
    //  number of bytes larger than what is specified for wMaxPacketSize
    if bytes_to_send > 64 /* USB_MAX_PACKET_SZ */ {
        bytes_to_send = 64;
    }

    // Apparently IRQs need to be disabled around call to WriteEP. If they
    //  are not and other interrupts are active of (higher priority)
    //  (i.e. ADC interrupts) we can get repeated prints or dropped data...
    //  Not sure why. Maybe related to built in CDC UART support?
    cortex_m::interrupt::free(|_| {
        let usb_api = RomDriver::get().usb_api();
        let tx_rd_idx = ucom.tx_rd_idx.get();
        if let Some(fifo_slice) = ucom.tx_fifo.get(tx_rd_idx..tx_rd_idx + bytes_to_send) {

            ucom.tx_sent = (usb_api.hw().write_ep)(ucom.usb, ucom.ep_in_idx, fifo_slice.as_ptr(), bytes_to_send as u32) as usize;
        }
    });

    if ucom.tx_sent == 0 {
        ucom.tx_busy.set(false);
    }
}

/// Receive data from the USB CDC UART.
fn usb_uart_rcv_data(ucom: &mut UcomData) {
    let usb_api = RomDriver::get().usb_api();
    if let Some(fifo_slice) = ucom.rx_fifo.get_mut(0..0x40) {
        cortex_m::interrupt::free(|_| usb_api.hw().read_ep)(ucom.usb, ucom.ep_out_idx, fifo_slice.as_mut_ptr());
    }
}

extern fn ucom_bulk_hdlr(usb: UsbHandle, data: *mut u8, evt: u32) -> i32 {
    let usb_api = RomDriver::get().usb_api();

    let ucom = unsafe { (data as *mut UcomData).as_mut().unwrap() };
    match evt {
        // USB_EVT_IN
        // Transfer from us to the USB host that we queued was completed.
        3 => {
            // We're done!
            ucom.tx_rd_idx.set((ucom.tx_rd_idx.get() + ucom.tx_sent) % ucom.tx_fifo.len());
            ucom.tx_sent = 0;
            ucom.tx_busy.set(false);


            // Try to start another one.
            usb_uart_tx_start(ucom);
        },
        // USB_EVT_OUT
        2 => {
            usb_uart_rcv_data(ucom);
        },
        // ERR_USBD_STALL
        0x40003 => {
            // setLedIntensity(0)
        },
        _ => (),
    }

    0
}

pub fn usb_putc(chara: u8) -> i32 {
    unsafe {
        let mut next_wr_idx = (UCOM_DATA.tx_wr_idx + 1) % UCOM_DATA.tx_fifo.len();

        // We're full already. Fuck.
        if next_wr_idx == UCOM_DATA.tx_rd_idx.get() {
            return chara as i32;
        }

        // Put new character info FIFO
        if let Some(place) = UCOM_DATA.tx_fifo.get_mut(UCOM_DATA.tx_wr_idx) {
            *place = chara;
            UCOM_DATA.tx_wr_idx = next_wr_idx;
        }

        if UCOM_DATA.tx_busy.get() {
            return chara as i32
        }

        //if usb_tx_fifo_num_bytes(&UCOM_DATA) >= UCOM_DATA.tx_fifo.len() / 2 && UCOM_DATA.connected.get() {
        //    usb_uart_tx_start(&mut UCOM_DATA);
        //}
    }

    return 0;
}

pub fn usb_putb(data: &[u8]) {
    for c in data {
        usb_putc(*c);
    }
}

pub fn usb_putnbr(mut n: u32) {
    usb_putnbr_base(n, b"0123456789");
}

pub fn usb_putnbr_hex(mut n: u32) {
    usb_putnbr_base(n, b"0123456789abcdef");
}

pub fn usb_putnbr_base(mut n: u32, base: &[u8]) {
    // At most 32 digits in a 32 bit number. We assume base is at least 2.
    let mut buf = [0; 32];
    let mut idx = 0;
    let base_len = base.len() as u32;
    // Do while pattern. This ensures that if n == 0, we still print 0.
    while {
        buf[buf.len() - idx - 1] = base[(n % base_len) as usize];
        n /= base_len;
        idx += 1;
        n != 0
    } {}
    usb_putb(&buf[buf.len() - idx..]);
}

pub fn usb_flush() {
    unsafe {
        while UCOM_DATA.tx_busy.get() {}

        usb_uart_tx_start(&mut UCOM_DATA);
    }
}

/*pub fn wait_until_sent() {
    // TODO: Figure this out
    unsafe {
        while UCOM_DATA.tx_rd_idx.get() != UCOM_DATA.tx_wr_idx {}
    }
}*/

pub fn enabled() -> bool {
    return unsafe { UCOM_DATA.connected.get() }
}

pub struct UartDebug;

impl core::fmt::Write for UartDebug {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        usb_putb(s.as_bytes());
        Ok(())
    }
}