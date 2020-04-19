use heapless::spsc::Queue;
use lpc11uxx::{Peripherals, Interrupt, SCB, USART, SYSCON, WWDT};
use cortex_m::peripheral::NVIC;
use cortex_m::peripheral::scb::SystemHandler;

static mut USART_WRITE_RING_BUFFER: Queue<u8, heapless::consts::U256> = Queue(heapless::i::Queue::new());
static mut USART_READ_STATE: UsartReadState = UsartReadState::NoData;
static mut USART_READ_PACKET_LEN: u8 = 0;
static mut USART_READ_PACKET: [u8; 256] = [0; 256];
static mut SHOULD_SEND_USART_PACKET: bool = false;

static mut USART_READ_PENDSV_PACKET_LEN: u8 = 0;
static mut USART_READ_PENDSV_PACKET: [u8; 256] = [0; 256];

#[derive(Debug, Clone, Copy)]
enum UsartReadState {
    NoData,
    HandlingFrame,
    EscapedCharacter,
    PacketSubmitted,
}

pub fn init_usart(syscon: &SYSCON, usart: &USART, nvic: &mut NVIC, scb: &mut SCB) {
    unsafe {
        USART_READ_STATE = UsartReadState::NoData;
    }

    // RingBuffer_Init is unnecessary - we're using ArrayDeque

    // Chip_UART_Init
    syscon.sysahbclkctrl.modify(|_, writer| writer.usart().enabled());
    syscon.uartclkdiv.write(|v| unsafe { v.div().bits(1) });
    usart.fcr_mut().write(|v| v
        .fifoen().enabled()
        .rxfifores().clear()
        .txfifores().clear());
    usart.lcr.write(|v| v
        .wls()._8_bit_character_leng()
        .sbs()._1_stop_bit());
    usart.fdr.write(|v| unsafe { v.mulval().bits(1) });

    // ChipUART_SetupFifos
    usart.fcr_mut().write(|v| v
        .fifoen().enabled()
        .rxtl().level2());

    // Enable access to the divisor registers.
    usart.lcr.modify(|_, v| v.dlab().enable_access_to_div());

    // Set the USART divisor latch to 3
    usart.dll().write(|v| unsafe { v.dllsb().bits(3) });

    usart.fdr.write(|v| unsafe {
        v
            .divaddval().bits(1)
            .mulval().bits(11)
    });

    // Disable access to the divisor registers, restore access to USART read/write registers.
    usart.lcr.modify(|_, v| v.dlab().disable_access_to_di());

    unsafe { NVIC::unmask(Interrupt::USART) };
    usart.ier_mut().modify(|_, v| v
        .rbrinten().enable_the_rda_inter()
        .rlsinten().enable_the_rls_inter());

    // TODO: Make sure those priority numbers are correct. NVIC_SetPriority does
    // fancy bit shifting I don't fully understand at 3AM.
    unsafe { nvic.set_priority(Interrupt::USART, 0) };
    unsafe { scb.set_priority(SystemHandler::PendSV, 1) };
}

pub fn handle_interrupt(mut usart: &mut USART) {
    if usart.iir().read().intid().is_receive_line_status() {
        let lsr = usart.lsr.read();
        let oe = lsr.oe().is_active();
        let pe = lsr.pe().is_active();
        let fe = lsr.fe().is_active();
        let bi = lsr.bi().is_active();
        let rxfe = lsr.rxfe().is_erro();

        if oe || pe || fe || bi || rxfe {
            usart.rbr().read();
        }

        if lsr.rdr().is_valid() {
            usart.rbr().read();
        }
    }


    while usart.lsr.read().rdr().is_valid() {
        handle_usart_data(&mut usart);
    }

    if usart.ier().read().threinten().bit_is_set() {
        while usart.lsr.read().thre().bit_is_set() {
            if let Some(val) = unsafe { USART_WRITE_RING_BUFFER.dequeue() } {
                usart.thr_mut().write(|v| unsafe { v.thr().bits(val) });
            } else {
                break;
            }
        }
        if unsafe { USART_WRITE_RING_BUFFER.is_empty() } {
            usart.ier_mut().modify(|_, v| v.threinten().clear_bit());
        }
    }
}

pub fn handle_pendsv(wwdt: &WWDT, syscon: &SYSCON) {
    match unsafe { USART_READ_PENDSV_PACKET[0] } {
        b'P' => unsafe {
            let size = crate::programming_mode::hid_handle_set_feature_report(wwdt, syscon, &USART_READ_PENDSV_PACKET[1..USART_READ_PENDSV_PACKET_LEN as usize]);
            if size != 0 {
                usart_send_hid_report(size);
            }
        },
        b'R' => unsafe {
            SHOULD_SEND_USART_PACKET = true;
        },
        b'S' => unsafe {
            SHOULD_SEND_USART_PACKET = false;
        },
        b'[' => unsafe {
            let size = crate::programming_mode::write_report_0x94(USART_READ_PENDSV_PACKET[1] as u16);
            usart_send_hid_report(size);
        },
        _ => ()
    }
    unsafe { USART_READ_STATE = UsartReadState::NoData; }
}

pub fn handle_usart_byte(data: u8) {
    unsafe {
        match (USART_READ_STATE, data) {
            (UsartReadState::PacketSubmitted, _) => (),
            (UsartReadState::NoData, 0x02) => {
                USART_READ_PACKET_LEN = 0;
                USART_READ_STATE = UsartReadState::HandlingFrame;
            },
            (UsartReadState::NoData, _) => (),
            (UsartReadState::EscapedCharacter, _) => {
                USART_READ_PACKET[USART_READ_PACKET_LEN as usize] = data ^ 0x20;
                USART_READ_PACKET_LEN = USART_READ_PACKET_LEN.wrapping_add(1);
            },
            (UsartReadState::HandlingFrame, 0x1f) => {
                USART_READ_STATE = UsartReadState::EscapedCharacter;
            },
            (UsartReadState::HandlingFrame, 0x03) => {
                // Figure out what's up with packet W? Why is it ignored?
                if USART_READ_PACKET_LEN != 0 {
                    if USART_READ_PACKET[0] != 0x57 {
                        let packet_len = USART_READ_PACKET_LEN as usize;
                        USART_READ_PENDSV_PACKET[..packet_len].copy_from_slice(&USART_READ_PACKET[..packet_len]);
                        USART_READ_PENDSV_PACKET_LEN = USART_READ_PACKET_LEN;
                        SCB::set_pendsv();
                        USART_READ_PACKET_LEN = 0;
                        USART_READ_STATE = UsartReadState::PacketSubmitted;
                        return;
                    }
                    USART_READ_PACKET_LEN = 0;
                    USART_READ_STATE = UsartReadState::NoData;
                }
            }
            (UsartReadState::HandlingFrame, _) => {
                USART_READ_PACKET[USART_READ_PACKET_LEN as usize] = data;
                USART_READ_PACKET_LEN = USART_READ_PACKET_LEN.wrapping_add(1);
            }
        }
    }
}

fn handle_usart_data(usart: &mut USART) {
    let data = usart.rbr().read().rbr().bits();
    handle_usart_byte(data);
}


fn RingBuffer_InsertMult(ring_buffer: &mut Queue<u8, heapless::consts::U256>, data: &[u8]) -> usize {
    let old_len = ring_buffer.len();
    // TODO: Please tell me this turns into a simple memcpy...
    for item in data {
        if ring_buffer.enqueue(*item).is_err() {
            break;
        }
    }
    ring_buffer.len() - old_len
}


fn usart_send_raw_str(data: &[u8]) -> usize {
    let peripherals = unsafe { Peripherals::steal() };

    // First, disable send interrupts.
    peripherals.USART.ier_mut().modify(|_, v| v.threinten().disable_the_thre_int());

    // Insert data to the ring buffer
    let mut inserted_len = RingBuffer_InsertMult(unsafe { &mut USART_WRITE_RING_BUFFER }, data);

    // Send the contents of the ring buffer
    while peripherals.USART.lsr.read().thre().is_empty() {
        if let Some(val) = unsafe { USART_WRITE_RING_BUFFER .dequeue() } {
            peripherals.USART.thr_mut().write(|v| unsafe { v.thr().bits(val) });
        } else {
            break;
        }
    }

    // Try to insert some more contents in the ring buffer.
    inserted_len += RingBuffer_InsertMult(unsafe { &mut USART_WRITE_RING_BUFFER }, &data[inserted_len..]);

    // Re-enable send interrupts
    peripherals.USART.ier_mut().modify(|_, v| v.threinten().enable_the_thre_inte());

    inserted_len
}

fn usart_send_02() {
    usart_send_raw_str(b"\x02");
}

// TODO: Write unit tests for this function cuz it's almost guaranteed to be wrong lol.
fn usart_send_escaped_str(mut data: &[u8]) {
    while let Some(pos) = data.iter().position(|&v| v == 0x02 || v == 0x03 || v == 0x1f) {
        if pos != 0 {
            usart_send_raw_str(&data[..pos]);
        }
        usart_send_raw_str(&[0x1fu8, data[pos] ^ 0x20]);
        data = &data[pos + 1..];
    }

    if !data.is_empty() {
        usart_send_raw_str(data);
    }
}

fn usart_send_03() {
    usart_send_raw_str(b"\x03");
}

pub fn usart_send_text_transmission(data: &[u8]) {
    // Replace disable_irq/enable_irq pairs with cortex_m::interrupt::free
    cortex_m::interrupt::free(|_| {
        usart_send_02();
        usart_send_escaped_str(data);
        usart_send_03();
    })
}

pub fn usart_send_R() {
    usart_send_text_transmission(b"R");
}

/// Ends an NRF firmware upload, and sends a checksum to verify there wasn't any
/// corruption on the wire.
pub fn usart_send_sig_packet(data: &[u8]) {
    cortex_m::interrupt::free(|_| {
        usart_send_02();
        usart_send_escaped_str(b"[");
        usart_send_escaped_str(&data[..0x10]);
        usart_send_03();
    })
}

pub fn usart_send_z_packet(data: &[u8]) {
    cortex_m::interrupt::free(|_| {
        usart_send_02();
        usart_send_escaped_str(b"Z");
        usart_send_escaped_str(data);
        usart_send_03();
    })
}

pub fn usart_send_reset() {
    usart_send_text_transmission(b"\\RESET");
}

pub fn usart_send_V_packet(data: &[u8]) {
    if unsafe { SHOULD_SEND_USART_PACKET } {
        cortex_m::interrupt::free(|_v| {
            usart_send_02();
            usart_send_escaped_str(b"V");
            let len = usize::from(data[3]);
            usart_send_escaped_str(&data[2..2 + len]);
            usart_send_03();
        });
    }
}

pub fn usart_send_hid_report(report_size: usize) {
    let mut data = [0; 0x48];
    data[0] = b'P';
    crate::programming_mode::copy_hid_report(&mut data[1..]);
    usart_send_text_transmission(&data[..1 + report_size])
}