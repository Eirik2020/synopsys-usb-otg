//! Target-specific definitions

use vcell::VolatileCell;

#[cfg(feature = "cortex-m")]
pub use cortex_m::interrupt;
#[cfg(feature = "riscv")]
pub use riscv::interrupt;

use crate::ral::register::RWRegister;
use crate::ral::{
    endpoint0_out, endpoint_in, endpoint_out, otg_device, otg_global, otg_global_dieptxfx,
    otg_pwrclk,
};
use crate::UsbPeripheral;

/// The most register reads a wait on the core makes before giving up. The
/// core's self-clearing bits (GRSTCTL's CSRST, RXFFLSH and TXFFLSH) clear in
/// "several clocks", RXFFLSH in 8 of the slower of the PHY and AHB clocks
/// (RM0368 22.16.2, OTG_FS_GRSTCTL; RM0433 57.14.5, OTG_GRSTCTL), and every read
/// of a core register is one AHB transfer, so this is thousands of times the
/// longest wait the manuals state while staying well under a millisecond.
pub const SPIN_LIMIT: u32 = 10_000;

/// Reads until `done` holds, at most [`SPIN_LIMIT`] times. Says whether it
/// held: a core that never clears its bit leaves the caller to go on, never
/// to hang.
pub fn spin_until(mut done: impl FnMut() -> bool) -> bool {
    for _ in 0..SPIN_LIMIT {
        if done() {
            return true;
        }
    }
    false
}

pub fn fifo_write(usb: UsbRegisters, channel: impl Into<usize>, buf: &[u8]) {
    let fifo = usb.fifo(channel.into());

    // Four bytes to a word, the last word padded with zeros.
    for chunk in buf.chunks(4) {
        let mut u32_bytes = [0u8; 4];
        for (byte, value) in u32_bytes.iter_mut().zip(chunk) {
            *byte = *value;
        }
        fifo.write(u32::from_ne_bytes(u32_bytes));
    }
}

pub fn fifo_read_into(usb: UsbRegisters, buf: &[VolatileCell<u32>]) {
    let fifo = usb.fifo(0);

    for p in buf {
        let word = fifo.read();
        p.set(word);
    }
}

/// Wrapper around device-specific peripheral that provides unified register interface
#[derive(Copy, Clone)]
pub struct UsbRegisters(usize);

impl UsbRegisters {
    #[inline(always)]
    pub fn new<USB: UsbPeripheral>() -> Self {
        Self(USB::REGISTERS as usize)
    }

    #[inline(always)]
    pub fn global(&self) -> &'static otg_global::RegisterBlock {
        unsafe { &*(self.0 as *const _) }
    }

    #[inline(always)]
    pub fn device(&self) -> &'static otg_device::RegisterBlock {
        unsafe { &*((self.0 + 0x800) as *const _) }
    }

    #[inline(always)]
    pub fn pwrclk(&self) -> &'static otg_pwrclk::RegisterBlock {
        unsafe { &*((self.0 + 0xe00) as *const _) }
    }

    #[inline(always)]
    pub fn fifo(&self, channel: usize) -> &'static RWRegister<u32> {
        // Sixteen FIFO windows; callers pass an endpoint number below
        // `ENDPOINT_COUNT`, so the mask only keeps the address in the block.
        let address = self.0 + 0x1000 + (channel & 0xf) * 0x1000;
        unsafe { &*(address as *const RWRegister<u32>) }
    }

    #[inline(always)]
    pub fn dieptxfx(&self, index: usize) -> &'static otg_global_dieptxfx::RegisterBlock {
        let address = self.0 + 0x100 + 4 * index;
        unsafe { &*(address as *const _) }
    }

    #[inline(always)]
    pub fn endpoint_in(&self, index: usize) -> &'static endpoint_in::RegisterBlock {
        let address = self.0 + 0x900 + 0x20 * index;
        unsafe { &*(address as *const _) }
    }

    #[inline(always)]
    pub fn endpoint0_out(&self) -> &'static endpoint0_out::RegisterBlock {
        let address = self.0 + 0xb00;
        unsafe { &*(address as *const _) }
    }

    #[inline(always)]
    pub fn endpoint_out(&self, index: usize) -> &'static endpoint_out::RegisterBlock {
        let address = self.0 + 0xb00 + 0x20 * index;
        unsafe { &*(address as *const _) }
    }
}
