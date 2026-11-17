#![allow(dead_code)]
use crate::target::{fifo_read_into, UsbRegisters};
use crate::UsbPeripheral;
use core::marker::PhantomData;
use core::slice;
use usb_device::{Result, UsbError};
use vcell::VolatileCell;

#[derive(Eq, PartialEq)]
pub enum EndpointBufferState {
    Empty,
    DataOut,
    DataSetup,
}

pub struct EndpointBuffer {
    buffer: &'static mut [VolatileCell<u32>],
    data_size: u16,
    has_data: bool,
    is_setup: bool,
}

impl EndpointBuffer {
    pub fn new(buffer: &'static mut [u32]) -> Self {
        Self {
            buffer: unsafe { &mut *(buffer as *mut [u32] as *mut [VolatileCell<u32>]) },
            data_size: 0,
            has_data: false,
            is_setup: false,
        }
    }

    pub fn read_packet(&mut self, buf: &mut [u8]) -> Result<usize> {
        if !self.has_data {
            return Err(UsbError::WouldBlock);
        }

        let data_size = self.data_size as usize;
        let (Some(out), Some(words)) = (
            buf.get_mut(..data_size),
            self.buffer.get(..(data_size + 3) / 4),
        ) else {
            return Err(UsbError::BufferOverflow);
        };

        // Each word holds four bytes of the packet, the last one fewer.
        for (chunk, word) in out.chunks_mut(4).zip(words) {
            let bytes = word.get().to_ne_bytes();
            for (byte, value) in chunk.iter_mut().zip(bytes) {
                *byte = value;
            }
        }

        self.has_data = false;

        Ok(data_size)
    }

    pub fn fill_from_fifo(
        &mut self,
        usb: UsbRegisters,
        data_size: u16,
        is_setup: bool,
    ) -> Result<()> {
        if self.has_data {
            return Err(UsbError::WouldBlock);
        }

        let Some(words) = self.buffer.get(..(data_size as usize + 3) / 4) else {
            return Err(UsbError::BufferOverflow);
        };
        fifo_read_into(usb, words);

        self.is_setup = is_setup;
        self.data_size = data_size;
        self.has_data = true;

        Ok(())
    }

    pub fn state(&self) -> EndpointBufferState {
        if self.has_data {
            if self.is_setup {
                EndpointBufferState::DataSetup
            } else {
                EndpointBufferState::DataOut
            }
        } else {
            EndpointBufferState::Empty
        }
    }

    pub fn capacity(&self) -> usize {
        self.buffer.len() * 4
    }
}

impl Default for EndpointBuffer {
    fn default() -> Self {
        EndpointBuffer::new(&mut [])
    }
}

pub struct EndpointMemoryAllocator<USB> {
    next_free_offset: usize,
    max_size_words: usize,
    memory: &'static mut [u32],
    tx_fifo_size_words: [u16; 9],
    _marker: PhantomData<USB>,
}

impl<USB: UsbPeripheral> EndpointMemoryAllocator<USB> {
    pub fn new(memory: &'static mut [u32]) -> Self {
        Self {
            next_free_offset: 0,
            max_size_words: 0,
            memory,
            tx_fifo_size_words: [0; 9],
            _marker: PhantomData,
        }
    }

    pub fn allocate_rx_buffer(&mut self, size: usize) -> Result<EndpointBuffer> {
        let size_words = (size + 3) / 4;

        let offset = self.next_free_offset;
        if size_words > self.memory.len().saturating_sub(offset) {
            return Err(UsbError::EndpointMemoryOverflow);
        }
        // The receive FIFO grows with every OUT buffer (`configure_all` sizes
        // it as their sum plus 30 words), so it must still fit beside the
        // transmit FIFOs already placed.
        if self.fifo_words_used() + size_words > USB::FIFO_DEPTH_WORDS {
            return Err(UsbError::EndpointMemoryOverflow);
        }

        self.next_free_offset += size_words;
        self.max_size_words = core::cmp::max(self.max_size_words, size_words);

        let buffer = unsafe {
            let ptr = self.memory.as_mut_ptr().add(offset);
            slice::from_raw_parts_mut(ptr, size_words)
        };
        Ok(EndpointBuffer::new(buffer))
    }

    pub fn allocate_tx_buffer(&mut self, ep_number: u8, size: usize) -> Result<()> {
        let Some(&allocated) = self.tx_fifo_size_words.get(ep_number as usize) else {
            return Err(UsbError::InvalidEndpoint);
        };
        if allocated != 0 {
            return Err(UsbError::InvalidEndpoint);
        }

        let used = self.fifo_words_used();

        let size_words = core::cmp::max((size + 3) / 4, 16);
        if (used + size_words) > USB::FIFO_DEPTH_WORDS {
            return Err(UsbError::EndpointMemoryOverflow);
        }

        if let Some(slot) = self.tx_fifo_size_words.get_mut(ep_number as usize) {
            *slot = size_words as u16;
        }

        Ok(())
    }

    /// The FIFO RAM in words that `configure_all` lays out for what is
    /// allocated so far: the receive FIFO (OUT buffers plus 30 words) and
    /// every transmit FIFO.
    fn fifo_words_used(&self) -> usize {
        30 + self.next_free_offset
            + self
                .tx_fifo_size_words
                .iter()
                .map(|&words| words as usize)
                .sum::<usize>()
    }

    /// Returns the size of memory allocated for OUT endpoints in words
    pub fn total_rx_buffer_size_words(&self) -> u16 {
        self.next_free_offset as u16
    }

    pub fn tx_fifo_size_words(&self, ep_number: usize) -> u16 {
        self.tx_fifo_size_words
            .get(ep_number)
            .copied()
            .unwrap_or(0)
    }

    pub fn max_buffer_size_words(&self) -> usize {
        self.max_size_words
    }
}
