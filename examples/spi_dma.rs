#![no_main]
#![no_std]

// Halt on panic
use panic_halt as _;

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use embedded_hal::spi::{Mode, Phase, Polarity};
use stm32f4xx_hal::pac::interrupt;
use stm32f4xx_hal::{
    dma::{config, traits::Stream, Channel0, MemoryToPeripheral, Stream4, StreamsTuple, Transfer},
    prelude::*,
    spi::*,
    stm32,
};

const ARRAY_SIZE: usize = 100;

type SpiDma = Transfer<
    Stream4<stm32::DMA1>,
    Channel0,
    Tx<stm32::SPI2>,
    MemoryToPeripheral,
    &'static mut [u8; ARRAY_SIZE],
>;

static G_TRANSFER: Mutex<RefCell<Option<SpiDma>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    if let Some(dp) = stm32::Peripherals::take() {
        // Set up the system clock.
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.freeze();

        let steams = StreamsTuple::new(dp.DMA1);
        let stream = steams.4;

        let gpiob = dp.GPIOB.split();
        let pb15 = gpiob.pb15.into_alternate_af5().internal_pull_up(true);
        let pb13 = gpiob.pb13.into_alternate_af5();

        let mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };

        let spi2 = Spi::spi2(dp.SPI2, (pb13, NoMiso, pb15), mode, 3_000_000.hz(), clocks);

        let buffer = cortex_m::singleton!(: [u8; ARRAY_SIZE] = [1; ARRAY_SIZE]).unwrap();

        for i in 0..ARRAY_SIZE {
            buffer[i] = i as u8;
        }

        let tx = spi2.use_dma().tx();

        let mut transfer = Transfer::init_memory_to_peripheral(
            stream,
            tx,
            buffer,
            None,
            config::DmaConfig::default()
                .memory_increment(true)
                .fifo_enable(true)
                .fifo_error_interrupt(true)
                .transfer_complete_interrupt(true),
        );

        transfer.start(|_tx| {});

        // Hand off transfer to interrupt handler
        cortex_m::interrupt::free(|cs| *G_TRANSFER.borrow(cs).borrow_mut() = Some(transfer));
        // Enable interrupt
        unsafe {
            cortex_m::peripheral::NVIC::unmask(stm32::Interrupt::DMA1_STREAM4);
        }
    }

    loop {
        cortex_m::asm::nop();
    }
}

#[interrupt]
fn DMA1_STREAM4() {
    static mut TRANSFER: Option<SpiDma> = None;

    let transfer = TRANSFER.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| G_TRANSFER.borrow(cs).replace(None).unwrap())
    });

    // Its important to clear fifo errors as the transfer is paused until it is cleared
    cortex_m::interrupt::free(|_cs| {
        if Stream4::<stm32::DMA1>::get_fifo_error_flag() {
            transfer.clear_fifo_error_interrupt();
        }
        if Stream4::<stm32::DMA1>::get_transfer_complete_flag() {
            transfer.clear_transfer_complete_interrupt();

            let buffer = cortex_m::singleton!(: [u8; ARRAY_SIZE] = [1; ARRAY_SIZE]).unwrap();
            for i in 0..ARRAY_SIZE {
                buffer[i] = (i + 1) as u8;
            }
            transfer.next_transfer(buffer).unwrap();
            cortex_m::peripheral::NVIC::mask(stm32::Interrupt::DMA1_STREAM5);
        }
    });
}
