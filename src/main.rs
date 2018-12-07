#![no_std]
#![no_main]

#[macro_use(entry)]
extern crate cortex_m_rt;
extern crate cortex_m;
extern crate panic_halt;
extern crate stm32f103xx_hal as hal;
extern crate usb_device;
extern crate stm32f103xx_usb;
extern crate embedded_hal;

use hal::prelude::*;
use hal::stm32f103xx;
use embedded_hal::digital::{InputPin, OutputPin};

use usb_device::prelude::*;
use stm32f103xx_usb::UsbBus;

mod cdc_acm;

/// Bitbang SPI transmission for one nibble of data.
///
/// Returns the four received bits.
fn transmit_nibble<I, O, C>(inpin: &I, outpin: &mut O, clkpin: &mut C, nibble: u8) -> u8
where I: InputPin, O: OutputPin, C: OutputPin
{
    let mut rx = 0u8;
    for clk in (0..4).rev() {
        if nibble >> clk & 1 == 1 {
            outpin.set_high();
        } else {
            outpin.set_low();
        }
        clkpin.set_low();
        cortex_m::asm::delay(1);

        rx |= (inpin.is_high() as u8) << clk;
        clkpin.set_high();
        cortex_m::asm::delay(1);
    }
    rx
}

enum Mode {
    Flash,
    FPGA,
}

#[entry]
fn main() -> ! {
    let dp = stm32f103xx::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr
        .use_hse(8.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .freeze(&mut flash.acr);

    assert!(clocks.usbclk_valid());

    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);

    // Grab SPI pins
    let mut mosi = gpioa.pa4.into_floating_input(&mut gpioa.crl);
    let mut miso = gpioa.pa2.into_floating_input(&mut gpioa.crl);
    let mut sclk = gpioa.pa5.into_pull_up_input(&mut gpioa.crl);
    let mut gpio = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
    let mut cs   = gpioa.pa6.into_open_drain_output(&mut gpioa.crl);

    // Set TPWR to an output and high to disable target power
    let mut tpwr = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);
    tpwr.set_high();

    // Set USB pullup on this hardware to begin USB enumeration
    let mut pa8 = gpioa.pa8.into_push_pull_output(&mut gpioa.crh);
    pa8.set_high();

    // Set up USB peripheral as a CDC class device
    let usb_bus = UsbBus::usb(dp.USB, &mut rcc.apb1);
    let serial  = cdc_acm::SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDevice::new(&usb_bus, UsbVidPid(0x5824, 0x27dd))
        .manufacturer("AGG")
        .product("AMP FlashProg")
        .serial_number("0000")
        .device_class(cdc_acm::USB_CLASS_CDC)
        .build(&[&serial]);

    // USB transmit and receive buffers
    let mut usb_rxbuf = [0u8; 64];
    let mut usb_txbuf = [0u8; 64];
    let mut usb_txidx = 0usize;

    let mut mode = Mode::Flash;

    loop {
        usb_dev.poll();

        // If we're not currently connected to a computer, just keep polling
        if usb_dev.state() != UsbDeviceState::Configured {
            continue;
        }

        // See if there's any data to read
        match serial.read(&mut usb_rxbuf) {
            Ok(count) if count > 0 => {
                // Iterate each incoming byte
                for c in usb_rxbuf[0..count].iter() {

                    match c {
                        // 'g': set GPIO low
                        0x67 => {
                            gpio.set_low();
                        },

                        // 'G': set GPIO high
                        0x47 => {
                            gpio.set_high();
                        }

                        // 's': set CS low
                        0x73 => {
                            cs.set_low();
                        }

                        // 'S': set CS hi-z
                        0x53 => {
                            cs.set_high();
                        }

                        // 'm': set mode to flash
                        0x6d => {
                            mode = Mode::Flash;
                        }

                        // 'M': set mode to FPGA
                        0x4d => {
                            mode = Mode::FPGA;
                        }

                        // 'p': set power off
                        0x70 => {
                            tpwr.set_high();
                        }

                        // 'P': set power on
                        0x40 => {
                            tpwr.set_low();
                        }

                        // Hex nibble: transmit
                        0x30...0x39 | 0x41...0x46 | 0x61...0x66 => {
                            // Decode hex nibble
                            let tx = match c {
                                0x30...0x39 => *c - 0x30,
                                0x41...0x46 => *c - 0x41 + 10,
                                0x61...0x66 => *c - 0x61 + 10,
                                _ => unreachable!(),
                            };

                            // Set up SCLK as output
                            let mut sclk_out = sclk.into_push_pull_output(&mut gpioa.crl);
                            sclk_out.set_high();

                            // Transmit and receive a nibble.
                            let rx = match mode {
                                Mode::Flash => {
                                    // Set up MOSI as output
                                    let mut outpin = mosi.into_push_pull_output(&mut gpioa.crl);

                                    let rx = transmit_nibble(&miso, &mut outpin, &mut sclk_out, tx);

                                    // Return MOSI
                                    mosi = outpin.into_floating_input(&mut gpioa.crl);

                                    rx
                                },
                                Mode::FPGA  => {
                                    // Set up MISO as output
                                    let mut outpin = miso.into_push_pull_output(&mut gpioa.crl);

                                    let rx = transmit_nibble(&mosi, &mut outpin, &mut sclk_out, tx);

                                    // Return MISO
                                    miso = outpin.into_floating_input(&mut gpioa.crl);

                                    rx
                                },
                            };

                            // Return SCLK
                            sclk = sclk_out.into_pull_up_input(&mut gpioa.crl);

                            // Encode received data to hex and enqueue for transmission
                            usb_txbuf[usb_txidx] = match rx {
                                0...9   => rx + 0x30,
                                10...15 => rx - 10 + 0x61,
                                _ => unreachable!(),
                            };
                            usb_txidx += 1;
                        }

                        // Ignore any other received byte
                        _ => {},
                    }
                }

                // Transmit any queued data over USB
                if usb_txidx > 0 {
                    serial.write(&usb_txbuf[0..usb_txidx]).ok();
                    usb_txidx = 0;
                }
            },

            // No action if there's no received data
            _ => { },
        }
    }
}
