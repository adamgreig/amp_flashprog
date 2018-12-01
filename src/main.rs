#![no_std]
#![no_main]

#[macro_use(entry)]
extern crate cortex_m_rt;
extern crate cortex_m;
extern crate panic_halt;
extern crate stm32f103xx_hal as hal;
extern crate usb_device;
extern crate stm32f103xx_usb;

use hal::prelude::*;
use hal::stm32f103xx;

use usb_device::prelude::*;
use stm32f103xx_usb::UsbBus;

mod cdc_acm;

fn delay_clk() {
    cortex_m::asm::delay(1);
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

    // Set USB pullup on this hardware
    let mut pa8 = gpioa.pa8.into_push_pull_output(&mut gpioa.crh);
    pa8.set_high();

    // Grab SPI pins
    let mut mosi = gpioa.pa4.into_floating_input(&mut gpioa.crl);
    let miso = gpioa.pa2.into_floating_input(&mut gpioa.crl);
    let mut sclk = gpioa.pa5.into_pull_up_input(&mut gpioa.crl);
    let mut gpio = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
    let mut cs = gpioa.pa6.into_open_drain_output(&mut gpioa.crl);

    cs.set_high();

    let usb_bus = UsbBus::usb(dp.USB, &mut rcc.apb1);
    let serial = cdc_acm::SerialPort::new(&usb_bus);

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
                for c in usb_rxbuf[0..count].iter_mut() {
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

                        // Hex nibble: transmit
                        0x30...0x39 | 0x41...0x46 | 0x61...0x66 => {
                            // Decode hex nibble
                            let tx = match c {
                                0x30...0x39 => *c - 0x30,
                                0x41...0x46 => *c - 0x41 + 10,
                                0x61...0x66 => *c - 0x61 + 10,
                                _ => unreachable!(),
                            };
                            let mut rx = 0u8;

                            // Set MOSI and SCLK to outputs
                            let mut mosi_out = mosi.into_push_pull_output(&mut gpioa.crl);
                            let mut sclk_out = sclk.into_push_pull_output(&mut gpioa.crl);
                            sclk_out.set_high();

                            // Process each bit
                            for clk in (0..4).rev() {
                                if tx >> clk & 1 == 1 {
                                    mosi_out.set_high();
                                } else {
                                    mosi_out.set_low();
                                }
                                sclk_out.set_low();
                                delay_clk();

                                if miso.is_high() {
                                    rx |= 1 << clk;
                                }
                                sclk_out.set_high();
                                delay_clk();
                            }

                            // Set MOSI and SCLK back to inputs
                            mosi = mosi_out.into_floating_input(&mut gpioa.crl);
                            sclk = sclk_out.into_pull_up_input(&mut gpioa.crl);

                            // Encode received data to hex and enqueue for transmission
                            usb_txbuf[usb_txidx] = match rx {
                                0...9   => rx + 0x30,
                                10...15 => rx + 0x61,
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
