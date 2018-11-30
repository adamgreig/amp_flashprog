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
    let mut sclk = gpioa.pa5.into_floating_input(&mut gpioa.crl);
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

    let mut last_nibble = None;

    loop {
        usb_dev.poll();

        if usb_dev.state() == UsbDeviceState::Configured {

            let mut rxbuf = [0u8; 64];
            let mut txbuf = [0u8; 64];
            let mut txidx = 0usize;

            match serial.read(&mut rxbuf) {
                Ok(count) if count > 0 => {

                    for c in rxbuf[0..count].iter_mut() {
                        match c {
                            // 'g'
                            0x67 => {
                                gpio.set_low();
                            },

                            // 'G'
                            0x47 => {
                                gpio.set_high();
                            }

                            // 's'
                            0x73 => {
                                cs.set_low();
                            }

                            // 'S'
                            0x53 => {
                                cs.set_high();
                            }

                            // \n
                            0x0A => {
                                last_nibble = None;
                            }

                            0x30...0x39 | 0x41...0x46 | 0x61...0x66 => {
                                let nibble = match c {
                                    0x30...0x39 => *c - 0x30,
                                    0x41...0x46 => *c - 0x41 + 10,
                                    0x61...0x66 => *c - 0x61 + 10,
                                    _ => unreachable!(),
                                };
                                match last_nibble {
                                    None => last_nibble = Some(nibble),
                                    Some(n) => {
                                        let tx = n << 4 | nibble;
                                        last_nibble = None;
                                        let mut rx = 0u8;
                                        let mut mosi_out = mosi.into_push_pull_output(&mut gpioa.crl);
                                        let mut sclk_out = sclk.into_push_pull_output(&mut gpioa.crl);
                                        for clk in 0..8 {
                                            let txbit = (tx >> (7 - clk)) & 1;
                                            sclk_out.set_low();
                                            if txbit == 1 {
                                                mosi_out.set_high();
                                            } else {
                                                mosi_out.set_low();
                                            }
                                            delay_clk();
                                            if miso.is_high() {
                                                rx |= 1 << (7 - clk);
                                            }
                                            sclk_out.set_high();
                                            delay_clk();
                                        }
                                        txbuf[txidx] = rx;
                                        txidx += 1;
                                        mosi = mosi_out.into_floating_input(&mut gpioa.crl);
                                        sclk = sclk_out.into_floating_input(&mut gpioa.crl);
                                    }
                                }
                            }

                            _ => {},
                        }
                    }
                    if txidx > 0 {
                        serial.write(&txbuf[0..txidx]).ok();
                    }
                },
                _ => { },
            }
        }
    }
}
