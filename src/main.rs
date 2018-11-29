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

#[entry]
fn main() -> ! {
    let dp = stm32f103xx::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr
        .hse(8.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .freeze(&mut flash.acr);

    assert!(clocks.usbclk_valid());

    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);

    // Set USB pullup on this hardware
    let mut pa8 = gpioa.pa8.into_push_pull_output(&mut gpioa.crh);
    pa8.set_high();

    let usb_bus = UsbBus::usb(dp.USB, &mut rcc.apb1);
    usb_bus.borrow_mut().enable_reset(&clocks, &mut gpioa.crh, gpioa.pa12);

    let serial = cdc_acm::SerialPort::new(&usb_bus);

    let usb_dev = UsbDevice::new(&usb_bus, UsbVidPid(0x5824, 0x27dd))
        .manufacturer("AGG")
        .product("AMP FlashProg")
        .serial_number("0000")
        .device_class(cdc_acm::USB_CLASS_CDC)
        .build(&[&serial]);

    usb_dev.force_reset().expect("reset failed");

    loop {
        usb_dev.poll();

        if usb_dev.state() == UsbDeviceState::Configured {

            let mut buf = [0u8; 8];

            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {

                    // Echo back in upper case
                    for c in buf[0..count].iter_mut() {
                        if 0x61 <= *c && *c <= 0x7a {
                            *c &= !0x20;
                        }
                    }

                    serial.write(&buf[0..count]).ok();
                },
                _ => { },
            }
        }
    }
}
