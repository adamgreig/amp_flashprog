// Minimal CDC-ACM implementation
//
// This file is copied from:
// https://github.com/mvirkkunen/stm32f103xx-usb/blob/master/examples/serial.rs
// where it is Copyright (c) 2018 Matti Virkkunen and released under the MIT license.
//

/// Minimal CDC-ACM implementation for the examples - this will eventually be a real crate!

use core::cell::Cell;

use usb_device::class_prelude::*;
use usb_device::Result;

pub const USB_CLASS_CDC: u8 = 0x02;
const USB_CLASS_DATA: u8 = 0x0a;
const CDC_SUBCLASS_ACM: u8 = 0x02;
const CDC_PROTOCOL_AT: u8 = 0x01;

const CS_INTERFACE: u8 = 0x24;
const CDC_TYPE_HEADER: u8 = 0x00;
const CDC_TYPE_CALL_MANAGEMENT: u8 = 0x01;
const CDC_TYPE_ACM: u8 = 0x02;
const CDC_TYPE_UNION: u8 = 0x06;

const REQ_SET_LINE_CODING: u8 = 0x20;
const REQ_SET_CONTROL_LINE_STATE: u8 = 0x22;

pub struct SerialPort<'a, B: 'a + UsbBus + Sync> {
    comm_if: InterfaceNumber,
    comm_ep: EndpointIn<'a, B>,
    data_if: InterfaceNumber,
    read_ep: EndpointOut<'a, B>,
    write_ep: EndpointIn<'a, B>,
    need_zip: Cell<bool>,
}

impl<'a, B: UsbBus + Sync> SerialPort<'a, B> {
    pub fn new(bus: &'a UsbBusWrapper<B>) -> SerialPort<'a, B> {
        SerialPort {
            comm_if: bus.interface(),
            comm_ep: bus.interrupt(8, 255),
            data_if: bus.interface(),
            read_ep: bus.bulk(64),
            write_ep: bus.bulk(64),
            need_zip: Cell::new(false),
        }
    }

    pub fn write(&self, data: &[u8]) -> Result<usize> {
        if self.need_zip.get() {
            return Ok(0);
        }

        if data.len() == 64 {
            self.need_zip.set(true);
        }

        match self.write_ep.write(data) {
            Ok(count) => Ok(count),
            Err(UsbError::Busy) => Ok(0),
            e => e,
        }
    }

    pub fn read(&self, data: &mut [u8]) -> Result<usize> {
        match self.read_ep.read(data) {
            Ok(count) => Ok(count),
            Err(UsbError::NoData) => Ok(0),
            e => e,
        }
    }
}

impl<'a, B: UsbBus + Sync> UsbClass for SerialPort<'a, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        // TODO: make a better DescriptorWriter to make it harder to make invalid descriptors
        writer.interface(
            self.comm_if,
            1,
            USB_CLASS_CDC,
            CDC_SUBCLASS_ACM,
            CDC_PROTOCOL_AT)?;

        writer.write(
            CS_INTERFACE,
            &[CDC_TYPE_HEADER, 0x10, 0x01])?;

        writer.write(
            CS_INTERFACE,
            &[CDC_TYPE_CALL_MANAGEMENT, 0x00, self.data_if.into()])?;

        writer.write(
            CS_INTERFACE,
            &[CDC_TYPE_ACM, 0x00])?;

        writer.write(
            CS_INTERFACE,
            &[CDC_TYPE_UNION, self.comm_if.into(), self.data_if.into()])?;

        writer.endpoint(&self.comm_ep)?;

        writer.interface(
            self.data_if,
            2,
            USB_CLASS_DATA,
            0x00,
            0x00)?;

        writer.endpoint(&self.write_ep)?;
        writer.endpoint(&self.read_ep)?;

        Ok(())
    }

    fn endpoint_in_complete(&self, addr: EndpointAddress) {
        if self.need_zip.get() && addr == self.write_ep.address() {
            self.need_zip.set(false);
            self.write_ep.write(&[]).ok();
        }
    }

    fn control_out(&self, req: &control::Request, buf: &[u8]) -> ControlOutResult {
        let _ = buf;

        if req.request_type == control::RequestType::Class
            && req.recipient == control::Recipient::Interface
        {
            return match req.request {
                REQ_SET_LINE_CODING => ControlOutResult::Ok,
                REQ_SET_CONTROL_LINE_STATE => ControlOutResult::Ok,
                _ => ControlOutResult::Ignore,
            };
        }

        ControlOutResult::Ignore
    }
}
