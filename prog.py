#!/usr/bin/env python3

import time
import math
import struct
import binascii
import tempfile
import argparse

import serial
from tqdm import trange


class Programmer:
    def __init__(self, port):
        self.serial = serial.Serial(port, timeout=0)

    def reset(self):
        self.serial.write(b"g")
        self.serial.flush()

    def unreset(self):
        self.serial.write(b"G")
        self.serial.flush()

    def select(self):
        self.serial.write(b"s")
        self.serial.flush()

    def unselect(self):
        self.serial.write(b"S")
        self.serial.flush()

    def fpga_mode(self):
        self.serial.write(b"M")
        self.serial.flush()

    def flash_mode(self):
        self.serial.write(b"m")
        self.serial.flush()

    def write(self, data):
        # Flush cache
        self.serial.read(1024)
        # Store received data
        rx = b""
        # Send hex-coded data in four-byte chunks
        tx = binascii.b2a_hex(data)
        for idx in range(0, len(tx), 8):
            # Transmit four bytes (8 hex nibbles)
            txdata = tx[idx:idx+8]
            self.serial.write(txdata)
            self.serial.flush()
            time.sleep(0.001)
            # Read response
            rxdata = self.serial.read(len(txdata))
            rx += rxdata
        if len(rx) != len(tx):
            print("Warning: Did not receive as many bytes as transmitted")
        return binascii.a2b_hex(rx)


class Flash:
    def __init__(self, programmer):
        self.programmer = programmer

    def read_id(self):
        self.programmer.flash_mode()
        # Hold FPGA in reset until we're done
        self.programmer.reset()
        # Wake up flash and check we can read its ID
        self.power_up()
        manufacturer, device = self.read_manufacturer()
        unique_id = self.read_unique_id()
        print(f"Flash: Manufacturer {manufacturer:02X}, device {device:02X}")
        print(f"       Unique ID: {unique_id}")

    def program(self, data, lma):
        self.read_id()

        # Erase enough space for the data to program
        print("Erasing flash...")
        self.erase_for_data(lma, len(data))

        # Write new image
        print("Programming flash...")
        self.program_data(lma, data)

        # Readback programmed data
        print("Verifying flash...")
        programmed = self.fast_read(lma, len(data))

        if programmed == data:
            print("Readback successful.")
        else:
            print("Error: Readback unsuccessful.")
            with tempfile.NamedTemporaryFile(delete=False) as f:
                f.write(programmed)
                print(f"Readback data stored in {f.name}")

    def erase_for_data(self, lma, length):
        # Adjust LMA to be 64K-block aligned
        length += (lma & 0xFFFF)
        lma &= 0xFF0000
        blocks = math.ceil(length / (64*1024))
        for block in trange(blocks):
            self.write_enable()
            self.block_erase_64k(lma + block * 64 * 1024)
            self.wait_while_busy()

    def program_data(self, lma, data):
        # Pad data to obtain 256B page alignment
        data = b"\xFF" * (lma & 0xFF) + data
        lma &= 0xFFFF00
        pages = math.ceil(len(data) / 256)
        for page in trange(pages):
            self.write_enable()
            self.page_program(lma + page * 256, data[page*256:(page+1)*256])
            self.wait_while_busy()

    def power_down(self):
        self._write(0xB9)

    def power_up(self):
        self._write(0xAB)

    def write_enable(self):
        self._write(0x06)

    def write_disable(self):
        self._write(0x04)

    def fast_read(self, address, length):
        length += 1
        address = self._pack_address(address)
        return self._read(0x0B, length, address)

    def page_program(self, address, data):
        assert 1 <= len(data) <= 256
        address = self._pack_address(address)
        self._write(0x02, address + data)

    def sector_erase(self, address):
        address = self._pack_address(address)
        self._write(0x20, address)

    def block_erase_32k(self, address):
        address = self._pack_address(address)
        self._write(0x52, address)

    def block_erase_64k(self, address):
        address = self._pack_address(address)
        self._write(0xD8, address)

    def chip_erase(self):
        self._write(0xC7)

    def read_manufacturer(self):
        data = self._read(0x90, 3+2)
        manufacturer, device = struct.unpack("BB", data[3:])
        return manufacturer, device

    def read_unique_id(self):
        data = self._read(0x4B, 4+8)
        unique_id = binascii.b2a_hex(data[4:])
        return unique_id.decode()

    def read_status1(self):
        return struct.unpack("B", self._read(0x05, 1))[0]

    def read_status2(self):
        return struct.unpack("B", self._read(0x35, 1))[0]

    def is_busy(self):
        return self.read_status1() & 1 == 1

    def wait_while_busy(self):
        while self.is_busy():
            time.sleep(0.05)

    def _read(self, command, nbytes, arguments=b""):
        """
        Issue command `command` (integer) followed by `arguments`,
        then read `nbytes` of subsequent data.
        """
        padding = b"\x00" * nbytes
        tx = struct.pack("B", command) + arguments + padding
        self.programmer.flash_mode()
        self.programmer.select()
        rx = self.programmer.write(tx)
        self.programmer.unselect()
        return rx[1:]

    def _write(self, command, data=b""):
        """
        Issue command `command` (integer) and write `data` subsequently.
        """
        tx = struct.pack("B", command) + data
        self.programmer.flash_mode()
        self.programmer.select()
        rx = self.programmer.write(tx)
        self.programmer.unselect()
        return rx[1:]

    def _pack_address(self, address):
        return struct.pack(">I", address)[1:]


class FPGA:
    def __init__(self, programmer):
        self.programmer = programmer

    def program(self, data):
        print("Programming FPGA...")
        # Bring FPGA into reset
        self.programmer.reset()
        # Power down attached flash (if not already powered down)
        flash = Flash(self.programmer)
        flash.power_down()
        # Release FPGA from reset in slave SPI mode
        self.programmer.fpga_mode()
        self.programmer.select()
        self.programmer.unreset()
        # Wait for FPGA to come out of reset
        time.sleep(0.01)
        # Send 8 dummy clocks with CS high then assert CS again
        self.programmer.unselect()
        self.programmer.write(b"\x00\x00")
        self.programmer.select()
        # Send configuration image
        for idx in trange(0, len(data), 4, unit='B', unit_scale=4):
            self.programmer.write(data[idx:idx+4])
        # Release CS and wait for configuration to be complete
        self.programmer.unselect()
        self.programmer.write(b"\x00" * 40)
        print("Programming complete.")


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--port",
        help="Serial port to use",
        default="/dev/serial/by-id/usb-AGG_AMP_FlashProg_0000-if00")
    parser.add_argument(
        "--lma",
        help="Load memory address (flash only)",
        default="0")
    parser.add_argument(
        "--fpga",
        help="Program FPGA directly (skip flash)",
        action='store_true')
    parser.add_argument(
        "--read-flash-id",
        help="Just read flash ID",
        action='store_true')
    parser.add_argument(
        "bin",
        help="binfile to program")
    return parser.parse_args()


def main():
    args = get_args()
    with open(args.bin, "rb") as f:
        data = f.read()
    lma = int(args.lma, 0)
    prog = Programmer(args.port)
    if args.read_flash_id:
        flash = Flash(prog)
        flash.read_id()
    elif args.fpga:
        fpga = FPGA(prog)
        fpga.program(data)
    else:
        flash = Flash(prog)
        flash.program(data, lma)


if __name__ == "__main__":
    main()
