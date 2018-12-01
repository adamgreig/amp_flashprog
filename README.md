# AMP FlashProg

This is an alternate firmware for the [AMP](https://github.com/adamgreig/amp)
or BMP which uses it as a SPI flash programmer, interfaced over USB serial.

## Pinout

```ascii
          ______
  VTref --|1  2|-- SWDIO/TMS....FLASH MOSI
    GND --|3  4|-- SWDCLK/TCK...FLASH SCLK
    GND --|5  6|-- SWO/TDO......FLASH CS
        x-|7  8|-- TDI..........GPIO
    GND --|9 10|-- nRST.........FLASH MISO
          ------

```

GPIO is controlled over the serial port and may be used for e.g. holding an
FPGA in reset while writing to its flash memory.


## Protocol

The serial port transmits and receives  ASCII-coded data.

| Transmit | Effect |
|----------|--------|
| 0x67 `g` | Sets GPIO low |
| 0x47 `G` | Sets GPIO high |
| 0x73 `s` | Sets CS low |
| 0x53 `S` | Sets CS high-impedance (typically then pulled high) |
| 0-9, a-f, A-F | Data nibbles |
| Anything else | No effect |

Each nibble is transmitted upon reception and the four data bits received on
the SPI bus are then sent back as a hex character over the serial port.

The MOSI and SCLK pins are high impedance when not actively transmitting data,
with SCLK pulled up.
