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

### SPI Read/Write

Each SPI transaction sends a number of bytes of data to the SPI device and
receives the same number back. The AMP will assert CS for the duration of the
transaction and de-assert it once the transaction is complete.

Over the USB-CDC serial port, begin a transaction with the `>` (GREATER-THAN
SIGN, 0x3E) character. This will cause CS to become asserted. Send each byte to
be transmitted as ASCII-coded hex pairs (e.g. to transmit the byte 0x41, send
`4` (0x34) and `1` (0x31) over the serial port). End the transaction with `\n`
(LINEFEED, 0x0A). This will de-assert CS.

While transmitting data, the device will begin to respond with `<` (LESS-THAN
SIGN, 0x3C) followed by the data received for each transmitted byte, again as
hex-coded ASCII. It will terminate the received data with `\n` at the end of
the transaction.

Transactions may have arbitrary lengths.

### GPIO Control

To set the GPIO pin low, transmit `g` (0x67) over the serial port. To set the
GPIO pin high, transmit `G` (0x47). These commands may be given at any time.
