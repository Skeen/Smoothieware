# Overview
This repository features a 'No SD card' version of Smoothieware, which adds a
new device target; `lpc1768_NOSD`, this disables the SD card code, and links
the output file with a new linker script.

This fork has been built with inspiration from;

* http://smoothieware.org/flashing-the-bootloader
* https://github.com/nullsub/Smoothieware
* https://gist.github.com/nullsub/10f4551eb0f3e2422409

# Usage
To build the 'No SD card' target, run;
```bash
DEVICES=lpc1768_NOSD make -j8 all
```

To flash (using [lpc21isp](http://sourceforge.net/projects/lpc21isp/)), run;
```bash
DEVICES=lpc1768_NOSD CONSOLE=/dev/ttyUSB0 make flash
```
Where `/dev/ttyUSB0` is the tty device to use.

Before flashing the chip must enter Serial Bootloader mode, to enter this mode;

* Press the RESET button
* Press the BOOTLOADER (ISP) button
* Release the RESET button
* Release the BOOTLOADER (ISP) button

# Smoothieware
Smoothie is a free, opensource, high performance G-code interpreter and CNC controller written in Object-Oriented C++ for the LPC17xx micro-controller ( ARM Cortex M3 architecture ). It will run on a mBed, a LPCXpresso, a SmoothieBoard, R2C2 or any other LPC17xx-based board. The motion control part is a port of the awesome grbl.

Documentation can be found here : http://smoothieware.org/

For information about Smoothieware itself, head over to [Smoothieware/Smoothieware](https://github.com/Smoothieware/Smoothieware)
