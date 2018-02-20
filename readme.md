---
---
# Down the STM32 Bare Metal Rabbit Hole

## Introduction

This repository is a collection of my code projects as I learn the aspects of programming an STM32F103C8 Cortex M3 micro controller at the "bare metal" register level.  I will be creating example code for most all aspects of the µC including SPI, USART, I2C, DMA, etc.

With a working toolchain, all projects can be built from within their project directory.  The `master.mk` file **REQUIRES** modification in order to set the paths to the build tools and CMSIS location.

The following tools are used for these projects:
* [ARM-GCC](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) compiler toolchain.
* [stm32flash](https://sourceforge.net/projects/stm32flash/) flash tool using the on-board STM32 serial bootloader over UART.
* [st-link](https://github.com/texane/stlink) flash tool using an ST-LINK V2 USB programmer.
* [CMSIS](https://github.com/getoffmyhack/STM32-CMSIS) my 'repackaged' STM32 CMSIS files.
* [Official STM32 CMSIS](http://www.st.com/en/embedded-software/stm32cube-mcu-packages.html) files as part of their STM32Cube MCU packages.

### Development Hardware

All example projects are using a ["Blue Pill"][blue pill] board as the target hardware platform.

### Common Directories

#### [template/](template)

The template directory contains the files necessary to start a new STM32F103 project.

#### [gdb/](gdb)

The gdb directory contains gbd command files used for debugging and flashing.

### Example Projects

#### [01 Hello World/](01_Hello_World)

The first project being the ubiquitous "Hello World" as a "Blinken Light".

[blue pill]: http://wiki.stm32duino.com/index.php?title=Blue_Pill