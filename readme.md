---
---
# Down the STM32 Bare Metal Rabbit Hole

## Introduction

This repository is a collection of my code projects as I learn the aspects of programming an STM32F103C8 Cortex M3 micro controller at the "bare metal" register level.  I will be creating example code for most all aspects of the µC including SPI, USART, I2C, DMA, etc.

With a working toolchain, all projects can be built from within their project directory.  The `master.mk` file **REQUIRES** modification in order to set the paths to the build tools.

The following tools are used for these projects:
* [ARM-GCC](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) compiler toolchain.
* [stm32flash](https://sourceforge.net/projects/stm32flash/) flash tool using the on-board STM32 serial bootloader over UART.
* [st-link](https://github.com/texane/stlink) flash tool using an ST-LINK V2 USB programmer.
* [Official STM32 CMSIS](http://www.st.com/en/embedded-software/stm32cube-mcu-packages.html) files as part of their STM32Cube MCU packages.

### Development Hardware

All example projects are using a ["Blue Pill"][blue pill] board as the target hardware platform.

### Common Directories

#### [CMSIS/](CMSIS)

Contains the [Cortex CMSIS](https://developer.arm.com/embedded/cmsis) files needed to build the projects.  These files have been repackaged from the official [STM32 CMSIS](http://www.st.com/en/embedded-software/stm32cube-mcu-packages.html) files.

#### [template/](template)

The template directory contains the files necessary to start a new STM32F103 project.

#### [gdb/](gdb)

The gdb directory contains gbd command files used for debugging and flashing.

### Example Projects

#### [01 Hello World/](01_Hello_World)

The first project being the ubiquitous "Hello World" as a "Blinken Light".

#### [02 Warp Speed/](02_Warp_Speed)

Extends "Hello World" to configure the clock to run at max speed (72MHz.)

#### [03 Tick Tock/](03_Tick_Tock)

This extends the previous projects implementing the Cortex SysTick timer in order to create a 1ms interrupt.  This interrupt will become the clock used to time the LED heartbeat.  This becomes the code for the project template.

#### [04 USART/](04_USART)

This project works as a simple serial test platform using USART1 to transmit and receive data by implementing a *chargen* and *echo* service.


[blue pill]: http://wiki.stm32duino.com/index.php?title=Blue_Pill
