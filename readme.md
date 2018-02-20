---
---
# Down the STM32 Bare Metal Rabbit Hole

## Introduction

This repository is a collection of my code projects as I learn the aspects of programming an STM32F103C8 Cortex M3 micro controller at the "bare metal" register level.  I will be creating example code for most all aspects of the µC including SPI, USART, I2C, DMA, etc.

### Development Hardware

All example projects are using a ["Blue Pill"][blue pill] board as the target hardware platform.

### [template/](template)

The template directory contains the files necessary to start a new STM32F103 project.

### [gdb/](gdb)

The gdb directory contains gbd command files used for debugging and flashing.

## Example Projects

### [01 Hello World/](01_Hello_World)

The first project being the ubiquitous "Hello World" as a "Blinken Light".

[blue pill]: http://wiki.stm32duino.com/index.php?title=Blue_Pill