---
---
# STM32 CMSIS Build Environment

## Introduction

This is a repackaging of the STM32 CMSIS build files included with [STMCube](http://www.st.com/en/embedded-software/stm32cube-mcu-packages.html) packages.  I am creating this repository as a bare minimum build environment for the STM32 line of ARM Cortex-M micro controllers.

The source packages used for this repository:

* [STM32CubeF1](http://www.st.com/en/embedded-software/stm32cubef1.html) Version 1.6.0

## Supported MCUs

Currently, this repository only includes the STM32F1xx Cube package, with specific modifications needed for STM32F103X8 builds.

## Modifications

The following list of modifications have been made in order to properly build STM32 projects:

* Device/STM32F1xx/linker/STM32F103XB_FLASH.ld
    * _Each blank line contained the `0` character.  Removed `0` from blank lines._
    * _Added_ `PROVIDE ( __end__ = . );` _to_ `._user_heap_stack` _needed for librdimon._
        * [reference](https://mcuoneclipse.com/2015/05/30/problem-undefined-reference-to-__end__-if-using-semihosting/)
        
##  Directory Structure

| Directory                |   Description                                                |
| -------------------------|--------------------------------------------------------------|
| `CMSIS/include`| - ARM Cortex Core M CMSIS include files                      |
| `Device/STM32F1xx/include` | - STM32F1xx include files                                    |
| `Device/STM32F1xx/linker`  | - STM32F1xx gcc linker scripts                               |
| `Device/STM32F1xx/src`	   | - STM32F1xx system C source; MCU specific startup ASM sources|

## Reference Links




