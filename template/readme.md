---
---

## STM32 Template Project Files

This directory contains a project "template" for creating new projects.

### Template Description

The project files contain code the performs the following tasks:
* *Initalizes SYSCLOCK to run at 72MHz*
* *Configures PC13 for LED output*
* *Configures Cortex SysTick for 1ms interrupts*
* *Provides a heartbeat LED toggle every 150ms*

### Template Usage

Copy all files in this directory into a new project directory, for example:

```
% mkdir led_flash
% cp -R template led_flash
% cd led_flash
```

Edit the Makefile to change the project name, which becomes the name for the build target.

### Make Targets

The included Makefile along with the master.ml file provide the following build targets:
* `all` -- the default target that builds the project binaries.
* `swdflash` -- Flashes the code using [ST Utils](https://github.com/texane/stlink) using an ST-Link V2 programmer.
* `serflash` -- Flashes the code using [stm32flash](https://sourceforge.net/p/stm32flash/code/ci/master/tree/) using the STM32 serial bootloader and a USB - TTY board.
* `debug` -- Starts simple debug session to an existing GDB server.
* `macros` -- Display all CPP macros.
* `clean` -- Removes all compiled object and binary files.


