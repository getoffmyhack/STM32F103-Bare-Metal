---
---
## Project 2 "Warp Speed"

Warp Speed is a simple extension of the "Hello World" project to enable the STM32F103 controller run at full speed.  The [Blue Pill] comes with an on-board 8MHz crystal which acts as the High Speed External clock.  This 8MHz clock is then multiplied by an internal PLL to achieve the max clock speed.

### Overview

The only difference from "Hello World" is the addition of the clock configuration.

### Hardware Setup

In order to run the STM32F103 µC at full speed, several registers need to be properly configured.  The init_clock() function performs the following steps:

1. Set the Flash waitstate to 2 *(Ref. Manual Sec. 3)*
2. Set the APB1 PCKL1 to not exceed 36MHz; set prescaleer to HCLK/2 (0b100) *(Ref. Manual Sec. 7)*
3. Enable High Speed External clock. *(Ref. Manual Sec. 7)*
4. Set PLL source to be HSE. *(Ref. Manual Sec. 7)*
5. Set PLL Multiplier to 9 (8 MHz Crystal x 9 = 72MHz) *(Ref. Manual Sec. 7)*
6. Enable the PLL. *(Ref. Manual Sec. 7)*
7. Set clock source to use PLL. *(Ref. Manual Sec. 7)*

```
void init_clock(void)
{
    // Conf clock : 72MHz using HSE 8MHz crystal w/ PLL X 9 (8MHz x 9 = 72MHz)
    FLASH->ACR      |= FLASH_ACR_LATENCY_2; // Two wait states, per datasheet
    RCC->CFGR       |= RCC_CFGR_PPRE1_2;    // prescale AHB1 = HCLK/2
    RCC->CR         |= RCC_CR_HSEON;        // enable HSE clock
    RCC->CFGR       |= RCC_CFGR_PLLSRC;     // set PLL source to HSE
    RCC->CFGR       |= RCC_CFGR_PLLMULL9;   // multiply by 9
    RCC->CR         |= RCC_CR_PLLON;        // enable the PLL
    RCC->CFGR       |= RCC_CFGR_SW_PLL;     // set clock source to pll
}
```

[blue pill]: http://wiki.stm32duino.com/index.php?title=Blue_Pill