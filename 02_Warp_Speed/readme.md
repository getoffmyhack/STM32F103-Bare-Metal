---
---
## Project 2 "Warp Speed"

Warp Speed is a simple extension of the "Hello World" project to enable the STM32F103 controller run at full speed.  The [Blue Pill] comes with an on-board 8MHz crystal which acts as the High Speed External clock.  This 8MHz clock is then multiplied by an internal PLL to achieve the max clock speed.

### Overview

The only difference from "Hello World" is the addition of the clock configuration.

### Hardware Setup

In order to run the STM32F103 µC at full speed, several RCC registers need to be properly configured.  The [SystemCoreClockUpdate()](http://arm-software.github.io/CMSIS_5/Core/html/group__system__init__gr.html) is a function provided in the CMSIS core system_stm32f1xx.c file and updates the SystemCoreClock variable.

The init_clock() function performs the following steps:

1. Set the Flash waitstate to 2 *(Ref. Manual Sec. 3)*
2. Set the APB1 PCKL1 to not exceed 36MHz; set prescaleer to HCLK/2 (0b100) *(Ref. Manual Sec. 7)*
3. Enable High Speed External clock. *(Ref. Manual Sec. 7)*
4. Wait until the HSE clock is ready.
5. Set PLL source to be HSE. *(Ref. Manual Sec. 7)*
6. Set PLL Multiplier to 9 (8 MHz Crystal x 9 = 72MHz) *(Ref. Manual Sec. 7)*
7. Enable the PLL. *(Ref. Manual Sec. 7)*
8. Wait until the PLL is ready.
9. Set clock source to use PLL. *(Ref. Manual Sec. 7)*
10. Update the SystemCoreClock variable.

```
void init_clock(void)
{
    // Conf clock : 72MHz using HSE 8MHz crystal w/ PLL X 9 (8MHz x 9 = 72MHz)
    FLASH->ACR      |= FLASH_ACR_LATENCY_2; // 1. Two wait states, per datasheet
    RCC->CFGR       |= RCC_CFGR_PPRE1_2;    // 2. prescale AHB1 = HCLK/2
    RCC->CR         |= RCC_CR_HSEON;        // 3. enable HSE clock
    while( !(RCC->CR & RCC_CR_HSERDY) );    // 4. wait for the HSEREADY flag
    
    RCC->CFGR       |= RCC_CFGR_PLLSRC;     // 5. set PLL source to HSE
    RCC->CFGR       |= RCC_CFGR_PLLMULL9;   // 6. multiply by 9
    RCC->CR         |= RCC_CR_PLLON;        // 7. enable the PLL
    while( !(RCC->CR & RCC_CR_PLLRDY) );    // 8. wait for the PLLRDY flag
    
    RCC->CFGR       |= RCC_CFGR_SW_PLL;     // 9. set clock source to pll
    
    SystemCoreClockUpdate();                // 10. calculate the SYSCLOCK value
}

```

[blue pill]: http://wiki.stm32duino.com/index.php?title=Blue_Pill