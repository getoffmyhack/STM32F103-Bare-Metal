---
---
## Project 1 "Hello World"

The ubiquitous "Hello World" program in the embedded world is the "Blinken Light."  This project does simply that, blinks an LED and does so with the bare minimum of code.

### Overview

The program logic should be self-explanatory, with the main STM32 guts being the GPIO setup and control in order to blink the LED. *(No delays were (mis-)used in the making of this project!)*

### Hardware Setup

To properly configure the GPIO the following steps are performed:<br>
*(the on-board LED found on the [Blue Pill][blue pill] is connected to PC13, active low)*

1. Enable the clock gate feeding GPIO Port C *(Ref. Manual Sec. 7)*
2. Reset PC13 **MODE** and **CNF** bits *(Ref. Manual Sec. 9)*
3. Set PC13 **MODE** bits to output at 50Mhz (0b11) and **CNF** bits to Push-Pull (0b00) *(Ref. Manual Sec. 9)*

```
void init_led(void)
{
    RCC->APB2ENR    |= RCC_APB2ENR_IOPCEN;                      // enable clock
    GPIOC->CRH      &=  ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);    // reset PC13
    GPIOC->CRH      |= (GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0); // config PC13
}
```

### Blinken Light

In order to blink the LED, the PC13 Bit needs to be toggled using the Bit Set Reset Register - **BSRR** *(Ref. Manual Sec. 9)*.

```
led_state_t led_heartbeat(led_state_t state)
{
    led_state_t next_state;
    
    switch(state)
    {
        case led_on:
            next_state  = led_off;
            GPIOC->BSRR = GPIO_BSRR_BR13;   // led on - active low
            break;
            
        case led_off:
            next_state  = led_on;
            GPIOC->BSRR = GPIO_BSRR_BS13;   // led off
            break;
    }
    
    return next_state;
}
```

[blue pill]: http://wiki.stm32duino.com/index.php?title=Blue_Pill