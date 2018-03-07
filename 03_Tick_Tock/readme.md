---
---
## Project 3 "Tick Tock"

Tick Tock extends the previous projects enabling the Cortext M3 SysTick timer in order to create a constant clock used to flash the LED heartbeat.

The source in this project becomes the basis of the STM32F103 project template files.

### Overview

The Cortex M3 core contains a 24 bit countdown timer used for creating a constant timer interrupt.  This timer is used in an RTOS as the scheduling timer, but can be used for whatever purpose needed.  In this case, it will be used as a simple scheduling timer.

### Hardware Setup

Although the SysTick timer can be configured via it's registers, the CMSIS core_cm3.h file contains the [SysTick_Config()][SysTick Core] function to set the number of clock ticks per interrupt.  Here the [SystemCoreClock][System Core Clock] variable is used to compute the number of ticks needed for a 1ms interrupt and passed to SysTick_Config() to set the registers.

```
void init_systick(void)
{
    int tick_time = SystemCoreClock/1000;       // Generate interrupt each 1 ms
    SysTick_Config(tick_time);                  // Configure systick timer
}
```

The Interrupt Service Routine (ISR) SysTick_Handler() is called each time the SysTick countdown timer reaches 0.  Here the LED delay counter is updated and the LED state is changed each time the delay counter resets to 0;

```
void SysTick_Handler(void)
{
    // update heartbeat LED delay counter and toggle state when needed
    led_delay_count = ( (led_delay_count + 1) % LED_DELAY_MS );
    if(led_delay_count == 0)
    {
        led_state       = led_state_next;
        led_state_next  = led_idle;
    }
}
```

[SysTick Core]: http://arm-software.github.io/CMSIS_5/Core/html/group__SysTick__gr.html
[System Core Clock]: http://arm-software.github.io/CMSIS_5/Core/html/group__system__init__gr.html