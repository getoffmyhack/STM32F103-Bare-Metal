/****************************************************************************\
 File:          main.c
 Date:          1 Feb 2018
 
 Description:   Simple Hello World LED Blinken Light
 
 Known bugs/missing features:
 
\****************************************************************************/

#include <stdint.h>
#include "stm32f1xx.h"

/*********************** defines                    *************************/

#define LED_DELAY   150000UL

/*********************** global variables           *************************/

typedef enum
{
    led_on, led_off
} led_state_t;

/*********************** function definitions       *************************/

/*--------------------------------------------------------------------------*\
 
 Function:      init_led()
 
 Description:   initalize LED PORT / PIN for heartbeat
 
 Parameters:    void
 Returns:       void
 
\*--------------------------------------------------------------------------*/

void init_led(void)
{
    RCC->APB2ENR    |= RCC_APB2ENR_IOPCEN;                      // enable clock
    GPIOC->CRH      &=  ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);    // reset PC13
    GPIOC->CRH      |= (GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0); // config PC13
}

/*--------------------------------------------------------------------------*\
 
 Function:      led_heartbeat()
 
 Description:   toggle heartbeat led as needed
 
 Parameters:    led_state_t state   - current led state
 Returns:       led_state_t         - next led state
 
\*--------------------------------------------------------------------------*/

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

/*--------------------------------------------------------------------------*\
 
 Function:      main()
 
 Description:   program entry point
 
 Parameters:    void
 Returns:       void
 
\*--------------------------------------------------------------------------*/

int main(void) 
{
    led_state_t    led_state       = led_on;
    uint32_t       led_delay_count = 0;

    init_led();

    while (1)
    {
        // update heartbeat LED delay counter and toggle state when needed
        led_delay_count = ( (led_delay_count + 1) % LED_DELAY );
        if(led_delay_count == 0)
        {
            led_state = led_heartbeat(led_state);
        }
    }
    
    return 0;
}

