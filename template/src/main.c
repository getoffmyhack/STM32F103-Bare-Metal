/****************************************************************************\
 File:          main.c
 Date:
 
 Description:
 
 Known bugs/missing features:
 
\****************************************************************************/

#include <stdint.h>
#include "stm32f1xx.h"
#include "globals.h"
#include "main.h"

/*********************** defines                    *************************/

// create simple define abstractions for heartbeat LED
#define LED_DELAY_MS        150
#define LED_PORT            GPIOC
#define LED_CR              CRH
#define LED_SET             GPIO_BSRR_BS13
#define LED_RESET           GPIO_BSRR_BR13
#define LED_PORT_RESET_BITS GPIO_CRH_MODE13 | GPIO_CRH_CNF13
#define LED_PORT_SET_BITS   GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0
#define LED_CLOCK           RCC_APB2ENR_IOPCEN

/*********************** global variables           *************************/

// heartbeat LED global vars
typedef enum
{
    led_idle = 0, led_on, led_off
} led_state_t;

volatile led_state_t    led_state       = led_idle;
volatile led_state_t    led_state_next  = led_on;
volatile uint16_t       led_delay_count = 0;

/*********************** function prototypes        *************************/

void init_systick(void);
void init_clock(void);
void init_led(void);
void init_hardware(void);
void led_heartbeat(void);

/*********************** ISR definitions            *************************/

/*--------------------------------------------------------------------------*\
 
 Function:      SysTick_Handler
 
 Description:   Cortex M3 SysTick ISR w/ heartbeat LED counter
 
 Parameters:    void
 Returns:       void
 
\*--------------------------------------------------------------------------*/

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

/*********************** function definitions       *************************/

/*--------------------------------------------------------------------------*\
 
 Function:      init_systick()
 
 Description:   initalizes SysTick timer to 1ms / tick, assumes
                SystemCoreClockUpdate() has been called post RCC config.
 
 Parameters:    void
 Returns:       void
 
\*--------------------------------------------------------------------------*/

void init_systick(void)
{
    int tick_time = SystemCoreClock/1000;       // Generate interrupt each 1 ms
    SysTick_Config(tick_time);                  // Configure systick timer
}


/*--------------------------------------------------------------------------*\
 
 Function:      init_clock()
 
 Description:   configure SysClock to run at 72MHz
 
 Parameters:    void
 Returns:       void
 
\*--------------------------------------------------------------------------*/

void init_clock(void)
{
    // Conf clock : 72MHz using HSE 8MHz crystal w/ PLL X 9 (8MHz x 9 = 72MHz)
    FLASH->ACR      |= FLASH_ACR_LATENCY_2; // Two wait states, per datasheet
    RCC->CFGR       |= RCC_CFGR_PPRE1_2;    // prescale AHB1 = HCLK/2
    RCC->CR         |= RCC_CR_HSEON;        // enable HSE clock
    while( !(RCC->CR & RCC_CR_HSERDY) );    // wait for the HSEREADY flag
    
    RCC->CFGR       |= RCC_CFGR_PLLSRC;     // set PLL source to HSE
    RCC->CFGR       |= RCC_CFGR_PLLMULL9;   // multiply by 9
    RCC->CR         |= RCC_CR_PLLON;        // enable the PLL
    while( !(RCC->CR & RCC_CR_PLLRDY) );    // wait for the PLLRDY flag
    
    RCC->CFGR       |= RCC_CFGR_SW_PLL;     // set clock source to pll

    while( !(RCC->CFGR & RCC_CFGR_SWS_PLL) );    // wait for PLL to be CLK
    
    SystemCoreClockUpdate();                // calculate the SYSCLOCK value
}

/*--------------------------------------------------------------------------*\
 
 Function:      init_led()
 
 Description:   initalize LED PORT / PIN for heartbeat
 
 Parameters:    void
 Returns:       void
 
\*--------------------------------------------------------------------------*/

void init_led(void)
{
    RCC->APB2ENR        |= LED_CLOCK;               // enable GPIO clock for LED
    LED_PORT->LED_CR    &= ~(LED_PORT_RESET_BITS);  // reset pin MODE / CNF
    LED_PORT->LED_CR    |=  (LED_PORT_SET_BITS);    // MODE: 50Mhz ouput CNF: PP
}

/*--------------------------------------------------------------------------*\
 
 Function:      init_hardware()
 
 Description:   call hardware initalization functions
 
 Parameters:    void
 Returns:       void
 
\*--------------------------------------------------------------------------*/

void init_hardware(void)
{
    init_clock();
    init_led();
    init_systick();
}

/*--------------------------------------------------------------------------*\
 
 Function:      led_heartbeat()
 
 Description:   toggle heartbeat led as needed
 
 Parameters:    void
 Returns:       void
 
\*--------------------------------------------------------------------------*/

void led_heartbeat(void)
{
    switch(led_state)
    {
        case led_on:
            led_state       = led_idle;
            led_state_next  = led_off;
            LED_PORT->BSRR  = LED_SET;
            break;
            
        case led_off:
            led_state       = led_idle;
            led_state_next  = led_on;
            LED_PORT->BSRR  = LED_RESET;
            break;
            
        default:
            break;
    }
}

/*--------------------------------------------------------------------------*\
 
 Function:      main()
 
 Description:   program entry point
 
 Parameters:    void
 Returns:       void
 
\*--------------------------------------------------------------------------*/

int main(void) 
{
    init_hardware();

    while (1)
    {
        // main loop
        led_heartbeat();
    }
    
    return 0;
}

