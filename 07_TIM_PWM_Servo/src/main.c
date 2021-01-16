 /****************************************************************************\
 File:          main.c
 Date:
 
 Description:
 
 Known bugs/missing features:
 
\****************************************************************************/
#include <stdio.h>
#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO

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

// PWM defines
#define PULSE_MIN           480            // 1000  µs = 1 ms
#define PULSE_CENTER        1340            // 1500  µs = 1.5 ms
#define PULSE_MAX           2225            // 2000  µs = 2 ms
#define PULSE_FREQ          50              // 20000 µs = 20 ms
#define PRESCALE_1MHZ       1000000         // 1 Mhz prescale

/*********************** global variables           *************************/

// heartbeat LED global vars
typedef enum
{
    led_idle = 0, led_on, led_off
} led_state_t;

volatile led_state_t    led_state       = led_idle;
volatile led_state_t    led_state_next  = led_on;
volatile uint16_t       led_delay_count = 0;

// pwm global vars
volatile uint16_t       pulse_width_us  = PULSE_CENTER;
volatile uint16_t       pulse_width_us_last = PULSE_CENTER;

/*********************** function prototypes        *************************/

void init_systick(void);
void init_clock(void);
void init_led(void);
void init_adc(void);
void init_pwm_timer(void);
void init_hardware(void);
void led_heartbeat(void);

uint16_t map_16bit(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

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
    
    // read ADC for scroll delay
    uint16_t adc_value = ADC1->DR;
    
    // map the ADC value to be between min and max servo values
    pulse_width_us = map_16bit(adc_value, 0, 4096, PULSE_MIN, PULSE_MAX);
    if(pulse_width_us != pulse_width_us_last)
    {
        TIM2->CCR2 = pulse_width_us;
        pulse_width_us_last = pulse_width_us;
        printf("%d\n", pulse_width_us);
    }
}

//------------------------_DEBUG----------------------________-------------

void put_char (uint8_t ch)
{
    if(ch == '\n')
    {
        USART1->DR = (int)('\r');
        while (!(USART1->SR & USART_SR_TXE));
    }
    USART1->DR = (int)(ch);
    while (!(USART1->SR & USART_SR_TXE));
    
}

int _write(int file, char *data, int len)
{
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
    {
        errno = EBADF;
        return -1;
    }
    
    for(int i = 0; i < len; i++)
    {
        put_char(data[i]);
    }
    
    // return # of bytes written - as best we can tell
    return (len);
}

void init_usart(uint32_t baudrate)
{
    RCC->APB2ENR    |= RCC_APB2ENR_IOPAEN;      // enable GPIOA clock
    RCC->APB2ENR    |= RCC_APB2ENR_USART1EN;    // enable USART1 clock
    
    GPIOA->CRH &= ~(GPIO_CRH_CNF9  | GPIO_CRH_MODE9);   // reset PA9
    GPIOA->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10);  // reset PA10
    
    GPIOA->CRH |= GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0;  // 0b11 50MHz output
    GPIOA->CRH |= GPIO_CRH_CNF9_1;    // PA9: output @ 50MHz - Alt-function Push-pull
    GPIOA->CRH |= GPIO_CRH_CNF10_0;   // PA10 RX - Mode = 0b00 (input) - CNF = 0b01 (input floating)
    
    // configure USART1 registers
    uint32_t baud   = (uint32_t)(SystemCoreClock/baudrate);
    USART1->BRR     = baud;
    USART1->CR1     = USART_CR1_TE | USART_CR1_UE;
    
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
 
 Function:      init_adc()
 
 Description:   initalize ADC hardware
 
 Parameters:    void
 Returns:       void
 
\*--------------------------------------------------------------------------*/

void init_adc(void)
{
    // ADC clock must be < 14MHz; Config prescale 72MHz / 6 = 12Mhz
    RCC->CFGR       |= RCC_CFGR_ADCPRE_DIV6;
    
    RCC->APB2ENR    |= RCC_APB2ENR_ADC1EN;  // enable ADC clock
    RCC->APB2ENR    |= RCC_APB2ENR_IOPAEN;  // enable GPIOA clock
    
    // reset MODE and CNF bits to 0000; MODE = input : CNF = analog mode
    GPIOA->CRL &= ~(GPIO_CRL_CNF0  | GPIO_CRL_MODE0);   // reset PA0
    
    // set sample time for ch 0 to 28.5 cycles (0b011)
    ADC1->SMPR2 |= ( ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_0);
    
    // ADC1->SQR1 L[3:0] = 0b0000 at reset; set for 1 conversion
    // ADC1->SQR3 SQ1[4:0] = 0b00000 at reset; 1st conversion = chan 0
    
    // put ADC1 into continuous mode and turn on ADC
    ADC1->CR2 |= (ADC_CR2_CONT | ADC_CR2_ADON);
    
    // reset calibration registers
    ADC1->CR2 |= (ADC_CR2_RSTCAL);
    
    // wait for calibration register initalized
    while(ADC1->CR2 & ADC_CR2_RSTCAL);
    
    // enable calibration
    ADC1->CR2 |= (ADC_CR2_CAL);
    
    // wait for calibration completed
    while(ADC1->CR2 & ADC_CR2_CAL);
    
    // not concerned about power consumption, just start the continuous
    // conversions and will read the DR periodically
    
    // start conversions
    ADC1->CR2 |= ADC_CR2_ADON;
}

/*--------------------------------------------------------------------------*\
 
 Function:      init_pwm_timer()
 
 Description:   init TIM2_CH2 for servo control PWM
 
 Parameters:    void
 Returns:       void
 
\*--------------------------------------------------------------------------*/

void init_pwm_timer(void)
{
    RCC->APB1ENR    |= RCC_APB1ENR_TIM2EN;  // enable TIMer clock
    RCC->APB2ENR    |= RCC_APB2ENR_IOPAEN;  // enable GPIOA clock
    
    // reset MODE and CNF bits to 0000; MODE = input : CNF = analog mode
    GPIOA->CRL &= ~(GPIO_CRL_CNF1  | GPIO_CRL_MODE1);   // reset PA1
    
    // set MODE: 0b11 out @ 50 MHz; CNF: 0b10 alternate out push-pull
    GPIOA->CRL |=  ( (GPIO_CRL_CNF1_1) | (GPIO_CRL_MODE1_1 | GPIO_CRL_MODE1_0) );
    
    // set time base
//#define PULSE_FREQ          50              // 20000 µs = 20 ms
//#define PRESCALE_1MHZ       1000000         // 1 Mhz prescale
    
    // set prescale to be F = 1 MHz ; T = 0.000001 = 1µs
    // Fcounter = Fprescale/(PSC + 1) -> PSC = (Fprescale / Fcounter) - 1
    uint16_t prescale   = (SystemCoreClock / PRESCALE_1MHZ);
    TIM2->PSC           = prescale - 1;
    
    // set period to be 50Hz / 20ms
    uint16_t period = (SystemCoreClock / prescale / PULSE_FREQ);
    TIM2->ARR       = period;
    
    // set duty cycle
    TIM2->CCR2 = PULSE_CENTER;
    
    // configure output compare
    
    // PWM mode 1
    TIM2->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);
    
    // OCR preload enable
//    TIM2->CCMR1 |=  TIM_CCMR1_OC2PE;
    
    // enable output compare on OC2 pin
    TIM2->CCER |= TIM_CCER_CC2E;
    
    // enable counter
    TIM2->CR1 |= TIM_CR1_CEN;
    
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
    init_usart(230400);
    init_led();
    init_adc();
    init_pwm_timer();
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
    
    // DEBUG
//    printf("%ld\n", TIM2->CCR2);
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
 
 Function:      map_16bit
 
 Description:   16 bit linear maping function
 
 Parameters:    uint16_t    x       - input value
                uint16_t    in_min  - input min value
                uint16_t    in_max  - input max value
                uint16_t    out_min - output min value
                uint16_t    out_max - output max value
 
 Returns:       uint16_t    - the mapped value
 
 \*--------------------------------------------------------------------------*/

uint16_t map_16bit(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
    
    printf("prescale: %ld\n", TIM2->PSC);
    printf("reload  : %ld\n", TIM2->ARR);
    printf("duty    : %ld\n", TIM2->CCR2);

    while (1)
    {
        // main loop
        led_heartbeat();
    }
    
    return 0;
}

