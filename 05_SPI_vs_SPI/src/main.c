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

#include "font_swiss_8x8.h"
#include "max7219.h"

/*********************** defines                    *************************/

// create simple define abstractions for heartbeat LED
#define LED_PORT            GPIOC
#define LED_CR              CRH
#define LED_SET             GPIO_BSRR_BS13
#define LED_RESET           GPIO_BSRR_BR13
#define LED_PORT_RESET_BITS GPIO_CRH_MODE13 | GPIO_CRH_CNF13
#define LED_PORT_SET_BITS   GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0
#define LED_CLOCK           RCC_APB2ENR_IOPCEN

// delay defines
#define SCROLL_DELAY_MS     50
#define LED_DELAY_MS        150

/*********************** global variables           *************************/

// heartbeat LED global vars
typedef enum
{
    led_idle = 0, led_on, led_off
} led_state_t;

volatile led_state_t    led_state       = led_idle;
volatile led_state_t    led_state_next  = led_on;
volatile uint16_t       led_delay_count = 0;

// scrolling display global vars
typedef enum
{
    scroll_state_idle = 0, scroll_state_active, scroll_state_blank
} scroll_state_t;

volatile scroll_state_t scroll_state        = scroll_state_idle;
volatile scroll_state_t scroll_state_next   = scroll_state_active;
volatile uint16_t       scroll_delay_count  = 0;


/*********************** function prototypes        *************************/

void init_systick(void);
void init_clock(void);
void init_led(void);
void init_spi(void);
void init_hardware(void);
void init_matrix(void);
void led_heartbeat(void);
void send_to_max(uint8_t reg, uint8_t reg_byte);
void clear_matrix(void);
void scroll_display(void);

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

    // update scroll counter and update state when needed
    scroll_delay_count = (scroll_delay_count + 1) % SCROLL_DELAY_MS;
    if(scroll_delay_count == 0)
    {
        scroll_state = scroll_state_next;
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
 
 Function:      init_spi()
 
 Description:   Initalizes the SPI hardware used for comms to MAX driver
 
 Parameters:    void
 Returns:       void
 
 \*--------------------------------------------------------------------------*/

void init_spi(void)
{
    RCC->APB2ENR    |= RCC_APB2ENR_IOPAEN;  // enable GPIOA clock
    RCC->APB2ENR    |= RCC_APB2ENR_SPI1EN;  // enable SPI1 clock
    
    // reset PA4, PA5, PA7 MODE and CNF to 0b00
    GPIOA->CRL &= ~(
                    (GPIO_CRL_MODE4 | GPIO_CRL_CNF4) |
                    (GPIO_CRL_MODE5 | GPIO_CRL_CNF5) |
                    (GPIO_CRL_MODE7 | GPIO_CRL_CNF7)
                    );
    
    // init PA4 NSS  - Mode = 11 (50Mhz) - CNF = 0b00
    GPIOA->CRL |= GPIO_CRL_MODE4_1 | GPIO_CRL_MODE4_0;
    
    // init PA5 SCK  - Mode = 11 (50Mhz) - CNF = 0b10 (Alt Function PP)
    GPIOA->CRL |= GPIO_CRL_MODE5_1 | GPIO_CRL_MODE5_0 | GPIO_CRL_CNF5_1;
    
    // init PA7 MOSI - Mode = 11 (50Mhz) - CNF = 0b10 (Alt Function PP)
    GPIOA->CRL |= GPIO_CRL_MODE7_1 | GPIO_CRL_MODE7_0 | GPIO_CRL_CNF7_1;
    
    GPIOA->BSRR = GPIO_BSRR_BS4;            // set the SS pin high
    
    // initialize the SPI configuration register
    SPI1->CR1  =  SPI_CR1_DFF   // 16-bit data frame
                | SPI_CR1_SSM   // software slave management enabled
                | SPI_CR1_SSI   // internal slave select
                | SPI_CR1_MSTR  // SPI master mode
                | SPI_CR1_BR_2; // bit rate prescale /32 (72MHz/32 = 2.25MHz)
    
    SPI1->CR1  |= SPI_CR1_SPE;  // enable SPI
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
    init_spi();
    init_matrix();
    init_systick();
}

/*--------------------------------------------------------------------------*\
 
 Function:      void init_matrix(void)
 
 Description:   Initalizes the max7219 driver and clears the display matrix
 
 Parameters:    void
 Returns:       void
 
 \*--------------------------------------------------------------------------*/

void init_matrix(void)
{
    // delay to allow time for the max's clock to stabilize before sending cmds
    for(int i = 0; i< 75000; i++);

    // turn off display-test mode.  According to the max7219 datasheet,
    // the max will default into display-test mode which will light all leds.
    // This overides the shutdown mode so display-test mode should be turned
    // off first.
    send_to_max(MAX7219_REG_DISPLAY_TEST, MAX7219_DISPLAY_TEST_OFF);
    
    // set all max driver to Shutdown Mode = on; this will put the
    // max chips in shutdown mode where all leds are turned off
    send_to_max(MAX7219_REG_SHUTDOWN, MAX7219_SHUTDOWN_MODE_ON);
    
    // set the max drivers to NO DECODE mode (no need to decode BCD chars)
    send_to_max(MAX7219_REG_DECODE_MODE, MAX7219_DECODE_MODE_OFF);
    
    // set segment intensity (drive current / pwm duty cycle)
    send_to_max(MAX7219_REG_INTENSITY, MAX7219_DUTY_CYCLE_31_32);
    
    // set to scan all 8 digits (digits on the max are rows on the 8x8 matrix)
    send_to_max(MAX7219_REG_SCAN_LIMIT, MAX7219_DISPLAY_DIGIT_0_7);
    
    // All init commands have been pushed through, now clear the register
    // contents of the max chips
    clear_matrix();
    
    // turn the max driver on
    send_to_max(MAX7219_REG_SHUTDOWN, MAX7219_SHUTDOWN_MODE_OFF);
}

/*--------------------------------------------------------------------------*\
 
 Function:      void send_to_max(uint8_t reg, uint8_t reg_byte)
 
 Description:   sends data via spi to max7219 chip
 
 Parameters:    uint8_t reg         - the register address in max7219
                uint8_t reg_byte    - the byte to write to the register
 
 Returns:       void
 
\*--------------------------------------------------------------------------*/

void send_to_max(uint8_t reg, uint8_t reg_byte)
{
    uint16_t data = ( (reg << 8) | reg_byte );
    
    GPIOA->BSRR = GPIO_BSRR_BR4;        // reset NSS pin
    
    SPI1->DR = data;                    // send data out SPI
    while( !(SPI1->SR & SPI_SR_TXE) );  // wait until transmit buffer empty
    while( SPI1->SR & SPI_SR_BSY );     // wait until SPI not busy
    
    GPIOA->BSRR = GPIO_BSRR_BS4;        // set NSS pin
}

/*--------------------------------------------------------------------------*\
 
 Function:      void clear_matrix(void)
 
 Description:   this will write all row bytes to 0 in memory of the max chips
 
 Parameters:    void
 Returns:       void
 
 \*--------------------------------------------------------------------------*/

void clear_matrix(void)
{
    uint8_t row;            // row loop counter
    
    // iterate through all row registers
    for(row = 0; row < MAX7219_ROWS; row++)
    {
        send_to_max(MAX7219_REG_ROW0 + row, 0x00);
    }
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
 
 Function:
 
 Description:
 
 Parameters:    void
 Returns:       void
 
\*--------------------------------------------------------------------------*/

void scroll_display(void)
{
    uint8_t i               = 0;
    uint8_t row_byte        = 0;
    
    static uint8_t buffer[8]    = {0,0,0,0,0,0,0,0};
    static uint8_t shift        = 0;
    static uint8_t display_char = 0;
    
    switch(scroll_state)
    {
        case scroll_state_active:
            for(i = 0; i < MAX7219_ROWS; i++)
            {
                row_byte = font[display_char][i];
                buffer[i] = (buffer[i] << 1) | (row_byte >> (7 - shift));
                send_to_max(MAX7219_REG_ROW0 + i, buffer[i]);
            }
            
            scroll_state_next = scroll_state_active;
            shift = (shift + 1) % 8;
            if(shift == 0)
            {
                display_char++;
                scroll_state_next = scroll_state_blank;
            }
            scroll_state = scroll_state_idle;
            break;
            
        case scroll_state_blank:
            for(i = 0; i < MAX7219_ROWS; i++)
            {
                buffer[i] <<= 1;
                send_to_max(MAX7219_REG_ROW0 + i, buffer[i]);
            }
            
            scroll_state_next   = scroll_state_active;
            scroll_state        = scroll_state_idle;
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

    
    // setup STM32 hardware
    init_hardware();
    
    while (1)
    {
        led_heartbeat();
        scroll_display();
    }
    return 0;
}

