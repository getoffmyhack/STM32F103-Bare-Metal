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
#define LED_PORT            GPIOC
#define LED_CR              CRH
#define LED_SET             GPIO_BSRR_BS13
#define LED_RESET           GPIO_BSRR_BR13
#define LED_PORT_RESET_BITS GPIO_CRH_MODE13 | GPIO_CRH_CNF13
#define LED_PORT_SET_BITS   GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0
#define LED_CLOCK           RCC_APB2ENR_IOPCEN

// usart defines
#define USART_BAUD          230400
#define USART_BUFFER_SIZE   16

// output defines
#define OUTPUT_MODE_KEY         0x03            // control-c
#define OUTPUT_CHARGEN_LENGTH   72
#define CHARGEN_STRING_LENGTH   95


// delay defines
#define LED_DELAY_MS        150
#define USART_DELAY_MS      5

/*********************** global variables           *************************/

// output mode global vars
typedef enum
{
    output_chargen = 0, output_echo
} output_mode_t;

output_mode_t       output_mode         = output_chargen;
volatile uint8_t    output_mode_changed = 1;

// heartbeat LED global vars
typedef enum
{
    led_idle = 0, led_on, led_off
} led_state_t;

volatile led_state_t    led_state       = led_idle;
volatile led_state_t    led_state_next  = led_on;
volatile uint16_t       led_delay_count = 0;

// usart global vars
typedef enum
{
    usart_idle = 0, usart_send
} usart_state_t;

typedef struct {
    uint8_t buffer[USART_BUFFER_SIZE];
    uint8_t head_pos;
    uint8_t tail_pos;
} rx_buffer_t;

volatile usart_state_t  usart_state         = usart_idle;
volatile uint16_t       usart_delay_count   = 0;
volatile rx_buffer_t    uart_buffer         = { {0}, 0, 0 };

/*********************** function prototypes        *************************/

void init_hardware(void);
void init_systick(void);
void init_clock(void);
void init_led(void);
void init_usart(uint32_t baudrate);

void    buffer_char(uint8_t c);
void    put_char(uint8_t byte);
int     put_line(char * string);
uint8_t get_char(uint8_t *c);

void led_heartbeat(void);
void chargen(void);


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
    
    // update the usart counter and change state if needed
    usart_delay_count = ( (usart_delay_count + 1) % USART_DELAY_MS );
    if(usart_delay_count == 0)
    {
        usart_state       = usart_send;
    }
}

/*--------------------------------------------------------------------------*\
 
 Function:      USART1_IRQHandler
 
 Description:   USART1 Interrupt service routine
 
 Parameters:    void
 Returns:       void
 
\*--------------------------------------------------------------------------*/

void USART1_IRQHandler(void)
{
    if(USART1->SR & USART_SR_ORE)
    {
        // process overrun error if needed
    }
    
    // get character from data reg
    uint8_t in_char = (USART1->DR & 0xFF);
    
    if(in_char == OUTPUT_MODE_KEY)
    {
        if(output_mode == output_chargen)
        {
            output_mode = output_echo;
        }
        else
        {
            output_mode = output_chargen;
        }
        output_mode_changed = 1;
        put_char('\n');
    }
    else
    {
        if(output_mode == output_echo)
        {
            buffer_char(in_char);
        }
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
 
 Function:      init_usart()
 
 Description:   initialize USART1 
 
 Parameters:    uint32_t baudrate - the baudrate to configure
 Returns:       void
 
\*--------------------------------------------------------------------------*/

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
    USART1->CR1     = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    
    // configure NVIC
    NVIC_EnableIRQ(USART1_IRQn);
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
    init_usart(USART_BAUD);
    init_systick();
}

/*--------------------------------------------------------------------------*\
 
 Function:      put_char()
 
 Description:   put char in USART data register
 
 Parameters:    uint8_t byte  -   the byte to send out usart
 Returns:       void
 
\*--------------------------------------------------------------------------*/

void put_char(uint8_t byte)
{
    if(byte == '\n')
    {
        put_char('\r');
    }
    USART1->DR = (int)(byte);
    while (!(USART1->SR & USART_SR_TXE));
}

/*--------------------------------------------------------------------------*\
 
 Function:      put_line()
 
 Description:   outputs passed in string and ends with newline
 
 Parameters:    char * string   - string to output
 Returns:       int             - number of chars output
 
\*--------------------------------------------------------------------------*/

int put_line(char * string)
{
    int count = 0;
    
    while(*string)
    {
        put_char(*string);
        string++;
        count++;
    }
    put_char('\n');
    
    return(count);
}

/*--------------------------------------------------------------------------*\
 
 Function:      buffer_char()
 
 Description:   places new character in ring buffer.  If buffer is full,
                quietly discard character.
 
 Parameters:    uint8_t c   - character to buffer
 Returns:       void
 
\*--------------------------------------------------------------------------*/

void buffer_char(uint8_t c)
{
    int i = (uart_buffer.head_pos + 1) % USART_BUFFER_SIZE;
    
    if (i != uart_buffer.tail_pos)
    {
        uart_buffer.buffer[uart_buffer.head_pos] = c;
        uart_buffer.head_pos = i;
    }
}

/*--------------------------------------------------------------------------*\
 
 Function:      get_char()
 
 Description:   gets next character from ring buffer
 
 Parameters:    uint8_t *c  - pointer to memory to put character
 Returns:       uint8_t     - 0 if buffer empty, 1 if char available
 
\*--------------------------------------------------------------------------*/

uint8_t get_char(uint8_t *c)
{
    // if head_pos = tail_pos, there are no characters in buffer
    if(uart_buffer.head_pos == uart_buffer.tail_pos)
        return 0;
    
    // put char from buffer into var pointed to by c
    *c = uart_buffer.buffer[uart_buffer.tail_pos];
    uart_buffer.tail_pos = (uart_buffer.tail_pos + 1) % USART_BUFFER_SIZE;
    
    return 1;
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
 
 Function:      chargen()
 
 Description:   implements chargen service
                https://en.wikipedia.org/wiki/Character_Generator_Protocol
 
 Parameters:    void
 Returns:       void
 
\*--------------------------------------------------------------------------*/

void chargen(void)
{
    static uint8_t  chargen_count           = 0;
    static uint8_t  chargen_start_index     = 0;
    static uint8_t  chargen_string_index    = 0;
    
    // this is an 'escaped' string: the " \ chars are escaped
    const char      chargen_string[]        =
    " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~";
    
    switch (usart_state)
    {
        case usart_send:
            usart_state = usart_idle;

            put_char(chargen_string[chargen_string_index]);
            chargen_string_index = (chargen_string_index + 1) % CHARGEN_STRING_LENGTH;
            
            chargen_count = (chargen_count + 1) % OUTPUT_CHARGEN_LENGTH;
            if(chargen_count == 0)
            {
                put_char('\n');
                chargen_start_index = (chargen_start_index + 1) % CHARGEN_STRING_LENGTH;
                chargen_string_index = chargen_start_index;
            }
            break;
            
        default:
            break;
            
    }
}

/*--------------------------------------------------------------------------*\
 
 Function:      echo()
 
 Description:   implements simlpe echo service
                https://en.wikipedia.org/wiki/Echo_Protocol
 
 Parameters:    void
 Returns:       void
 
\*--------------------------------------------------------------------------*/

void echo(void)
{
    uint8_t in_char;

    if(get_char(&in_char))
    {
        switch(in_char)
        {
            case 0x0D:
                put_char('\n');
                break;
            default:
                put_char(in_char);
        }
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
    put_char('\n');

    while (1)
    {
        led_heartbeat();
        
        switch (output_mode)
        {
        case output_chargen:
            if(output_mode_changed)
            {
                put_line("mode: chargen - cntl-c to change");
                output_mode_changed = 0;
            }
            chargen();
            break;
            
        case output_echo:
            if(output_mode_changed)
            {
                put_line("mode: echo    - cntl-c to change");
                output_mode_changed = 0;
            }
            echo();
            break;
        }
    }
    
    return 0;
}

