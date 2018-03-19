---
---
## Project 4 "USART"

Project 4 configures the USART peripheral for transmit, receive and enables the receive interrupt for accepting incoming data.

### Overview

This project is used as a general USART example, but also contains code logic to be used as a simple serial test platform which operates in two different modes. Switching modes is done by sending a Control-C character.

The first, and power-on mode, is the *chargen* mode.  This mode acts like an [RFC 864][rfc864] standard [character generator service][chargen] and continuously outputs the character data on the USART port.

```
!"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefgh
"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghi
#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghij
$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijk
%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijkl
&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklm
'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmn
()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmno
)*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmnop
*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmnopq
+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmnopqr
,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmnopqrs
-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmnopqrst
```

The second mode implements a simple [RFC 862][rfc862] standard [echo service][echo].  Each character received on the USART is retransmitted back out the USART port.

### Hardware Setup

Most of the code in this project does not strictly relate to the USART and all USART code is contained in two functions and an interrupt service routine.

The initialization function accomplishes several tasks.  
* The first task is configuring the clock gating to enable the GPIOA and USART1 clocks.  
* The second task configures the GPIO pins; the pin configuration bits are first reset to 0s for both the TX, PA9, pin and the RX, PA10, pin.  The TX pin is configured as 50MHz output, push-pull and alternate function.  The RX pin is configured input mode and floating.
* The next task configures the USART1 registers.  The baud rate is passed as an argument and the value for the baud rate register is dependent upon the system clock speed.  The USART CR1 register enables transmit, receive, receive not empty interrupt and the USART peripheral.
* The last task needed is to enable the USART1 interrupt within the Cortex NVIC.

```
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
```

### Transmit Data

Transmitting data is accomplished through the put_char() function.  This function simply puts a character passed in as a function argument into the USART_DR register.  Once the data has been written to the USART_DR, it waits until the TXE - Transmit data register empty flag to be set.

```
void put_char(uint8_t byte)
{
    if(byte == '\n')
    {
        put_char('\r');
    }
    USART1->DR = (int)(byte);
    while (!(USART1->SR & USART_SR_TXE));
}
```

### Receive Data via IRQ

The USART1 IRQ Handler is fairly straight forward in terms of how it handles the interrupt.  One major point worth noting is that the RX Not Empty Interrupt Enable control bit will trigger the USART1 IRQ on two different events, an OverRun Error (ORE) event or an Received Data Ready to be Read event (RXNE - RX Not Empty).  Section 27.4 in the technical reference manual describes the USART interrupts.

If the ORE event triggered the interrupt, it must be cleared using the sequence of a read to the USART_SR register followed by a read to the USART_DR register.  The RXNE event is cleared by reading the USART_DR register.

The rest of the code in the ISR simply processes the incoming character.

```
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
            bufferchar(in_char);
        }
    }
}
```
[rfc864]: https://tools.ietf.org/html/rfc864
[chargen]: https://en.wikipedia.org/wiki/Character_Generator_Protocol
[rfc862]: https://tools.ietf.org/html/rfc862
[echo]: https://en.wikipedia.org/wiki/Echo_Protocol