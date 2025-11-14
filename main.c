/*
 * main.c
 *
 *  Created on: 14 Nov 2025
 *      Author: Sayed
 */


#include <msp430.h>
#include <stdint.h>
#include <string.h>

void clockConfigure(void);

void debug_print(const char *s);
void debug_uart_init(void);
void pico_uart_init(void);
void pico_send(const char *s);
void pico_receive_line(char *buffer, uint16_t max_len);
void Serial_Enable(void);

void clockConfigure(void)
{
    CSCTL0_H = CSKEY_H;               // Unlock CS registers
    CSCTL1 = DCOFSEL_0;               // DCO at 1 MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;  // No dividers
    CSCTL0_H = 0;                      // Lock CS registers
}

void debug_print(const char *s)
{
    while (*s)
    {
        while (!(UCA3IFG & UCTXIFG));
        UCA3TXBUF = *s++;
    }
}

void Serial_Enable(void)
{
    P4DIR |= BIT0;      // Set P4.0 as output
    P4OUT |= BIT0;
    P4DIR |= BIT1;
    P4DIR |= BIT0;
}

void debug_uart_init(void)
{
    UCA3CTLW0 |= UCSWRST;       // put USCI in reset
    UCA3CTLW0 |= UCSSEL__SMCLK; // SMCLK clock source

    // Baud 9600 @ 1MHz
    UCA3BRW = 6;
    UCA3MCTLW = (8 << 4) | UCOS16;

    // Select UART function for P2.0 (TX) and P2.1 (RX)
    P2SEL0 &= ~(BIT0 | BIT1);
    P2SEL1 |=  (BIT0 | BIT1);

    UCA3CTLW0 &= ~UCSWRST;      // release reset
}

void pico_uart_init(void)
{

    UCA0CTLW0 |= UCSWRST;
    UCA0CTLW0 |= UCSSEL__SMCLK;

    // Baud 9600 @ 1MHz
    UCA0BRW = 6;
    UCA0MCTLW = (8 << 4) | UCOS16;

    // Select UART pins: P4.3 TX, P4.4 RX
    P4SEL0 |=  BIT3 | BIT4;
    P4SEL1 &= ~(BIT3 | BIT4);

    UCA0CTLW0 &= ~UCSWRST;

}

void pico_send(const char *s)
{
    while (*s)
    {
        while (!(UCA0IFG & UCTXIFG));
        UCA0TXBUF = *s++;
    }
}

void pico_receive_line(char *buffer, uint16_t max_len)
{
    uint16_t idx = 0;

    while (idx < max_len - 1)
    {
        while (!(UCA0IFG & UCRXIFG));
        char c = UCA0RXBUF;

        if (c == '\n')
            break;

        buffer[idx++] = c;
    }
    buffer[idx] = '\0';
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;  // Unlock GPIOs
    Serial_Enable();
    clockConfigure();
    debug_uart_init();   // UCA3 @ P2.0/P2.1
    pico_uart_init();    // UCA0 @ P4.3/P4.4

    debug_print("MSP430FR5043 Host Started\r\n");

    char reply[64];

    while (1)
    {
        Serial_Enable();
        pico_send("GET\n");
        debug_print("Sent: GET\n");

        pico_receive_line(reply, sizeof(reply));

        debug_print("Received: ");
        debug_print(reply);
        debug_print("\r\n");

        __delay_cycles(1000000);
    }
}
