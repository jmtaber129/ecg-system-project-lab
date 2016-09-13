#include "uart-queue.h"

#include <msp430g2553.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>

#define TXLED BIT0
#define RXLED BIT6
#define TXD BIT2
#define RXD BIT1

void ConfigureClocks();
void ConfigurePorts();
void ConfigureUart();
void ConfigureTimer();

char timer_count = 50;

UartQueue uart_queue;

int main(void) {
  ConfigureClocks();
  ConfigurePorts();
  ConfigureUart();
  ConfigureTimer();

  __bis_SR_register(GIE);

  // Serial data-streaming loop.
  while (1) {
    if (!uart_queue.Empty()) {
      // Get the next string to send.
      const char* current_char = uart_queue.Front();

      // Iterate through the string until a NULL char is encountered.
      while(*current_char != '\0') {
        // Wait for the transmit buffer to be ready.
        while (!(IFG2 & UCA0TXIFG));

        // Load the char into the buffer.
        UCA0TXBUF = *current_char;

        // Move to the next char.
        ++current_char;
      }

      // Remove the string we just sent from the queue.
      uart_queue.Pop();
    } else {
      // If there's nothing left to process, enter low-power mode.
      __bis_SR_register(LPM0_bits);
    }

  }
}

void ConfigureClocks() {
  WDTCTL = WDTPW + WDTHOLD; // Stop WDT.
  DCOCTL = 0; // Select lowest DCOx and MODx settings.
  BCSCTL1 = CALBC1_1MHZ; // Set DCO.
  DCOCTL = CALDCO_1MHZ;
}

void ConfigurePorts() {
  P2DIR |= 0xFF; // All P2.x outputs.
  P2OUT &= 0x00; // All P2.x reset.
  P1SEL |= RXD + TXD ; // P1.1 = RXD, P1.2=TXD.
  P1SEL2 |= RXD + TXD ; // P1.1 = RXD, P1.2=TXD.
  P1DIR |= RXLED + TXLED;
  P1OUT &= 0x00;
}

void ConfigureUart() {
  UCA0CTL1 |= UCSSEL_2; // SMCLK.
  UCA0BR0 = 0x08; // 1MHz 115200.
  UCA0BR1 = 0x00; // 1MHz 115200.
  UCA0MCTL = UCBRS2 + UCBRS0; // Modulation UCBRSx = 5.
  UCA0CTL1 &= ~UCSWRST; // Initialize USCI state machine.
}

void ConfigureTimer() {
  // TODO(jmtaber129): Change this to generate an interrupt every 2.778ms
  // (360Hz) for analog readings.
  TA1CCR0 = 65535;
  TA1CCTL0 = CCIE;
  TA1CTL = TASSEL_2+ID_3+MC_1;
}

#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer_A (void) {
  TA1CTL &= ~TAIFG;

  ++timer_count;

  // Toggle P1.0 for debugging purposes.
  P1OUT ^= BIT0;

  // TODO(jmtaber129): Read from the analog pin, serialize the reading, and add
  // the serialized reading to the queue.

  char buffer[10];
  buffer[0] = timer_count;
  buffer[1] = '\n';
  buffer[2] = '\0';
  uart_queue.Push(buffer);

  // The string that was just pushed to the queue needs to be sent by the main
  // loop, so exit LPM.
  LPM0_EXIT;

  return;
}
