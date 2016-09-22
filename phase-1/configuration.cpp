#include "configuration.h"

#include <msp430g2553.h>

void Configuration::Init(int* adc_sample) {
  ConfigureClocks();
  ConfigurePorts();
  ConfigureUart();
  ConfigureAdc(adc_sample);
  ConfigureTimer();

  __bis_SR_register(GIE);
}

void Configuration::ConfigureClocks() {
  // Note: If you modify clock speeds or sources, you need to change the UART
  // and timer configuration methods, too.
  WDTCTL = WDTPW + WDTHOLD; // Stop WDT.
  DCOCTL = CALDCO_1MHZ;
  BCSCTL1 = CALBC1_1MHZ; // Set DCO.
}

void Configuration::ConfigurePorts() {
  P2DIR |= 0xFF; // All P2.x outputs.
  P2OUT &= 0x00; // All P2.x reset.
  P1SEL |= RXD + TXD + BIT5; // P1.1 = RXD, P1.2 = TXD. P1.5 = ADC.
  P1SEL2 |= RXD + TXD ; // P1.1 = RXD, P1.2=TXD.
  P1DIR |= RXLED + TXLED;
  P1OUT &= 0x00;
}

void Configuration::ConfigureUart() {
  UCA0CTL1 |= UCSSEL_2; // SMCLK.
  UCA0BR0 = 0x08; // 1MHz 115200.
  UCA0BR1 = 0x00; // 1MHz 115200.
  UCA0MCTL = UCBRS2 + UCBRS0; // Modulation UCBRSx = 5.
  UCA0CTL1 &= ~UCSWRST; // Initialize USCI state machine.
  UC0IE |= UCA0RXIE; // Enable USCI_A0 RX interrupt.
}

void Configuration::ConfigureTimer() {
  // Configure timer for 360Hz.
  TA1CCR0 = 2778;  // Generate an interrupt every 2.778ms.
  TA1CCTL0 = CCIE;
  TA1CTL = TASSEL_2 + ID_0 + MC_1;
}

void Configuration::ConfigureAdc(int* dtc_address) {
  ADC10CTL1 = INCH_3 + ADC10DIV_3;
  ADC10CTL0 = SREF_0 + ADC10SHT_3 + ADC10ON + ADC10IE;
  ADC10AE0 |= BIT3;
  ADC10SA = (int)dtc_address;
  ADC10DTC1 = 1;  // Only keep the latest sample.
  ADC10DTC0 = ADC10CT + ADC10FETCH;
  __delay_cycles(1000);  // Wait for the ADC reference to settle.
  ADC10CTL0 |= ENC + ADC10SC;
}
