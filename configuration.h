#ifndef ECG_SYSTEM_PROJECT_LAB_CONFIGURATION_H_
#define ECG_SYSTEM_PROJECT_LAB_CONFIGURATION_H_

#define TXLED BIT0
#define RXLED BIT6
#define TXD BIT2
#define RXD BIT1

class Configuration {
  public:
    // Configures MSP430G2553 registers.  Uses 'adc_sample' to configure ADC
    // DTC.
    static void Init(int* adc_sample);
  private:
    static void ConfigureClocks();
    static void ConfigurePorts();
    static void ConfigureUart();
    static void ConfigureTimer();

    // Configures the ADC DTC to load ADC10MEM into 'dtc_address'.
    static void ConfigureAdc(int* dtc_address);
};

#endif /* ECG_SYSTEM_PROJECT_LAB_CONFIGURATION_H_ */
