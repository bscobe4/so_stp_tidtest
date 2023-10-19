/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

/**
 * main.c
 */

#include <msp430.h>
#include <stdint.h>

#define    TXLENGTH      12     // Number of data bytes in each message transmitted by the UART
#define    ADC_DATA_SIZE 6      // Number of bytes used to store ADC data (when the ADC is configured in 10-bit mode, the 3 ADC channels require 6 bytes.
#define STARTMASKVALUE 255      // value of start mask byte at the beginning of data message send by the UART
#define STOPMASKVALUE 255       // value of stop mask byte at the beginning of data message send by the UART

// Persistent variable stored in FRAM. Used to count the number of time the MSP430 boots. Options for different compilers. This byte is stored in the FRAM when the program is flashed to the MSP430, and retains its value between hard resets.
// See forum post below regarding possible issues using persistent attribute with GCC:
// https://e2e.ti.com/support/microcontrollers/msp-low-power-microcontrollers-group/msp430/f/msp-low-power-microcontroller-forum/564231/msp430fr5969-gcc-v5-3-0-219-using-persistent-attribute
#if defined(__TI_COMPILER_VERSION__)
#pragma PERSISTENT(boot_tracker)
static uint8_t boot_tracker = 1;
#elif defined(__IAR_SYSTEMS_ICC__)
__persistent static uint8_t boot_tracker = 1;
#elif defined(__GNUC__)
static uint8_t __attribute__((persistent)) boot_tracker = 1;
#else
#error Compiler not supported!
#endif

//Global variables
unsigned char ADC_Data[ADC_DATA_SIZE];                              // Array to store ADC data from 3 instances of ADC10MEM0 register. Size 6 assuming 10-bit format (two bytes, for which only the two LSBs matter on the second byte.
unsigned char ADC_Chan[3] = {ADC10INCH_0,ADC10INCH_1,ADC10INCH_3};  // Each index has the value of the ADC10INCH bits of ADC10MCTL0 associated with the ADC channel being used
volatile unsigned int ADC_ChanIndex = 0;                            // Current index of ADC_Chan used

volatile uint8_t UART_TxBuff[TXLENGTH];                             //UART Transmit Buffer with following format:
                                                                    //start mask [0] ADC data [1:6], byte showing status of both motor controller fault bits [7], 16 bit counter [8:9], number of times system boots [10], end mask [11]

unsigned int i = 0;                                                 //Counter for incrementing index of UART Tx
volatile char outputCount16lo = 0;                                  // low byte of 16-bit counter for output buffer
volatile char outputCount16hi = 0;                                  // high byte of 16-bit counter for output buffer
uint8_t initialized = 0;
uint8_t start_serialTx = 0;

/*
#########################
# Function Declarations #
#########################
*/

//Increment the value of the boot counter stored in FRAM
void initBootMemory()
{
    boot_tracker += 1;
}

//Initialize GPIO pins
void initGPIO()
{
    //P1.0 --> ISENA (MAX22204)
    //P1.1 --> ISENB (MAX22204)
    //P1.3 --> IMON_out (LT8611)
    P1SEL0 |= BIT0 + BIT1 + BIT3;       //Tertiary function selected for P1.0, P1.1 and P1.2
    P1SEL1 |= BIT0 + BIT1 + BIT3;

    //P1.2 --> SLEEP2 (DRV8424)
    //P1.4 --> STEP2  (DRV8424)
    P1DIR |= BIT2 + BIT4;               //P1.4 output
    P1OUT |= BIT2 + BIT4;               //P1.4 high

    //P2.4 --> STEP1 (MAX22204)
    P2DIR |= BIT4;                      //P2.4 output
    P2OUT |= BIT4;                      //P2.4 high

    //P2DIR |= BIT6;                      //P2.4 output *DEBUG*     (MSP430 EVM board does not have access to P2.4, so this can be used to probe the TA1 output for STEP1)
    //P2OUT |= BIT6;                      //P2.4 high   *DEBUG*

    //P2.0 --> UCA0TXD (UART TX)
    //P2.1 --> UCA0RXD (UART RX)
    P2SEL1 |= BIT0 | BIT1;              //Select secondary function for P2.0 and P2.1 (UART)
    P2SEL0 &= ~(BIT0 | BIT1);

    //P3.0 --> DIR2 (DRV8424)
    //P3.1 --> EN2 (DRV8424)
    //P3.2 --> FAULT2 (DRV8424)
    //P3.5 --> FAULT1 (MAX22204)
    //P3.6 --> DIR1 (MAX22204)
    //P3.7 --> EN1 (MAX22204)
    P3DIR |= BIT0 + BIT1 + BIT6 + BIT7; //P3.6 and P3.7 output
    P3DIR &= ~(BIT2 + BIT5);               //P3.5 input
    P3OUT |= BIT0 + BIT1 + BIT6 + BIT7; // P3.6 and P3.7 High

}

// Initialize three timers used to toggle the STEP and DIR inputs for the MAX22204 and DRV8424.
// STEP1 and STEP2 are toggled at a rate of 0.05 Hz by different timers (TA1 and TB0 respectively).
// Both DIR signals are toggled by the same timer, TA0, at a rate of 1Hz.
// Note that they are toggled at 0.05 Hz and 1 Hz, so the periods of the square waves are 0.1 Hz and 2 Hz
void initTimers()
{
    CSCTL0_H = 0xA5;                    // CSKEY password. Must be 0x0A5 when writing in word mode
    CSCTL1 |= DCOFSEL0 + DCOFSEL1;      // Set max DCO setting as 11b = 8MHz (since DCORSEL = 0 upon reset)
    CSCTL2 = SELA_3 + SELS_3 + SELM_3;  // Set ACLK = SMCLK = MCLK = DCO
    CSCTL3 = DIVA_5 + DIVS_3 + DIVM_3;  // Set dividers to ACLK = 1/32, SMCLK = 1/8 and MCLK = 1/8

    //Timer A1 setup (STEP1 MAX22204)
    TA1CTL = TASSEL_2 + MC_2 + ID_0;    // Configure TA1 to be sourced from SMCLK, in CONTINUOUS mode, with a unity clock divider
    TA1CCTL0 = CCIE;                    // TACCR0 interrupt enabled
    TA1CCR0 = 50000;                    // Triggers TACCR0 interrupt 20 times/ sec assuming SMCLK / 8 = 8MHz/ 8 = 1MHz

    //Timer B0 setup (STEP2 DRV8424)
    TB0CTL = TBSSEL_2 + MC_2 + ID_0;    // Configure TB0 to be sourced from SMCLK, in CONTINUOUS mode, with a unity clock divider
    TB0CCTL0 = CCIE;                    // TBCCR0 interrupt enabled
    TB0CCR0 = 50000;                    // Triggers TB0CCR0 interrupt 20 times/ sec assuming SMCLK / 8 = 8MHz/ 8 = 1MHz

    //Timer A0 setup (DIR1 and DIR2 for MAX22204 and DRV8424 respectively)
    TA0CTL = TASSEL_1 + MC_2 + ID_3;    // Configure TA1 to be sourced from ACLK, in CONTINUOUS mode, with a 1/8 clock divider
    TA0CCTL0 = CCIE;                    // TACCR0 interrupt enabled
    TA0CCR0 = 31200;                    // Triggers TA1CCR0 interrupt every 1 sec assuming ACLK/ 32 = 8 MHz/ 32 = 250KHz


}

//Initialize ADC settings
void initADC()
{
    //Single channel read implementation (instead of sequence read)
    ADC10CTL0   |= ADC10SHT_2 + ADC10ON;        // ADC10ON, Sample and Hold time =16 ADC clks
    ADC10CTL1   |= ADC10SHP + ADC10CONSEQ_0;    // Sampling timer, software trigger, single channel per conversion
    ADC10CTL2   |= ADC10RES;                    // Conversion resolution 10-bits
    ADC10MCTL0  |= ADC10INCH_0;                 // Initial input channel is 0; Vref=AVCC
    ADC10IE     |= ADC10IE0;                    // Enable ADC conversion complete interrupt

}

//Initialize UART settings
void initUART()
{
    UCA0CTL1 |= UCSWRST;    // Put UCA0 into reset to change registers
    UCA0CTL1 = UCSSEL_1;    // Set ACLK as UCBRCLK --> 250kHz (this is the clock used by the UART to

    //UCA0CTL0 |= UCSPB_H;   //Two stop bits
    //UCA0CTL0 |= UCMSB_H;   //Set RX and TX shift registers to read MSB first
    //UCA0ABCTL |= UCABDEN;  //Enable automatic baud-rate detection

    //UCA0BRW = 1; UCA0MCTLW |= UCOS16 + UCBRF0 + UCBRF1 + UCBRF2 + UCBRS0; // For 9600 baud assuming FBRCLK = FACLK = 250kHz ; INT((250k / 9.6k) / 16) = 1 ;  UCBRW = 1
                                                                            // 250000/9600 - INT(250000/9600)=0.041
                                                                            // UCBRSx value = 0x01 (See UG)
                                                                            // UCBRFx value = 7
                                                                            // UCOS16 = 1

    UCA0BRW = 6;  UCA0MCTLW |= UCOS16 + UCBRF1 + UCBRS0 + UCBRS4; //For 2400 baud assuming FBRCLK = FACLK = 250kHz ; INT((250k / 2.4k) / 16) = 6 ;  UCBRW = 6
                                                                  // 250000/2400 - INT(250000/2400)=0.166
                                                                  // UCBRSx value = 0x11 (See UG table 18-4)
                                                                  // UCBRFx value = INT(0.166*16) = 2
                                                                  // UCOS16 = 1 since UC

    UCA0CTL1 &= ~UCSWRST;       // release from reset
    UCA0IE |= UCRXIE;           // Enable RX interrupt

}

void UART_Tx(){
    //Transmit all the bytes in sequence, waiting for the TX buffer to be ready.
    while(i < TXLENGTH)
    {
        while (!(UCA0IFG&UCTXIFG)); // USCI_A0 TX buffer ready?
        UCA0TXBUF = UART_TxBuff[i]; // TX -> RXed character
        i++;
        __delay_cycles(5000);
    }

    i = 0; //reset counter

    //Increment 16 bit counter as separate bytes
    if ( (outputCount16hi >= 255) && outputCount16lo >= 255){
        outputCount16hi = 0;
        outputCount16lo = 0;
    }
    else if (outputCount16lo >= 255){
        outputCount16hi++;
        outputCount16lo = 0;
    }
    else
    {
        outputCount16lo++;
    }
}

//#############
//# Main Loop #
//#############

int main(void)
{
    //Run init functions once
    if (initialized == 0){
        WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

        initGPIO();
        initTimers();
        initADC();
        initUART();
        initBootMemory();
        //__no_operation(); //For breakpoint
        initialized = 1;
    }

	//main loop increments ADC channel, starts ADC conversion, then reenters low-power mode to wait for interrupts.
	while(1)
	{

	ADC10CTL0 &= ~ADC10ENC;                   // ADC10 disable in order to change ADC_Chan
	ADC10MCTL0 = ADC_Chan[ADC_ChanIndex / 2];       //Need to do = instead of |=

	//while (ADC10CTL1 & ADC10BUSY);
	__delay_cycles(5000);
	ADC10CTL0 |= ADC10ENC + ADC10SC;          // Start sampling and conversion
	__bis_SR_register(CPUOFF + GIE);       // Enter LPM0 w/ interrupt

	if(start_serialTx == 1){
	    ADC10IE     &= ~ADC10IE0;
	    start_serialTx = 0;
	    UART_Tx();
	    ADC10IE     |= ADC10IE0;                    // Enable ADC conversion complete interrupt
	}

	__delay_cycles(5000);
	__no_operation();



	}
}

//##############################
//# Interrupt Service Routines #
//##############################

//Timer interrupts assuming three timers in continuous mode

//TA1 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) Timer1_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  P2OUT ^= BIT4;     //Toggle MAX22204 STEP
  TA1CCR0 += 50000;  //Add Offset to TACCR0
}

//TB0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_B0_VECTOR
__interrupt void Timer_B (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_B0_VECTOR))) Timer_B (void)
#else
#error Compiler not supported!
#endif
{
  P1OUT ^= BIT4;     //Toggle DRV8424 STEP
  TB0CCR0 += 50000;  //Add Offset to TBCCR0

  //Assign values to data bytes to be transmitted by the UART

      UART_TxBuff[0] = STARTMASKVALUE; //Start mask

      UART_TxBuff[1] = ADC_Data[0]; //Ch0 Low Byte
      UART_TxBuff[2] = ADC_Data[1]; //CH0 High Byte
      UART_TxBuff[3] = ADC_Data[2]; //Ch1 Low Byte
      UART_TxBuff[4] = ADC_Data[3]; //CH1 High Byte
      UART_TxBuff[5] = ADC_Data[4]; //Ch2 Low Byte
      UART_TxBuff[6] = ADC_Data[5]; //CH2 High Byte

      //FAULT1 and FAULT2 byte
      //If P3.2 is high, make the 4 least significant bits high. If P3.5 is high, make the 4 most significant bits high.
      UART_TxBuff[7] = 0;
      if (P3IN & BIT2){
          UART_TxBuff[7] |= 0x0F;
      }

      if (P3IN & BIT5){
          UART_TxBuff[7] |= 0xF0;
      }

      //16- bit counter
      UART_TxBuff[8] = outputCount16lo; //Low byte
      UART_TxBuff[9] = outputCount16hi; //High byte

      UART_TxBuff[10] = boot_tracker-2; //Boot counter byte

      UART_TxBuff[11] = STOPMASKVALUE;  //Stop mask

      start_serialTx = 1;

      __bic_SR_register_on_exit(CPUOFF); //Exit low power mode
}

// Timer A0 interrupt service routine. Also transmits data message over UART
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A (void)
#else
#error Compiler not supported!
#endif
{
    P3OUT ^= BIT0 + BIT6;       //Toggle DIR1 and DIR2
    TA0CCR0 += 31200;           // Add Offset to TACCR0

    /*
    //Assign values to data bytes to be transmitted by the UART

    UART_TxBuff[0] = STARTMASKVALUE; //Start mask

    UART_TxBuff[1] = ADC_Data[0]; //Ch0 Low Byte
    UART_TxBuff[2] = ADC_Data[1]; //CH0 High Byte
    UART_TxBuff[3] = ADC_Data[2]; //Ch1 Low Byte
    UART_TxBuff[4] = ADC_Data[3]; //CH1 High Byte
    UART_TxBuff[5] = ADC_Data[4]; //Ch2 Low Byte
    UART_TxBuff[6] = ADC_Data[5]; //CH2 High Byte

    //FAULT1 and FAULT2 byte
    //If P3.2 is high, make the 4 least significant bits high. If P3.5 is high, make the 4 most significant bits high.
    UART_TxBuff[7] = 0;
    if (P3IN & BIT2){
        UART_TxBuff[7] |= 0x0F;
    }

    if (P3IN & BIT5){
        UART_TxBuff[7] |= 0xF0;
    }

    //16- bit counter
    UART_TxBuff[8] = outputCount16lo; //Low byte
    UART_TxBuff[9] = outputCount16hi; //High byte

    UART_TxBuff[10] = boot_tracker-2; //Boot counter byte

    UART_TxBuff[11] = STOPMASKVALUE;  //Stop mask

    start_serialTx = 1;

    __bic_SR_register_on_exit(CPUOFF); //Exit low power mode
    */

}

// ADC10 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC10_VECTOR))) ADC10_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(ADC10IV,12))
  {
    case  0: break;                          // No interrupt
    case  2: break;                          // conversion result overflow
    case  4: break;                          // conversion time overflow
    case  6: break;                          // ADC10HI
    case  8: break;                          // ADC10LO
    case 10: break;                          // ADC10IN
    case 12: ADC_Data[ADC_ChanIndex] = ADC10MEM0_L;     //Assign low byte
             ADC_Data[ADC_ChanIndex + 1] = ADC10MEM0_H; //Assign high byte

             //Determine next ADC channel to be read
             if (ADC_ChanIndex >= (ADC_DATA_SIZE - 2) )
             {
                 ADC_ChanIndex = 0;
             }
             else
             {
                ADC_ChanIndex += 2;
             }


             __bic_SR_register_on_exit(CPUOFF);
             break;                          // Clear LPM0 bits from 0(SR)
    default: break;
  }
}

/*
//UCA0 Interrupt Handler
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCA0IV,0x08))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
          __bic_SR_register_on_exit(CPUOFF); //DEBUG band-aid for one of the interrupts that stops ADC conversion from being triggered in the main loop
    break;

  case 4:break;                             // Vector 4 - TXIFG
  default: break;
  }
}
*/

