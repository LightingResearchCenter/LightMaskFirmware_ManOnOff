/*
 * calibrateMode.c
 *
 *  Created on: February 14, 2013
 *      Author: Andrew Bierman
 */

#include "msp430.h"
#include "timers.h"
#include "i2c.h"
#include "main.h"
#include "flashMemory.h"

unsigned int calLeft = 262, calRight = 262; 	// Irradiance calibration milliWatts/m^2 per DAC count
static unsigned int A0result;
static unsigned int A1result;
static unsigned int A11result;

void calFlash(unsigned int irrad, unsigned int duration){ 	// irradiance in milliWatts/m^2, duration in milliseconds

	USBtiming = 1; 		// Can't go into LMP3 when connected to USB (set in USBEventHandling.c, but set here again to make sure)
	writeDAC(irrad); 				// Mask on
	tbStart((float)duration); 		// Pulse duration in milliseconds
	__bis_SR_register(LPM0_bits + GIE); 	// Enter LPM0 (USB timing)
	__no_operation();
	writeDAC(0); 					// Mask off
}

// Monitor LED array current and battery voltage
void checkLEDCurrent(void) {
	int i;
	static unsigned char data[8];

	while(REFCTL0 & REFGENBUSY);              // If reference generator busy, WAIT
	REFCTL0 |= REFVSEL_1+REFON;               // Select internal reference = 2.0V, turn Reference ON
	for ( i=0; i<50; i++);                    // Delay for reference start-up, also delay for I2C DAC to settle
	ADC12CTL0 = ADC12ON+ADC12MSC+ADC12SHT0_12; 	// Turn on ADC12, set sampling time to 1024 ADC12OSC cycles (was ADC12SHT0_08)
	ADC12CTL1 = ADC12SHP+ADC12CONSEQ_1; 		// Use sampling timer, single sequence
	ADC12MCTL0 = (ADC12SREF_1 + ADC12INCH_0); 	// Vr+ = Vref+ and Vr- = AVss
	ADC12MCTL1 = (ADC12SREF_1 + ADC12INCH_1); 	// Vr+ = VeRef+ and Vr- = AVss
	ADC12MCTL2 = (ADC12SREF_1 + ADC12INCH_11 + ADC12EOS); 	// Vr+ = Vref+ and Vr- = AVss + end of conversion sequence

	ADC12IE = 0x04;                           // Enable ADC12IFG.2
	ADC12CTL0 |= ADC12ENC;                    // Enable conversions
	ADC12CTL0 |= ADC12SC;                     // Start conversion - software trigger

	__bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, Enable interrupts
	__no_operation();                         // For debugger
	while(REFCTL0 & REFGENBUSY);              // If reference generator busy, WAIT
	REFCTL0 &= ~REFON;           		// Turn Reference OFF
	__no_operation();                         // For debugger

	currentTime = RTC_getCalendarTime();
	data[0] = currentTime.DayOfMonth;
	data[1] = currentTime.Hours;
	data[2] = currentTime.Minutes;
	data[3] = currentTime.Seconds;
	data[5] = (unsigned char)(A0result>>8); 	// High byte
	data[4] = (unsigned char)A0result; 	// Low byte
	data[7] = (unsigned char)(A1result>>8); 	// High byte
	data[6] = (unsigned char)A1result; 	// Low byte
	flashWriteDataBankB(data, 8);
}

// Write Current Log Separator
void writeCurrentLogSeparator(void){
	static unsigned char data[8];

	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
	flashWriteDataBankB(data, 8);
}

// Check battery voltage
unsigned int checkBattery(void){
	int i;
	while(REFCTL0 & REFGENBUSY);              // If reference generator busy, WAIT
	REFCTL0 |= REFVSEL_1+REFON;               // Select internal reference = 2.0V, turn Reference ON
	for ( i=0; i<50; i++);                    // Delay for reference start-up, also delay for I2C DAC to settle
	ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
	ADC12CTL1 = ADC12SHP;                     // Use sampling timer (single sample by default)
	ADC12MCTL0 = (ADC12SREF_1 + ADC12INCH_11); 	// Vr+ = Vref+ and Vr- = AVss + end of conversion sequence
	ADC12IE = 0x0001;                         // Enable ADC12IFG.0
	ADC12CTL0 |= ADC12ENC;                    // Enable conversions
	ADC12CTL0 |= ADC12SC;                     // Start conversion - software trigger

	__bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, Enable interrupts
	__no_operation();
	while(REFCTL0 & REFGENBUSY);              // If reference generator busy, WAIT
	REFCTL0 &= ~REFON;           			  // Turn Reference OFF
	return A11result;
}

#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
{
  // static unsigned int index = 0;

  switch(__even_in_range(ADC12IV,34))
  {
  case  0: break;                          // Vector  0:  No interrupt
  case  2: break;                          // Vector  2:  ADC overflow
  case  4: break;                          // Vector  4:  ADC timing overflow
  case  6:		                           // Vector  6:  ADC12IFG0
  	  A11result = ADC12MEM0;	       // Move A0 results, IFG is cleared
  	  ADC12CTL0 &= ~ADC12ENC;               // disable conversions
  	  ADC12CTL0 &= ~ADC12SC;                // Stop conversion - software trigger
  	  ADC12IE &= ~0x0001;                   // disable ADC12IFG.0
  	  ADC12CTL0 &= ~ADC12ON; 				// Turn off ADC
  	  __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
  	break;
  case  8: break;                          // Vector  8:  ADC12IFG1
  case 10: 		                           // Vector 10:  ADC12IFG2
	    A0result = ADC12MEM0;           // Move A0 results, IFG is cleared
	    A1result = ADC12MEM1;           // Move A1 results, IFG is cleared
	    A11result = ADC12MEM2;           // Move A2 results, IFG is cleared
	    //index++;                      // Increment results index
	    ADC12CTL0 &= ~ADC12ENC;         // disable conversions
	    ADC12CTL0 &= ~ADC12SC;          // Stop conversion - software trigger
	    ADC12IE &= ~0x0004;             // disable ADC12IFG.2
	    ADC12CTL0 &= ~ADC12ON; 			// Turn off ADC
	    __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0

	  break;
  case 12: break; 							// Vector 12:  ADC12IFG3
  case 14: break;                           // Vector 14:  ADC12IFG4
  case 16: break;                           // Vector 16:  ADC12IFG5
  case 18: break;                           // Vector 18:  ADC12IFG6
  case 20: break;                           // Vector 20:  ADC12IFG7
  case 22: break;                           // Vector 22:  ADC12IFG8
  case 24: break;                           // Vector 24:  ADC12IFG9
  case 26: break;                           // Vector 26:  ADC12IFG10
  case 28: break;                 		    // Vector 28:  ADC12IFG11
  case 30: break;                           // Vector 30:  ADC12IFG12
  case 32: break;                           // Vector 32:  ADC12IFG13
  case 34: break;                           // Vector 34:  ADC12IFG14
  default: break;
  }
}
