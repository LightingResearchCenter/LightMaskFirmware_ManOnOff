//This file implements the basic i2c functions

#include "msp430.h"
#include "i2c.h"
#include "calibrateMode.h"

unsigned char i2caddress;
unsigned char tdata[66];
unsigned char rdata[12];
int nackFlag;

//setup i2c
void i2cActivate() {
	_DINT();
	//P3SEL |= 0x06;                            // Assign I2C pins to USCI_B0
	UCB0CTL1 |= UCSWRST;                      // Enable SW reset
	UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
	UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
	UCB0BR0 = 50;                             // fSCL = SMCLK/50 = ~80kHz for 4MHz DCO
	UCB0BR1 = 0;
	UCB0I2CSA = i2caddress;                      // set slave address
	UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
	_BIS_SR(GIE);           		 		  // General Interrupt Enable
}

// Write I2C data to the DAC
void writeDAC(unsigned int level) {
	unsigned int DACcount = 0;

	DACcount = (unsigned int)((((long int)level)*1000)/(long int)calLeft); 	//level in W/m^2, calLeft in milliW/m^2/count
	i2caddress = 0x4C;
	tdata[0] = (unsigned char)(DACcount>>8); 	// high byte
	tdata[1] = (unsigned char)DACcount;		// low byte
	i2cTransmit(tdata,2,1);
	DACcount = (unsigned int)((((long int)level)*1000)/(long int)calRight); 	//level in W/m^2, calLeft in milliW/m^2/count
	i2caddress = 0x4D;
	tdata[0] = (unsigned char)(DACcount>>8); 	// high byte
	tdata[1] = (unsigned char)DACcount;		// low byte
	i2cTransmit(tdata,2,1);
}

//transmit data over i2c
void i2cTransmit(unsigned char *data, int datalength, int stop) {
	int i;

	UCB0I2CSA = i2caddress;
	//UCB0IE |= UCTXIE;
	UCB0IE &= ~UCRXIE;
	// _BIS_SR(GIE);           // General Interrupt Enable should be restored upon exiting interrupt (RETI)
	nackFlag = 0;
	UCB0CTL1 |= UCTR + UCTXSTT;                   // I2C TX, start condition
	UCB0IE |= UCNACKIE; 	// enable nack interrupt
	//while (UCB0CTL1 & UCTXSTT); 		// wait until slave acknowledges address

	for(i = 0; i < datalength; i++) {
		if (nackFlag){ 		//(UCB0IFG & UCNACKIFG){
			nackFlag = 0;
			//UCB0CTL1 |= UCTXSTP;
			break; 					// if Nack then break out of loop
		}
		UCB0TXBUF = data[i];   	  				  // Load TX buffer

		//turn off CPU while waiting for next
		UCB0IE |= UCTXIE;                          // Enable TX interrupt
		//LPM0;
		__bis_SR_register(LPM0_bits + GIE); 	//enter low power mode
		//notDone = 1;
		//while (notDone);
	}

	if(stop) {
		UCB0CTL1 |= UCTXSTP;
		while (UCB0CTL1 & UCTXSTP);		  //ensure stop condition was sent
	}
	UCB0IE &= ~UCTXIE;
	UCB0IE &= ~UCNACKIE;
}

//receive data over i2c
void i2cReceive(unsigned char *data, int datalength) {
	UCB0I2CSA = i2caddress;
	UCB0CTL1 &= ~UCTR;
	UCB0IE &= ~UCTXIE;
	UCB0IE |= UCRXIE;
	UCB0CTL1 |= UCTXSTT;		  // I2C TX, start condition
	data[0] = UCB0RXBUF;

	//turn off CPU while waiting for next
	UCB0IE |= UCRXIE;                          // Enable RX interrupt
	LPM0;
	int i = 0;
	for(i = 0; i < datalength; i++) {
		data[i] =  UCB0RXBUF;

		//turn off CPU while waiting for next
		UCB0IE |= UCRXIE;                          // Enable RX interrupt
		LPM0;									  //enter low power mode
	}
	UCB0CTL1 |= UCTXSTP;
	while (UCB0CTL1 & UCTXSTP);		  //ensure stop condition was sent
	UCB0IE &= ~UCRXIE;
}

//i2c Tx/Rx interrupt vector
//on completion of Tx/Rx should clear interrupt flags and exit LPM
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
  switch(__even_in_range(UCB0IV,12))
  {
  case  0: break;                           // Vector  0: No interrupts
  case  2: break;                           // Vector  2: ALIFG
  case  4:            		                // Vector  4: NACKIFG
  	  nackFlag = 1;
  	  __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
  	  break;
  case  6: break;                           // Vector  6: STTIFG
  case  8: break;                           // Vector  8: STPIFG
  case 10: 	                           // Vector 10: RXIFG
	  UCB0IFG &= ~UCRXIFG;                  // Clear USCI_B0 TX interrupt flag
	  __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
	  break;
  case 12:                                  // Vector 12: TXIFG
	  //notDone = 0;
	  UCB0IFG &= ~UCTXIFG;                  // Clear USCI_B0 TX interrupt flag
	  __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
    break;
  default: break;
  }
}



