/*
 * flashMemory.c
 *
 *  Created on: Jan 13, 2013
 *      Author: bierma2
 *  Revised on June 20, 2013
 *  	Author: Andrew Bierman
 *  Revised on: Jan 23, 2014
 *  	Author: Andrew Bierman
 *  	findNextAddrBankB(void) was corrected (line 218) and added loop counter to while loop conditional
 */

#include "msp430.h"
#include "main.h"
#include "flashMemory.h"
#include "calibrateMode.h"
#include "timers.h"

unsigned char* flashAddressLEDCurrent;

// Write an unsigned integer to flash memory Info D segment
void flashWriteUnsignedInt(unsigned int position, unsigned int data){
	unsigned short bGIE;
	int i;
	static unsigned char prevData[128];
	unsigned char* flashAddr = (unsigned char*)INFO_D_SEGMENT_ADDR;

	// Read the segment to preserve segment data
	for (i=0;i<128;i++){
		prevData[i] = *flashAddr++;
	}
	// Update data with new unsigned integer
	// Multiply position by 2 because integers saved are 2 bytes each, offset is zero for saved integers
	prevData[(position*2)] = (unsigned char)data; 	// Low byte
	prevData[(position*2 + 1)] = (unsigned char)(data>>8); 	// High byte

	// Write data back to flash
	bGIE  = (__get_SR_register() & GIE);  //save interrupt status
	__disable_interrupt();

	// Erase the segment
	flashAddr = (unsigned char*)INFO_D_SEGMENT_ADDR;
	FCTL3 = FWKEY;                            // Clear the lock bit
	FCTL1 = FWKEY+ERASE;                      // Set the Erase bit
	*flashAddr = 0;                           // Dummy write, to erase the segment

	// Write the data to the segment
	FCTL1 = FWKEY+WRT;                        // Set WRT bit for write operation
	for (i=0;i<128;i++){
		*flashAddr++ = prevData[i];                 // Write data to flash
	}

	FCTL1 = FWKEY;                            // Clear WRT bit
	FCTL3 = FWKEY+LOCK;                       // Set LOCK bit

	__bis_SR_register(bGIE); //restore interrupt status
}

// Write an alarm structure to flash memory
void flashWriteAlarm(unsigned int position, Calendar data){
	unsigned short bGIE;
	int i;
	static unsigned char prevData[128];
	unsigned char* flashAddr = (unsigned char*)INFO_D_SEGMENT_ADDR;

	// Read the segment to preserve segment data
	for (i=0;i<128;i++){
		prevData[i] = *flashAddr++;
	}
	// Update data with new unsigned integer
	// multiply position by 8 because alarm saved are 8 bytes each, alarm data starts at offset 10
	prevData[(position*8  + 10)] = data.Seconds;
	prevData[(position*8  + 11)] = data.Minutes;
	prevData[(position*8  + 12)] = data.Hours;
	prevData[(position*8  + 13)] = data.DayOfWeek;
	prevData[(position*8  + 14)] = data.DayOfMonth;
	prevData[(position*8  + 15)] = data.Month;
	prevData[(position*8  + 16)] = ((data.Year)>>8); 	// High byte
	prevData[(position*8  + 17)] = (data.Year); 		// Low byte

	// Store the number of onTimes set (numalarms)
	if (position < 7){ // numAlarms counts only the number of onTime alarms
		prevData[122]  = (unsigned char)(position + 1); 	// 122 = (6+7)*8 + 18
	}

	bGIE  = (__get_SR_register() & GIE);  //save interrupt status
	__disable_interrupt();

	// Erase the segment
	flashAddr = (unsigned char*)INFO_D_SEGMENT_ADDR;
	FCTL3 = FWKEY;                            // Clear the lock bit
	FCTL1 = FWKEY+ERASE;                      // Set the Erase bit
	*flashAddr = 0;                           // Dummy write, to erase the segment

	// Write the data to the segment
	FCTL1 = FWKEY+WRT;                        // Set WRT bit for write operation
	for (i=0;i<128;i++){
		*flashAddr++ = prevData[i];                 // Write data to flash
	}

	FCTL1 = FWKEY;                            // Clear WRT bit
	FCTL3 = FWKEY+LOCK;                       // Set LOCK bit

	 __bis_SR_register(bGIE); //restore interrupt status
}

// Initialize parameters and alarms from info flash (segment D)
void initFromFlash(void){
	int i;
	static unsigned char data[128];
	unsigned char* flashAddr = (unsigned char*)INFO_D_SEGMENT_ADDR;

	// Read the segment
	for (i=0;i<128;i++){
		data[i] = *flashAddr++;
	}
	pulseDur = (unsigned int)((unsigned int)data[0] + 256*(unsigned int)data[1]);
	pulseInt = (unsigned int)((unsigned int)data[2] + 256*(unsigned int)data[3]);
	pulseRep = (unsigned int)((unsigned int)data[4] + 256*(unsigned int)data[5]);
	calLeft  = (unsigned int)((unsigned int)data[6] + 256*(unsigned int)data[7]);
	calRight = (unsigned int)((unsigned int)data[8] + 256*(unsigned int)data[9]);
	for (i=0;i<7;i++){
		on_Times[i].Seconds = data[i*8 + 10 + 0];
		on_Times[i].Minutes = data[i*8 + 10 + 1];
		on_Times[i].Hours = data[i*8 + 10 + 2];
		on_Times[i].DayOfMonth = data[i*8 + 10 + 4];
		on_Times[i].Month = data[i*8 + 10 + 5];
		on_Times[i].Year = 256*(unsigned int)data[i*8 + 10 + 7] + (unsigned int)data[i*8 + 10 + 6];

		offTimes[i].Seconds = data[i*8 + 66 + 0];  // 10 + (7*8) = 66
		offTimes[i].Minutes = data[i*8 + 66 + 1];
		offTimes[i].Hours = data[i*8 + 66 + 2];
		offTimes[i].DayOfMonth = data[i*8 + 66 + 4];
		offTimes[i].Month = data[i*8 + 66 + 5];
		offTimes[i].Year = 256*(unsigned int)data[i*8 + 66 + 7] + (unsigned int)data[i*8 + 66 + 6];
	}
	numAlarms = (unsigned int)(data[122]);
}

// Write an bytes to flash memory MAIN_BANK_B_
void flashWriteDataBankB(unsigned char data[], int length){
	unsigned short bGIE;
	int i;

	if(flashAddressLEDCurrent < (unsigned char*)MAX_RECORD_ADDRESS){

		bGIE  = (__get_SR_register() & GIE);  //save interrupt status
		__disable_interrupt();
		// Write the data to the segment
		FCTL3 = FWKEY;                            // Clear the lock bit
		FCTL1 = FWKEY+WRT;                        // Set WRT bit for write operation
		for (i=0;i<length;i++){
			*flashAddressLEDCurrent++ = data[i];                 // Write data to flash
		}
		FCTL1 = FWKEY;                            // Clear WRT bit
		FCTL3 = FWKEY+LOCK;                       // Set LOCK bit

		__bis_SR_register(bGIE); //restore interrupt status
	}
}

// Erase flash main memory bank B
void flashEraseBankB(void){
	unsigned short bGIE;
	unsigned char* flashAddr;

	//wdReset_1000();
	wdOff();

	bGIE  = (__get_SR_register() & GIE);  //save interrupt status
	__disable_interrupt();

	while(BUSY & FCTL3){;                     // Check for busy flash

	}
	// Erase the segment
	flashAddr = (unsigned char*)MAIN_BANK_C_ADDR;
	FCTL3 = FWKEY;                            // Clear the lock bit
	FCTL1 = FWKEY+MERAS;                      // Set the mass erase bit to erase the whole bank
	*flashAddr = 0;                           // Dummy write, to erase the segment

	while(BUSY & FCTL3);                     // Check for erase completion

	//FCTL1 = FWKEY;                            // Clear WRT bit
	FCTL3 = FWKEY+LOCK;                       // Set LOCK bit
	wdReset_1000();

	__bis_SR_register(bGIE); //restore interrupt status

	flashAddressLEDCurrent = (unsigned char*)MAIN_BANK_C_ADDR; // reset memory location to start of main bank B
}

// Find next unused flash address in main bank B after reset
void findNextAddrBankB(void){
	int count,found = 0;
	static unsigned char value = 0, valuelow = 0;
	static long int lowAddr = MAIN_BANK_C_ADDR, highAddr = MAX_RECORD_ADDRESS;
	//static long int lowAddr = 0x00C400, highAddr = 0x0143FF;
	static long int flashAddress;

	lowAddr =  MAIN_BANK_C_ADDR;
	highAddr = MAX_RECORD_ADDRESS;
	count = 0;
	while(!found && count<20) { 	// maximum count is 12 (2^12 = 4096 = max records, so 12 is max iteration in bisection search algorithm
		count += 1;
		wdReset_1000();
		flashAddress = 8*((lowAddr + highAddr)/16); // round average down to multiple of 8 so flashAddress is always at the start of a record
		valuelow = *(unsigned char*)flashAddress;
		value = *(unsigned char*)(flashAddress+8);
		if((value == 255) && (valuelow != 255)) {
			found = 1;
			flashAddress += 8;
		}
		else if((value == 255) && (flashAddress > MAIN_BANK_C_ADDR)) {
			highAddr = flashAddress;
		}
		else if(flashAddress <= MAIN_BANK_C_ADDR) {
			flashAddress = MAIN_BANK_C_ADDR;
			found = 1;
		}
		else if(flashAddress >= MAX_RECORD_ADDRESS - 16){ 	// corrected 23Jan2015, previously was MAX_RECORD_ADDRESS - 8
			if(value!=255) flashAddress = MAX_RECORD_ADDRESS;
			else flashAddress = MAX_RECORD_ADDRESS - 8;
			found = 1;
		}
		else {
			lowAddr = flashAddress;
		}
	}
	//if(flashAddress!=MAIN_BANK_B_ADDR) flashAddress += 8;
	if(count==20) flashAddress = MAX_RECORD_ADDRESS;
	flashAddressLEDCurrent = (unsigned char*)flashAddress;
}

