/*
 * flashMemory.h
 *
 *  Created on: Jan 13, 2013
 *      Author: bierma2
 */

#ifndef FLASHMEMORY_H_
#define FLASHMEMORY_H_

//#define INFO_D_SEGMENT_ADDR 0x001800
#define INFO_D_SEGMENT_ADDR 0x001880    // Using address for info C segment, must match pragma init in main.c
#define MAIN_BANK_C_ADDR 0x014400
#define MAX_RECORD_ADDRESS 0x01C3FF

#include "main.h"

void flashWriteUnsignedInt(unsigned int position, unsigned int data);
void flashWriteAlarm(unsigned int position, Calendar data);
void initFromFlash(void);
void flashWriteDataBankB(unsigned char data[], int length);
void flashEraseBankB(void);
void findNextAddrBankB(void);

extern unsigned char* flashAddressLEDCurrent;

#endif /* FLASHMEMORY_H_ */
