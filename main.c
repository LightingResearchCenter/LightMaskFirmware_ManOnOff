// firmware code for NIH Light Mask Controller
// Andrew Bierman
// November 16, 2012; Revised February 14, 2013
// Revised May 01, 2013 Fixed getCalRight command (was calLeft for both sides) line 435

// Major revision 16-May-2014
// Added check battery command which uses ADC to measure battery voltage
// Added LED current monitoring and saving to MSP430 flash
//   Added functions that read ADC and saves results, along with date-time stamps, to flash memory
//   Added function that erases block C of flash
//
// 26-Jun-2014 Revision: Added '#End of log#' text at the end of LED current log (lines 515-519)
// 		Added time delay of 10 ms between each log record transmit
//
// 23-Jan-2015 Revision: Corrected infinite loop in findNextAddrBankB(void) when current log is full
// 		and added loop counter in while conditional to ensure it stops.
//      Changed #pragma command that assigns constants to information memory, and edited lnk_msp430f5528.cmd accordingly.
//          Defined memory locations .infoD00, .infoD0A, and .infoD7A and removed .infoD
//          Defined memory locations .infoC00, .infoC0A, and .infoC7A and removed .infoC
//          Changed to using information memory C instead of D becasue of possibly stressing flash D when
//             programming device without erasing emeory. Must erase both main and information memory
//             when programming. This resolves why chips would be difficult to program and not verify.
//      Fixed initFromFlash() so it now uses the correct byte order when determining values from flash
//      Added initNumAlarms to flash initialization.
//  09-Feb-2015 Coded new version that always flashes LEDs regardless of alarm status
//      Added function void startFlashingRegardlessAlarm(void);
//      Placed function at start of program and in usbEventHandling.c at USB_handleVbusOffEvent();
//      Commented out code in RTC interrupt routine that switches LEDs on and off;

/*
 *****************************************************************
 * Change Mask ID number and Firmware version on lines 99 and 101
 * ***************************************************************
 */

#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/types.h"
#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/usb.h"
#include "F5xx_F6xx_Core_Lib/HAL_UCS.h"
#include "F5xx_F6xx_Core_Lib/HAL_PMM.h"
#include "F5xx_F6xx_Core_Lib/HAL_FLASH.h"
#include "timers.h"
#include "i2c.h"
#include "flashMemory.h"
#include "calibrateMode.h"
#include <string.h>

// HID includes
#include "USB_API/USB_HID_API/UsbHid.h"
#include "usbConstructs.h"
#include "main.h"
// #include <math.h>

// HID declarations
volatile BYTE bHIDDataReceived_event = FALSE; // Indicates data has been received without an open rcv operation
#define MAX_STR_LENGTH 64
#define PULSE_DUR_LIMIT 3000
#define PULSE_INT_LIMIT 300
#define PULSE_REP_LIMIT_HI 300
#define PULSE_REP_LIMIT_LOW 5

volatile Calendar currentTime,alarmTime;
volatile Calendar on_Times[7],offTimes[7]; // **** change type to constant to store in non-volatile flash *****
volatile Alarm nextAlarm;
int numAlarms = 3, armed = 1;
unsigned int pulseRep = 5; 		// units of seconds
unsigned int pulseDur = 900; 	// units of milliseconds
unsigned int pulseInt = 50; 	// units of W/m^2 (1 W/m^2 roughly = 4 DAC counts (0 to 4095)
int outputFlag = 0;
int flashFlagPrev = 0;
int flashFlag = 0;
int batteryFlag = 0; 		// Is the 20V battery connected?
int batteryEventFlag = 0; 	// Battery interrupt on P1.3


// Initialize segment D flash with on_Times and offTimes
//#pragma DATA_SECTION (initParam, "INFO_SEGMENT_D1");
#pragma DATA_SECTION (initParam, ".infoC00");
const unsigned int initParam[5] = {2000, 225, 30, 262, 262};
//#pragma DATA_SECTION (initTimes, "INFO_SEGMENT_D2");
#pragma DATA_SECTION (initTimes, ".infoC0A");
const Calendar initTimes[14] =  {{0,1,12,0,7,1,2013},
								 {0,3,12,0,7,1,2013},
								 {0,5,12,0,7,1,2013},
								 {0,7,12,0,7,1,2013},
								 {0,9,12,0,7,1,2013},
								 {0,11,12,0,7,1,2013},
								 {0,13,12,0,7,1,2013},
								 {0,2,12,0,7,1,2013},
								 {0,4,12,0,7,1,2013},
								 {0,6,12,0,7,1,2013},
								 {0,8,12,0,7,1,2013},
								 {0,10,12,0,7,1,2013},
								 {0,12,12,0,7,1,2013},
								 {0,14,12,0,7,1,2013}};
#pragma DATA_SECTION (initNumAlarms, ".infoC7A");
const int initNumAlarms = 3;

#pragma DATA_SECTION (maskID, ".infoD00");
const char maskID[10] = "R020";
#pragma DATA_SECTION (FirmwareVersion, ".infoD0A");
const char FirmwareVersion[32] = "ManualOnOff_12Feb2015";


void main(void) {
	static char pieceOfString[MAX_STR_LENGTH] = "";    // Holds the new addition to the string
	static char outString[MAX_STR_LENGTH] = "";        // Holds the outgoing string
	static char wholeString[MAX_STR_LENGTH] = "";     	// The entire input string from the last 'return'
	char * subStr; 						// Holds the 8 character command part of the received string
	static char monthStr[3], dayStr[3], hoursStr[3], minutesStr[3],secondsStr[3]; 	// for printing the time
	static char parameterStr[6]; 							// for printing pulseDur, pulseInt, pulseRep
	int n,index,i = 0,j;
	char badCommand = FALSE;
	static unsigned int WperSqM = 0, milliSec = 1000;
	static int LEDOn = 0,LEDOnPrev = 0;

	//wdOff();
	wdReset_16000(); 		// Reset watchdog timer for 16 second interval

	Init_StartUp();     	// Initialize clocks, power, I/Os
	currentTime = RTC_getCalendarTime();
	setRTC(); 				// Setup the RTC, Set time to currentTime. Commented-out temporary time set for testing
	//Init_Alarm(); 			// Initialize array of alarm times ***** Remove/change so that updated alarm values are used ****
	initFromFlash(); 		// Initialize parameters and alarm times from info flash (segment D)
	findNextAddrBankB(); 	// Find flash memory location where next LED current measure is saved
	i2cActivate(); 			// set up IO pins and clock for I2C communication with DAC
	nextAlarm = findNextOnAlarmIndex(numAlarms);
	setAlarm(nextAlarm);

	__enable_interrupt();

	USB_init();                     // Initialize the USB module

	// Enable all USB events
	USB_setEnabledEvents(kUSB_allUsbEvents);
	
	// If USB is already connected when the program starts up, then there won't be a USB_handleVbusOnEvent().
	// So we need to check for it, and manually connect if the host is already present.
	if (USB_connectionInfo() & kUSB_vbusPresent){
		USBtiming = 1;
		if (USB_enable() == kUSB_succeed){
			USB_reset();
	        USB_connect();
	    }
	}
	else{
		USBtiming = 0;
		USB_disable();
	    XT2_Stop();
	}
	// Is the 20V battery already connected?
	if (P1IN & 0x04){ 	// No battery
		batteryFlag = 0;
		P1IES |= 0x04; 			// P1.2 High/Low edge  Set interrupt for attachment of battery
		P1IFG &= ~0x04;       	// P1.2 IFG cleared
		P1OUT &= ~0x02; 		// Switch off battery MOSFET (set P1.1 low)
	}
	else{ 	// battery connected
		batteryFlag = 1;
		P1IES &= ~0x04; 		// P1.2 Low/High edge  Set interrupt for removal of battery
		P1IFG &= ~0x04;       	// P1.2 IFG cleared
		P1OUT |= 0x02; 			// Switch on battery MOSFET (set P1.1 high)
	}

	//checkLEDCurrent();
	writeCurrentLogSeparator();

	startFlashingRegardlessAlarm();

	while(1){
	    switch(USB_connectionState()){
	        case ST_USB_DISCONNECTED:
	        	P1IE |= 0x04;         	// P1.2 interrupt enabled (for 20V battery detection)
	        	if (flashFlag == flashFlagPrev){
	        		__bis_SR_register(LPM3_bits + GIE); 	// Enter LPM3 until VBUS-on event or RTC alarm, timerA or timerB interrupt, or P1.2 interrupt
	        		__no_operation();
	        	}
	        	flashFlagPrev = flashFlag;
	        	if (batteryEventFlag) batterySwitch();

	        	if (outputFlag){
	        		if (flashFlag){
	        			P1OUT |= 0x01; 			// turn on LED on port 1.0
	        			writeDAC(pulseInt); 	// turn LED mask on by programming DAC to pulseInt
	        			LEDOn = 1;
	        		}
	        		else{
	        			P1OUT &= ~0x01; // turn off LED on port 1.0
	        			writeDAC(0); 	// turn LED mask off by programming DAC to zero
	        			LEDOn = 0;
	        		}
	        		if(LEDOn != LEDOnPrev){
	        			for(i=0;i<20000;i++); // delay for approximately 2.5 ms
	        			checkLEDCurrent();
	        			LEDOnPrev = LEDOn;
	        		}
	        	}
	        	else{
	        		P1OUT &= ~0x01; // turn off LED on port 1.0
	        		writeDAC(0); 	// turn LED mask off by programming DAC to zero
	        		if(USBtiming==0) batterySwitchIdle(0); 	// Switch off battery to save charge only if not connected to USB
	        		LEDOn = 0;
	        	}
	        	break;

	        case ST_USB_CONNECTED_NO_ENUM:
	            break;

	        case ST_ENUM_ACTIVE:
	        	P1IE |= 0x04;         	// P1.2 interrupt enabled (for 20V battery detection)
	            __bis_SR_register(LPM0_bits + GIE);  // Enable interrupts with LPM0 entry
	            __no_operation();

	            if (batteryEventFlag) batterySwitch();

	            // Exit LPM on USB receive and perform a receive operation
	            if(bHIDDataReceived_event){                       // Some data is in the buffer; begin receiving a command
	            // Add bytes in USB buffer to theCommand
	            	hidReceiveDataInBuffer((BYTE*)pieceOfString,MAX_STR_LENGTH,HID0_INTFNUM); 	// Get the next piece of the string
	                strcat(wholeString,pieceOfString);
	                if(retInString(wholeString)){                                // Has the user pressed return yet?
	                	subStr = strtok(wholeString,":,-");
	                	for(i=0;i<MAX_STR_LENGTH;i++){    // Clear the string in preparation for the next one
	                		outString[i] = 0x00;
	                		pieceOfString[i] = 0x00;
	                	}
	                	while(1){ 	// Construct a dummy loop so that a break statement can be used to avoid testing all the if statements after the command is found
	                	if(!(strcmp(subStr, "setClock"))){
	                		subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                		if (subStr != NULL) currentTime.Year = Ascii2Num(subStr);
	                		subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                		if (subStr != NULL) currentTime.Month = Ascii2Num(subStr);
	                		subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                		if (subStr != NULL) currentTime.DayOfMonth = Ascii2Num(subStr);
	                		subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                		if (subStr != NULL) currentTime.Hours = Ascii2Num(subStr);
	                		subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                		if (subStr != NULL){
	                			currentTime.Minutes = Ascii2Num(subStr);
	                			badCommand = FALSE;
	                		}
	                		else badCommand = TRUE;

	                		if (!badCommand){
	                			RTC_calendarInit(currentTime);
	                			RTC_startClock();

	                           	strcpy(outString,"\r\nClock set successfully.");
	                           	//hidSendDataInBackground((BYTE*)outString,strlen(outString),HID0_INTFNUM,0);
	                           	hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                		}
	                		else{
	                			strcpy(outString,"\r\nClock NOT set.");
	                			//hidSendDataInBackground((BYTE*)outString,strlen(outString),HID0_INTFNUM,0);
	                			hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                		}
	                		break;
	                    }
	                    if(!(strcmp(subStr, "getClock"))){
	                    	currentTime = RTC_getCalendarTime();
	                    	num2ASCII((unsigned int)(currentTime.Month),monthStr,2);
	                    	num2ASCII((unsigned int)(currentTime.DayOfMonth),dayStr,2);
	                    	num2ASCII((unsigned int)(currentTime.Hours),hoursStr,2);
	                    	num2ASCII((unsigned int)(currentTime.Minutes),minutesStr,2);
	                    	num2ASCII((unsigned int)(currentTime.Seconds),secondsStr,2);

	                	  	strcpy(outString,"\r\nTime =  "); 	// Prepare the outgoing string
	                	  	strcat(outString,monthStr);
	                	  	strcat(outString,"/");
	                	  	strcat(outString,dayStr);
	                	  	strcat(outString," ");
	                	  	strcat(outString,hoursStr);
	                        strcat(outString,":");
	                        strcat(outString,minutesStr);
	                        strcat(outString,":");
	                        strcat(outString,secondsStr);
	                        strcat(outString,"\r\n");

	                        //hidSendDataInBackground((BYTE*)outString,strlen(outString),HID0_INTFNUM,0);
	                        hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                        break;
	                    }
	                    if(!(strcmp(subStr, "on_Times"))){
	                    	subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                    	if (subStr !=NULL) index = Ascii2Num(subStr);
	                    	subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                		if (subStr != NULL) on_Times[index].Year = Ascii2Num(subStr);
	                		subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                		if (subStr != NULL) on_Times[index].Month = Ascii2Num(subStr);
	                		subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                		if (subStr != NULL) on_Times[index].DayOfMonth = Ascii2Num(subStr);
	                		subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                		if (subStr != NULL) on_Times[index].Hours = Ascii2Num(subStr);
	                		subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                		if (subStr != NULL){
	                			on_Times[index].Minutes = Ascii2Num(subStr);
	                			badCommand = FALSE;
	                		}
	                		else badCommand = TRUE;

	                		if (!badCommand){
	                			numAlarms = index+1;
	                			flashWriteAlarm(index, on_Times[index]);

	                           	strcpy(outString,"\r\non_Time set successfully.");
	                           	//hidSendDataInBackground((BYTE*)outString,strlen(outString),HID0_INTFNUM,0);
	                           	hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                		}
	                		else{
	                			strcpy(outString,"\r\non_Time NOT set.");
	                			//hidSendDataInBackground((BYTE*)outString,strlen(outString),HID0_INTFNUM,0);
	                			hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                		}
	                		break;
	                    }
	                   if(!(strcmp(subStr, "offTimes"))){
	                    	subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                    	if (subStr !=NULL) index = Ascii2Num(subStr);
	                    	subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                    	if (subStr != NULL) offTimes[index].Year = Ascii2Num(subStr);
	                    	subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                    	if (subStr != NULL) offTimes[index].Month = Ascii2Num(subStr);
	                    	subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                    	if (subStr != NULL) offTimes[index].DayOfMonth = Ascii2Num(subStr);
	                    	subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                    	if (subStr != NULL) offTimes[index].Hours = Ascii2Num(subStr);
	                    	subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                    	if (subStr != NULL){
	                    	    offTimes[index].Minutes = Ascii2Num(subStr);
	                    	    badCommand = FALSE;
	                    	}
	                    	else badCommand = TRUE;

	                    	if (!badCommand){
	                    		flashWriteAlarm(index + 7, offTimes[index]); 	// add 7 because on-Times are in first 7 memory positions

	                    		strcpy(outString,"\r\noffTime set successfully.");
	                    		//hidSendDataInBackground((BYTE*)outString,strlen(outString),HID0_INTFNUM,0);
	                    		hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                    	}
	                    	else{
	                    		strcpy(outString,"\r\noff_Time NOT set.");
	                    		//hidSendDataInBackground((BYTE*)outString,strlen(outString),HID0_INTFNUM,0);
	                    		hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                    	}
	                    	break;
	                    }
	                    if(!(strcmp(subStr, "pulseDur"))){
	                    	subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                    	if (subStr != NULL) pulseDur = (unsigned int)Ascii2Num(subStr);
	                    	if (pulseDur > PULSE_DUR_LIMIT) pulseDur = PULSE_DUR_LIMIT;
	                    	flashWriteUnsignedInt(0, pulseDur); 		// position 0
	                    	num2ASCII(pulseDur,parameterStr,4);
	                    	strcpy(outString,"\r\npulseDur set to ");
	                    	strcat(outString,parameterStr);         // Prepare the outgoing string
	                    	strcat(outString,"\r\n");
	                    	//hidSendDataInBackground((BYTE*)outString,strlen(outString),HID0_INTFNUM,0);
	                    	hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                    	break;
	                    }
	                    if(!(strcmp(subStr, "pulseRep"))){
	                    	subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                    	if (subStr != NULL) pulseRep = Ascii2Num(subStr);
	                    	if (pulseRep > PULSE_REP_LIMIT_HI) pulseRep = PULSE_REP_LIMIT_HI;
	                    	if (pulseRep < PULSE_REP_LIMIT_LOW) pulseRep = PULSE_REP_LIMIT_LOW;
	                    	flashWriteUnsignedInt(2, pulseRep); 		// position 2
	                    	num2ASCII(pulseRep,parameterStr,4);
	                    	strcpy(outString,"\r\npulseRep set to ");
	                    	strcat(outString,parameterStr);         // Prepare the outgoing string
	                    	strcat(outString,"\r\n");
	                    	//hidSendDataInBackground((BYTE*)outString,strlen(outString),HID0_INTFNUM,0);
	                    	hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                    	break;
	                    }
	                    if(!(strcmp(subStr, "pulseInt"))){
	                    	subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                    	if (subStr != NULL) pulseInt = Ascii2Num(subStr);
	                    	if (pulseInt > PULSE_INT_LIMIT) pulseInt = PULSE_INT_LIMIT;
	                    	flashWriteUnsignedInt(1, pulseInt); 		// position 1
	                    	num2ASCII(pulseInt,parameterStr,4);
	                    	strcpy(outString,"\r\npulseInt set to ");
	                    	strcat(outString,parameterStr);         // Prepare the outgoing string
	                    	strcat(outString,"\r\n");
	                    	//hidSendDataInBackground((BYTE*)outString,strlen(outString),HID0_INTFNUM,0);
	                    	hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                    	break;
	                    }
	                    if(!(strcmp(subStr, "getOn"))){
	                    	for(i=0;i<numAlarms;i++){
	                    		alarmTime = on_Times[i];
	                    		num2ASCII((unsigned int)(alarmTime.DayOfMonth),dayStr,2);
	                    		num2ASCII((unsigned int)(alarmTime.Hours),hoursStr,2);
	                    		num2ASCII((unsigned int)(alarmTime.Minutes),minutesStr,2);
	                    		num2ASCII((unsigned int)(i),parameterStr,1);
	                    		strcpy(outString,"\r\nOn time ");
	                    		strcat(outString,parameterStr);
	                    		strcat(outString,": ");
	                    		strcat(outString,dayStr);         // Prepare the outgoing string
	                    		strcat(outString," ");
	                    		strcat(outString,hoursStr);
	                    		strcat(outString,":");
	                    		strcat(outString,minutesStr);
	                    		//strcat(outString,"\r\n");
	                    		hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                    	}
	                    	break;
	                    }
	                    if(!(strcmp(subStr, "getOff"))){
	                    	for(i=0;i<numAlarms;i++){
	                    		alarmTime = offTimes[i];
	                    		num2ASCII((unsigned int)(alarmTime.DayOfMonth),dayStr,2);
	                    		num2ASCII((unsigned int)(alarmTime.Hours),hoursStr,2);
	                    		num2ASCII((unsigned int)(alarmTime.Minutes),minutesStr,2);
	                    		num2ASCII((unsigned int)(i),parameterStr,1);
	                    		strcpy(outString,"\r\nOff time ");
	                    		strcat(outString,parameterStr);
	                    		strcat(outString,": ");
	                    		strcat(outString,dayStr);         // Prepare the outgoing string
	                    		strcat(outString," ");
	                    		strcat(outString,hoursStr);
	                    		strcat(outString,":");
	                    		strcat(outString,minutesStr);
	                    		//strcat(outString,"\r\n");
	                    		hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                    	}
	                    	break;
	                    }
	                    if(!(strcmp(subStr, "getInt"))){
	                    	num2ASCII(pulseInt,parameterStr,4);
	                    	strcpy(outString,"\r\npulseInt set to ");
	                    	strcat(outString,parameterStr);         // Prepare the outgoing string
	                    	strcat(outString,"W/m^2\r\n");
	                    	//hidSendDataInBackground((BYTE*)outString,strlen(outString),HID0_INTFNUM,0);
	                    	hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                    	break;
	                    }
	                    if(!(strcmp(subStr, "getDur"))){
	                    	num2ASCII(pulseDur,parameterStr,4);
	                    	strcpy(outString,"\r\npulseDur set to ");
	                    	strcat(outString,parameterStr);         // Prepare the outgoing string
	                    	strcat(outString,"milliseconds\r\n");
	                    	//hidSendDataInBackground((BYTE*)outString,strlen(outString),HID0_INTFNUM,0);
	                    	hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                    	break;
	                    }
	                    if(!(strcmp(subStr, "getRep"))){
	                    	num2ASCII(pulseRep,parameterStr,4);
	                    	strcpy(outString,"\r\npulseRep set to ");
	                    	strcat(outString,parameterStr);         // Prepare the outgoing string
	                    	strcat(outString,"seconds\r\n");
	                    	//hidSendDataInBackground((BYTE*)outString,strlen(outString),HID0_INTFNUM,0);
	                    	hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                    	break;
	                    }
	                    if(!(strcmp(subStr,"calLeft"))){
	                    	subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                    	if (subStr != NULL) calLeft = Ascii2Num(subStr);
	                    	flashWriteUnsignedInt(3, calLeft); 		// position 3
	                    	num2ASCII(calLeft,parameterStr,4);
	                    	strcpy(outString,"\r\ncalLeft set to ");
	                    	strcat(outString,parameterStr);         // Prepare the outgoing string
	                    	strcat(outString,"\r\n");
	                    	//hidSendDataInBackground((BYTE*)outString,strlen(outString),HID0_INTFNUM,0);
	                    	hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                    	break;
	                    }
	                    if(!(strcmp(subStr,"calRight"))){
	                    	subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                    	if (subStr != NULL) calRight = Ascii2Num(subStr);
	                    	flashWriteUnsignedInt(4, calRight); 		// position 4
	                    	num2ASCII(calRight,parameterStr,4);
	                    	strcpy(outString,"\r\ncalRight set to ");
	                    	strcat(outString,parameterStr);         // Prepare the outgoing string
	                    	strcat(outString,"\r\n");
	                    	//hidSendDataInBackground((BYTE*)outString,strlen(outString),HID0_INTFNUM,0);
	                    	hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                    	break;
	                    }
	                    if(!(strcmp(subStr,"calFlash"))){
	                		subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                		if (subStr != NULL) WperSqM = Ascii2Num(subStr);
	                		subStr = strtok (NULL, ":,-"); 	// strtok function remembers pointer location from previous call
	                		if (subStr != NULL) milliSec = Ascii2Num(subStr);
	                    	calFlash(WperSqM, milliSec);
	                    	num2ASCII(WperSqM,parameterStr,4);
	                    	strcpy(outString,"\r\nW/m2 = ");
	                    	strcat(outString,parameterStr);         // Prepare the outgoing string
	                    	strcat(outString," milliSec = ");
	                    	num2ASCII(milliSec,parameterStr,5);
	                    	strcat(outString,parameterStr);
	                    	strcat(outString,"\r\n");
	                    	//hidSendDataInBackground((BYTE*)outString,strlen(outString),HID0_INTFNUM,0);
	                    	hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                    	break;
	                    }
	                    if(!(strcmp(subStr, "getCalLeft"))){
	                    	num2ASCII(calLeft,parameterStr,4);
	                    	strcpy(outString,"\r\ncalLeft set to ");
	                    	strcat(outString,parameterStr);         // Prepare the outgoing string
	                    	strcat(outString,"W/m^2 per step\r\n");
	                    	//hidSendDataInBackground((BYTE*)outString,strlen(outString),HID0_INTFNUM,0);
	                    	hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                    	break;
	                    }
	                    if(!(strcmp(subStr, "getCalRight"))){
	                    	num2ASCII(calRight,parameterStr,4);
	                    	strcpy(outString,"\r\ncalRight set to ");
	                    	strcat(outString,parameterStr);         // Prepare the outgoing string
	                    	strcat(outString,"W/m^2 per step\r\n");
	                    	//hidSendDataInBackground((BYTE*)outString,strlen(outString),HID0_INTFNUM,0);
	                    	hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                    	break;
	                    }
	                	if(!(strcmp(subStr, "getLEDCurrentLog"))){
	                		n = (int)(((long int)flashAddressLEDCurrent - MAIN_BANK_C_ADDR)/8);
	                		if((long int)flashAddressLEDCurrent==MAX_RECORD_ADDRESS) n++; // add one if memory bank is completely full
	                		static unsigned char* flashAddr = (unsigned char*)MAIN_BANK_C_ADDR;
	                		flashAddr = (unsigned char*)MAIN_BANK_C_ADDR;
	                		static unsigned int* flashAddrInt;
	                		strcpy(outString,"\r\nLED currents\r\n");
	                		for(i=0;i<n;i++){
	                			num2ASCII((unsigned int)*flashAddr++,dayStr,2);
	                		    num2ASCII((unsigned int)*flashAddr++,hoursStr,2);
	                		    num2ASCII((unsigned int)*flashAddr++,minutesStr,2);
	                		    num2ASCII((unsigned int)*flashAddr++,secondsStr,2);
	                		    flashAddrInt = (unsigned int*)flashAddr++;
	                		    num2ASCII(*flashAddrInt++,parameterStr,4);
	                		    flashAddr++;
	                		    strcat(outString,dayStr);
	                		    strcat(outString,",");
	                		    strcat(outString,hoursStr);
	                		    strcat(outString,",");
	                		    strcat(outString,minutesStr);
	                		    strcat(outString,",");
	                		    strcat(outString,secondsStr);
	                		    strcat(outString,",");
	                		    strcat(outString,parameterStr);
	                		    strcat(outString,",");
	                		    num2ASCII(*flashAddrInt,parameterStr,4);
	                		    flashAddr++;
	                		    flashAddr++; 	// index twice because last two values are integers
	                		    strcat(outString,parameterStr);
	                		    strcat(outString,"\r\n");
	                		    hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                		    for(j=0;j<MAX_STR_LENGTH;j++){    // Clear the string in preparation for the next one
	                		        outString[j] = 0x00;
	                		     }
	                		    tbDelay(10);
	                		}
	                		strcpy(outString,"# End of Log #\r\n");
	                		hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                		for(j=0;j<MAX_STR_LENGTH;j++){    // Clear the string in preparation for the next one
	                			outString[j] = 0x00;
	                		}
	                		break;
	                	}
	                	if(!(strcmp(subStr, "eraseLEDLog"))){
	                		strcpy(outString,"\r\nStart erasing log\r\n");
	                		hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                		flashEraseBankB();
	                		strcpy(outString,"\r\nLED current log erased\r\n");
	                		hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                		break;
	                	}
	                	if(!(strcmp(subStr, "checkBattery"))){
	                		num2ASCII(checkBattery(),parameterStr,4);
	                		strcpy(outString,"\r\nBattery voltage = ");
	                		strcat(outString,parameterStr);
	                		strcat(outString," (mV)\r\n");
	                		hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                		break;
	                	}
	                	if(!(strcmp(subStr, "getID"))){
	                		strcpy(outString,"\r\nMask ID\r\n");
	                		strcat(outString,maskID);
	                		strcat(outString,"\r\n");
	                		hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                		break;
	                	}
	                	if(!(strcmp(subStr, "getFirmwareVer"))){
	                		strcpy(outString,"\r\nFirmware Version\r\n");
	                		strcat(outString,FirmwareVersion);
	                		strcat(outString,"\r\n");
	                		hidSendDataWaitTilDone((BYTE*)outString,strlen(outString),HID0_INTFNUM,1000000);
	                		break;
	                	}
	                	// Handle other terminated strings
	                	strcpy(outString,"\r\nNo such command!\r\n\r\n");                   // Prepare the outgoing string
	                	hidSendDataInBackground((BYTE*)outString,strlen(outString),0,0);    // Send the response over USB
	                	break;
	                	} 	// Dummy while loop
	                    for(i=0;i<MAX_STR_LENGTH;i++){                        // Clear the string in preparation for the next one
	                    	wholeString[i] = 0x00;
	                	}
	                }
	                bHIDDataReceived_event = FALSE;
	            }
	            break;

	        case ST_ENUM_SUSPENDED:
	            __bis_SR_register(LPM3_bits + GIE);            // Enter LPM3, until a resume or VBUS-off event
	            break;

	        case ST_ENUM_IN_PROGRESS:
	             break;

	        case ST_ERROR:
	             break;

	        default:;
	    } // switch(USB_connectionState())
	}  // while(1)
}

/*----------------------------------------------------------------------------+
| System Initialization Routines                                              |
+----------------------------------------------------------------------------*/

// This function initializes the F5xx Universal Clock System (UCS):
// MCLK/SMCLK:  driven by the DCO/FLL, set to USB_MCLK_FREQ (a configuration constant defined by the Descriptor Tool) 8 Mhz Default, 4 Mhz crystal
// ACLK:  the internal REFO oscillator
// FLL reference:  the REFO
// This function anticipates any MSP430 USB device (F552x/1x/0x, F563x/663x), based on the project settings.

VOID Init_Clock(VOID)
{
#   if defined (__MSP430F563x_F663x)
		while(BAKCTL & LOCKIO)                    // Unlock XT1 pins for operation
      	BAKCTL &= ~(LOCKIO);                    // enable XT1 pins
     	// Workaround for USB7
    	UCSCTL6 &= ~XT1OFF;
#   endif

    //Initialization of clock module
    if (USB_PLL_XT == 2)
    {
#       if defined (__MSP430F552x) || defined (__MSP430F550x)
        	P5SEL |= 0x0C;                        // enable XT2 pins for F5529
#       elif defined (__MSP430F563x_F663x)
			P7SEL |= 0x0C;
#       endif
        // use REFO for FLL and ACLK
        //UCSCTL3 = (UCSCTL3 & ~(SELREF_7)) | (SELREF__REFOCLK);
        UCSCTL3 = (UCSCTL3 & ~(SELREF_7)) | (SELREF__XT1CLK);
        //UCSCTL4 = (UCSCTL4 & ~(SELA_7)) | (SELA__REFOCLK);
        UCSCTL4 = (UCSCTL4 & ~(SELA_7)) | (SELA__XT1CLK); 	// SMCLK and MCLK keep default settings (DCO/2)
        //UCSCTL4 = 0x00; //set clocks to XT1 as per programmer's guide

        Init_FLL(USB_MCLK_FREQ/1000, USB_MCLK_FREQ/32768);             // set FLL (DCOCLK)
        XT2_Start(XT2DRIVE_3);
    }
    else 	// This case is not used because we are using a separate crystal for USB_PLL
    {
#       if defined (__MSP430F552x) || defined (__MSP430F550x)
            P5SEL |= 0x10;                    // enable XT1 pins
#       endif
        UCSCTL3 = SELREF__REFOCLK;            // run FLL mit REF_O clock
        //UCSCTL4 = (UCSCTL4 & ~(SELA_7)) | (SELA__REFOCLK); // set ACLK = REFO
        //UCSCTL4 = 0xA4;
        //UCSCTL4 = 0x00; //set clocks to XT1 as per programmer's guide
        Init_FLL(USB_MCLK_FREQ/1000, USB_MCLK_FREQ/32768);       // set FLL (DCOCLK)
        XT1_Start(XT1DRIVE_3);
    }
}

//----------------------------------------------------------------------------

// This function initializes the I/Os.  It expects to be running on an MSP430 FET target board; so
// if using on another board, it's a good idea to double-check these settings.  It sets most of the
// I/Os as outputs, to eliminate floating inputs, for optimum power usage.
// This function anticipates any MSP430 USB device (F552x/1x/0x, F563x/663x), based on the project settings.

VOID Init_Ports(VOID)
{
    // Initialization of ports all unused pins as outputs with low-level

		P1OUT = 0x00; 	// P1.1 low to keep P-chan MOSFET off
	    P1SEL = 0x00;
	    P1DIR = 0xFB; 	// All outputs except P1.2 is input

	    P2OUT = 0x00; 	//
	    P2SEL = 0x00;
	    P2DIR = 0xFF; 	// All outputs

	    P3OUT    =  0x00;
	    P3SEL   |=  0x03;      // Assign I2C pins to USCI_B0
	    P3DIR    =  0xFC;      // All other pins  to output

	    P4OUT   =   0x01; 	// P4.0 high for power ps2 (I2C pullup)
	    P4SEL   =   0x00;
	    P4DIR   =   0xFF; 	// All outputs

	    P5OUT = 0x00;
	    //P5SEL = 0x3F; 	// P5.0, P5.1 external reference input, P5.2, P5.3 XT2, P5.4, P5.5 XT1
	    P5SEL = 0x3C; 	// P5.2, P5.3 XT2, P5.4, P5.5 XT1, when not using external reference
	    P5DIR = 0xFF; 	// All outputs

	    P6OUT = 0x00;
	    P6SEL |= 0x03; 	// Enable A/D channel A0, A1
	    //P6SEL = 0x00;
	    P6DIR = 0xFC; 	// P6.0 and P6.1 are inputs, all others are outputs
	    //P6SEL = 0x07; 	// P6.0, P6.1, P6.2, P6.3, P6.4 selected for analog inputs A0, A1, A2, A3, A4
	    //P6DIR = 0xF8; 	// P6.0, P6.1, P6.2, P6.3, P6.4 Inputs, All others Outputs

	    // PJDIR   =   0xFFFF; 	// Replace before stand-alone operation
	    // PJOUT   =   0x0000;
}


//----------------------------------------------------------------------------

VOID Init_StartUp(VOID)
{
   //unsigned short bGIE;
   //bGIE  = (__get_SR_register() &GIE);  //save interrupt status

   //__disable_interrupt();               // Disable global interrupts

    Init_Ports();                        // Init ports (do first ports because clocks do change ports)
    SetVCore(3);                         // USB core requires the VCore set to 1.8 volt, independ of CPU clock frequency
    Init_Clock();

    // taStart(); 	// time starts when alarm-on criteria is met
    //i2cActivate();

    //__bis_SR_register(bGIE); //restore interrupt status
}


#pragma vector = UNMI_VECTOR
__interrupt VOID UNMI_ISR(VOID)
{
    switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG))
    {
    case SYSUNIV_NONE:
      __no_operation();
      break;
    case SYSUNIV_NMIIFG:
      __no_operation();
      break;
    case SYSUNIV_OFIFG:
      UCSCTL7 &= ~(DCOFFG+0+0+XT2OFFG); // Clear OSC flaut Flags fault flags
      SFRIFG1 &= ~OFIFG;                                // Clear OFIFG fault flag
      break;
    case SYSUNIV_ACCVIFG:
      __no_operation();
      break;
    case SYSUNIV_BUSIFG:

      // If bus error occurred - the cleaning of flag and re-initializing of USB is required.
      SYSBERRIV = 0;            // clear bus error flag
      USB_disable();            // Disable
    }
}

// This function returns true if there's an 0x0D character in the string; and if so,
// it trims the 0x0D and anything that had followed it.
unsigned char retInString(char* string)
{
  BYTE retPos=0,i,len;
  char tempStr[MAX_STR_LENGTH] = "";

  strncpy(tempStr,string,strlen(string));     // Make a copy of the string
  len = strlen(tempStr);
  while((tempStr[retPos] != 0x21) && (retPos++ < len)); // Find 0x21 "!"; if not found, retPos ends up at len

  if(retPos<len)                              // If 0x21 was actually found...
  {
    for(i=0;i<MAX_STR_LENGTH;i++)             // Empty the buffer
      string[i] = 0x00;
    strncpy(string,tempStr,retPos);           // ...trim the input string to just before 0x0D
    return TRUE;                              // ...and tell the calling function that we did so
  }

  return FALSE;                               // Otherwise, it wasn't found
}

// This returns the value of the number on the specified line of the log_info.txt file
int Ascii2Num(char* string){
	int value,number = 0;

	while (string[0] !=NULL){  // while not equal to the end-of-string (NULL) character
		if (string[0] > 0x2F && string[0] < 0x3A){  // if character is ascii numeral
			value = string[0] - 0x30;  // convert ascii to number
			number = number*10 + value;  // multiply by 10 and add next digit
		}
		string++;
	}
	return (number);
}

// This returns an ASCII character string representing the number passed to it
void num2ASCII(unsigned int number, char* str, int numDigits){
	int k;
	unsigned char digit;
	for (k=numDigits-1;k>=0;k--){
		digit = number % 10;
		number /= 10;
		str[k] = (digit | 0x30); // convert to ASCII code
	}
	str[numDigits] = NULL;
}

// Initialize time and date settings of RTC
void RTC_calendarInit (Calendar CalendarTime)
{
    RTCCTL1 |= RTCMODE_H + RTCHOLD_H; 	// Calendar mode, hold counter

    RTCCTL1 &= ~(RTCBCD); 	// Hexadecimal mode (not BCD)

    RTCSEC = CalendarTime.Seconds;
    RTCMIN = CalendarTime.Minutes;
    RTCHOUR = CalendarTime.Hours;
    RTCDOW = CalendarTime.DayOfWeek;
    RTCDAY = CalendarTime.DayOfMonth;
    RTCMON = CalendarTime.Month;
    RTCYEAR = CalendarTime.Year;
}

// Get the current time from RTC
Calendar RTC_getCalendarTime(void)
{
    Calendar tempCal;

    // while ( !(HWREG(baseAddress + OFS_RTCCTL01) & RTCRDY) ) ;
    while (!(RTCRDY)); 	// Wait for RTC to be ready (not updating)

    tempCal.Seconds = RTCSEC;
    tempCal.Minutes = RTCMIN;
    tempCal.Hours = RTCHOUR;
    tempCal.DayOfWeek = RTCDOW;
    tempCal.DayOfMonth = RTCDAY;
    tempCal.Month = RTCMON;
    tempCal.Year = RTCYEAR;

    return (tempCal) ;
}

// Start the real time clock
void RTC_startClock (void)
{
	RTCCTL1 &= ~(RTCHOLD_H);
	RTCPS1CTL |= RT1PSIE; 		// enable 1-second interval interrupt (RT1PSIE = 0x0002)
}

// Stop the real time clock (for setting)
void RTC_holdClock ()
{
	RTCCTL1 |= RTCHOLD_H;
}

// Find On_Times alarm index
Alarm findNextOnAlarmIndex(int numTimes){
	int j, index;
	char type = 0;
	long int CT, onT, offT;
	Alarm nextAlarm;

	index = -1;
	currentTime = RTC_getCalendarTime();
	CT = 535680*currentTime.Year + 44640*currentTime.Month + 1440*currentTime.DayOfMonth + 60*currentTime.Hours + currentTime.Minutes;
	for(j=0;j<numTimes;j++){
		onT = 535680*on_Times[j].Year + 44640*on_Times[j].Month + 1440*on_Times[j].DayOfMonth + 60*on_Times[j].Hours + on_Times[j].Minutes;
		offT = 535680*offTimes[j].Year + 44640*offTimes[j].Month + 1440*offTimes[j].DayOfMonth + 60*offTimes[j].Hours + offTimes[j].Minutes;
		if ((onT - CT) > 0){
			index = j;
		    type = 1;
		    break;
		}
		else if ((offT - CT) > 0){
		    index = j;
		    type = 0;
		    break;
		}
	}
	nextAlarm.index = index;
	nextAlarm.type = type;
	return nextAlarm;
}

// Temporarily set RTC time
void setRTC(void){

	currentTime.Year = 2015;
	currentTime.Month = 1;
	currentTime.DayOfMonth = 1;
	currentTime.Hours = 0;
	currentTime.Minutes = 0;
	currentTime.Seconds = 0;

	RTC_calendarInit(currentTime);
	RTC_startClock();
}
// Initialize alarm times
void Init_Alarm(void){
	on_Times[0].Year = 2013;
	on_Times[0].Month = 1;
	on_Times[0].DayOfMonth = 7;
	on_Times[0].Hours = 12;
	on_Times[0].Minutes = 1;
	on_Times[0].Seconds = 1;
	on_Times[1].Year = 2013;
	on_Times[1].Month = 1;
	on_Times[1].DayOfMonth = 7;
	on_Times[1].Hours = 12;
	on_Times[1].Minutes = 3;
	on_Times[1].Seconds = 1;
	on_Times[2].Year = 2013;
	on_Times[2].Month = 1;
	on_Times[2].DayOfMonth = 7;
	on_Times[2].Hours = 12;
	on_Times[2].Minutes = 5;
	on_Times[2].Seconds = 1;

	offTimes[0].Year = 2013;
	offTimes[0].Month = 1;
	offTimes[0].DayOfMonth = 7;
	offTimes[0].Hours = 12;
	offTimes[0].Minutes = 2;
	offTimes[0].Seconds = 1;
	offTimes[1].Year = 2013;
	offTimes[1].Month = 1;
	offTimes[1].DayOfMonth = 7;
	offTimes[1].Hours = 12;
	offTimes[1].Minutes = 4;
	offTimes[1].Seconds = 1;
	offTimes[2].Year = 2013;
	offTimes[2].Month = 1;
	offTimes[2].DayOfMonth = 7;
	offTimes[2].Hours = 12;
	offTimes[2].Minutes = 6;
	offTimes[2].Seconds = 1;

	numAlarms = 3;
}

// Set alarm
void setAlarm(Alarm theNextAlarm){
	if(theNextAlarm.index != -1){
		RTCCTL0 &= ~RTCAIE; 	// disable alarm interrupt
		RTCCTL0 &= ~RTCAIFG; 	// clear alarm interrupt flag
		RTCAMIN = 0x00; 		// disable each alarm (and clear alarm times)
		RTCAHOUR = 0x00;
		RTCADAY = 0x00;
		RTCADOW = 0x00;
		if(theNextAlarm.type == 1){ // Lights on alarm type
			RTCAMIN = (on_Times[nextAlarm.index].Minutes)|0x80; 	// ORed with 0x80 to enable alarm
			RTCAHOUR = (on_Times[nextAlarm.index].Hours)|0x80;
			RTCADAY = (on_Times[nextAlarm.index].DayOfMonth)|0x80;
		}
		else{
			RTCAMIN = (offTimes[nextAlarm.index].Minutes)|0x80; 	// ORed with 0x80 to enable alarm
			RTCAHOUR = (offTimes[nextAlarm.index].Hours)|0x80;
			RTCADAY = (offTimes[nextAlarm.index].DayOfMonth)|0x80;
		}
		RTCCTL0 |= RTCAIE; 	// enable alarm interrupt
	}
}

// Battery MOSFET switch
void batterySwitch(void){
	unsigned int i,j;

	batteryEventFlag = 0;
	for(j=0;j<10;j++){
		for(i=0;i<65535;i++); 	//delay approximately 150 ms (for 4 MHz clock, 1 cycle per loop iteration)
	}
	if (P1IN & 0x04){ 			// battery disconnected
		P1OUT &= ~0x02; 		// Switch off battery MOSFET (set P1.1 low)
		batteryFlag = 0;
		P1IES |= 0x04; 		// P1.2 High/Low edge  Set interrupt for attachment of battery
	}
	else{ 						// battery present
		P1OUT |= 0x02; 			// Switch on battery MOSFET (set P1.1 high)
		batteryFlag = 1;
		P1IES &= ~0x04; 		// P1.2 Low/High edge  Set interrupt for removal of battery
	}
}

// Battery MOSFET switch to save current when idle (outputFlag = 0)
void batterySwitchIdle(int state){
	if(state == 1){
		P1OUT |= 0x02; 			// Switch on battery MOSFET (set P1.1 high)
	}
	else{
		P1OUT &= ~0x02; 		// Switch off battery MOSFET (set P1.1 low)
	}
}

#pragma vector=RTC_VECTOR
__interrupt void RTC_ISR (void)
{
	unsigned int i,j;
    switch (__even_in_range(RTCIV,16)){
        case 0: break;  //No interrupts
        case 2: break;  //RTCRDYIFG
        case 4:         //RTCEVIFG
            //Interrupts every minute
            __no_operation();
            break;
        case 6:         //RTCAIFG
        	// Commented out case 6 so that mask continues to flash regardless of alarms
        	//if(armed){
        		//if(nextAlarm.type == 1){ 	// if lights-on alarm
        			//nextAlarm.type = 0; 	// next alarm lights off
        			//setAlarm(nextAlarm);
        			//outputFlag = 1;
        			//flashFlag = 1;
        			//batterySwitchIdle(1); 	// Switch on battery and wait for DACs to initialize
        			for(j=0;j<10;j++){
        					for(i=0;i<65535;i++); 	//delay approximately 150 ms (for 4 MHz clock, 1 cycle per loop iteration)
        			}
        			//taStart(pulseRep); 		// pulse repetition period in seconds

        		//}
        		//else{
        			//nextAlarm.type = 1; 	// next alarm lights on
        			//nextAlarm.index++;
        			//setAlarm(nextAlarm);
        			//outputFlag = 0;
        			//flashFlag = 0;

        		//}
        	//}
        	//
            __no_operation();
            break;
        case 8: break;  //RT0PSIFG
        case 10: 		//RT1PSIFG
        	wdReset_16000(); 		// Reset watchdog timer for 16 second interval
        	RTCPS1CTL &= ~RT1PSIFG; 	// Clear interrupt flag
        	break;
        case 12: break; //Reserved
        case 14: break; //Reserved
        case 16: break; //Reserved
        default: break;
    }
}

// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
	P1IE &= ~0x04;         	// P1.2 interrupt disabled (don't want to catch multiple bounces)
	P1IFG &= ~0x04;         // P1.4 IFG cleared
	batteryEventFlag = 1;
	LPM3_EXIT;
}

// Make LEDs flash regardless of alarm status
void startFlashingRegardlessAlarm(void){
	unsigned int i,j;
	outputFlag = 1;
	flashFlag = 0; 	// flashFlag is set to 1 when timerA count==1. If set to 1 here the first pulse is 1 s too long.
	batterySwitchIdle(1); 	// Switch on battery and wait for DACs to initialize
	for(j=0;j<10;j++){
	   for(i=0;i<65535;i++); 	//delay approximately 150 ms (for 4 MHz clock, 1 cycle per loop iteration)
	}
	taStart(pulseRep); 		// pulse repetition period in seconds
}
/*----------------------------------------------------------------------------+
| End of source file                                                          |
+----------------------------------------------------------------------------*/
/*------------------------ Nothing Below This Line --------------------------*/

