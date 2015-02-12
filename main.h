/*
 * main.h
 *
 *  Created on: Jan 9, 2013
 *      Author: bierma2
 */

#ifndef MAIN_H_
#define MAIN_H_

typedef struct {
    unsigned char Seconds;
    unsigned char Minutes;
    unsigned char Hours;
    unsigned char DayOfWeek;
    unsigned char DayOfMonth;
    unsigned char Month;
    unsigned int Year;
} Calendar;

typedef struct {
	int index;
	char type;
} Alarm;

// Function Prototypes
void Init_StartUp();               // Initialize clocks, power, and I/Os
unsigned char retInString(char* string);
int Ascii2Num(char* string);
void RTC_calendarInit (Calendar CalendarTime);
Calendar RTC_getCalendarTime (void);
void RTC_startClock (void);
void RTC_holdClock (void);
void Init_Alarm(void);
void Init_Alarm(void);
Alarm findNextOnAlarmIndex(int numTimes);
void setAlarm (Alarm theNextAlarm);
void setRTC(void);
void num2ASCII(unsigned int number,char* str,int numDigits);
void batterySwitch(void);
void batterySwitchIdle(int state);
void startFlashingRegardlessAlarm(void);


extern unsigned int pulseRep; 	// units of seconds
extern unsigned int pulseDur; 			// units of milliseconds
extern unsigned int pulseInt; 	// units of DAC counts (0 to 4095)
extern volatile Calendar on_Times[7],offTimes[7];
extern int numAlarms;
extern volatile Calendar currentTime;
extern int flashFlag,outputFlag;

#endif /* MAIN_H_ */
