//This handle the watchdog, timer A (add later), and timer B

#ifndef TIMERS_H_
#define TIMERS_H_

extern char timerAevent;
extern char USBtiming;
extern unsigned int pulseRep;
extern unsigned int pulseDur;

void tbDelay(float milliseconds);
void tbDelayMode(float milliseconds, int sleepMode);
void wdReset_250();
void wdReset_1000();
void wdReset_16000();
void wdOff();
void taStart(unsigned int pulseRep);
void taStop();
void tbStart(float pulseDur);

#endif /* TIMERS_H_ */
