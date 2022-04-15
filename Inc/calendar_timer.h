/*
 * calendar_timer.h
 *
 *  Created on: 30 џэт. 2021 у.
 *      Author: Prog
 */

#ifndef INC_CALENDAR_TIMER_H_
#define INC_CALENDAR_TIMER_H_

#include "main.h"
 #define SECOND_A_DAY 86400
//uint8_t time_mas[]={'0','1','2','3','4','5','6','7','8','9'};
typedef struct
 {
   int year;
   char mon;
   char mday;
   char hour;
   char min;
   char sec;
   char wday;
 } unixColodar;

unixColodar unixTime;

void counterToColodar (unsigned long counter, unixColodar * unixTime);
unsigned long colodarToCounter (unixColodar * unixTime);
 void mRTCInit(void);
 uint32_t mRTCGetCounter(void) ;
 void mRTCSetCounter(uint32_t count);



#endif /* INC_CALENDAR_TIMER_H_ */
