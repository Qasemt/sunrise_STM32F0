#ifndef __ALARM_H
#define __ALARM_H

#define BKP_VALUE 0x32F

void Clock_Init( void );
static void RTC_Config( void );
void RTC_TimeRegulate( void );
void RTC_TimeShow( void );
void RTC_AlarmShow( void );
void WakeUpSequence( viod );

#endif // __ALARM_H
