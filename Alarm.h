#ifndef __ALARM_H
#define __ALARM_H

#define BKP_VALUE 0x32F
#define MAX_PWM ((1<<16) - 1)

void Clock_Init( void );
static void RTC_Config( void );
void RTC_TimeRegulate( void );
void RTC_TimeShow( void );
void RTC_AlarmShow( void );
void WakeUpSequence( viod );

#endif // __ALARM_H
