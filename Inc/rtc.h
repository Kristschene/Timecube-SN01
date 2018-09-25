
#include "stm32l0xx_hal.h"


void rtc_init(void);
void rtc_set_time_date(RTC_TimeTypeDef* sTime, RTC_DateTypeDef* sDate);
RTC_TimeTypeDef rtc_get_time(void);
RTC_DateTypeDef rtc_get_date(void);


