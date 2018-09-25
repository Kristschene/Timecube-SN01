/**
  ******************************************************************************
  * @file           : io_def.h
  * @brief          : Defines port pins readable
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IO_DEF_H__
#define __IO_DEF_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"


/* Defines -------------------------------------------------------------------*/

// PORT A
// TOUCH_SENS
#define TOUCH_SENS_PA0			TSC_GROUP1_IO1

// THERM_SENS
#define THERM_SENS_PA1			GPIO_PIN_1

// BLACK_MARK_SENS
#define BLACK_MARK_SENS_PA2		GPIO_PIN_2

// TOUCH_SAMPLE
#define TOUCH_SAMPLE_PA3		TSC_GROUP1_IO4

// MOT_PWR
#define MOT_PWR_PA4				GPIO_PIN_4

// MOT_AIN1
#define MOT_AIN1_PA5			GPIO_PIN_5

// MOT_AIN2
#define MOT_AIN2_PA6			GPIO_PIN_6

// MOT_BIN2
#define MOT_BIN2_PA7			GPIO_PIN_7

// STROBE2
#define STROBE2_PA8				GPIO_PIN_8

// LATCH
#define LATCH_PA9				GPIO_PIN_9

// CLK
#define CLK_PA10				GPIO_PIN_10

// DATA
#define DATA_PA11				GPIO_PIN_11

// TP_POWER
#define TP_POWER_PA12			GPIO_PIN_12

// LED_RED
#define LED_RED_PA15			GPIO_PIN_15


// PORT B
// MOT_BIN1
#define MOT_BIN1_PB0			GPIO_PIN_0

// STROBE1
#define STROBE1_PB1				GPIO_PIN_1

// LED_GREEN
#define LED_GREEN_PB3			GPIO_PIN_3

// DCF77_DATA
#define DCF77_DATA_PB4			GPIO_PIN_4

// DCF77_PON
#define DCF77_PON_PB5			GPIO_PIN_5

// PAPER_SENS
#define PAPER_SENS_PB6			GPIO_PIN_6




#endif /* __IO_DEF_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
