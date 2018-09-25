/**
  ******************************************************************************
  * @file           : power.c
  * @brief          : Library for power control
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

/* Includes ------------------------------------------------------------------*/
#include "power.h"
#include "led.h"
#include "dcf77.h"


/* Private variables ---------------------------------------------------------*/


void power_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

  // GPIO Ports Clock Enable -------------------------------------------------
  __HAL_RCC_GPIOA_CLK_ENABLE();


  // Init Pin to control LED/Thermalprinter Power Supply ------------------------------------
  GPIO_InitStruct.Pin = TP_POWER_PA12 | MOT_PWR_PA4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void tp_power_on(void)
{
	HAL_GPIO_WritePin(GPIOA, TP_POWER_PA12, GPIO_PIN_SET);
}

void tp_power_off(void)
{
	HAL_GPIO_WritePin(GPIOA, TP_POWER_PA12, GPIO_PIN_RESET);
}

void mot_power_on(void)
{
	HAL_GPIO_WritePin(GPIOA, MOT_PWR_PA4, GPIO_PIN_SET);
}

void mot_power_off(void)
{
	HAL_GPIO_WritePin(GPIOA, MOT_PWR_PA4, GPIO_PIN_RESET);
}

void power_deact_all_periph_for_sleep(void)
{
	mot_power_off();
	led_disable();
	tp_power_off();
	dcf77_deinit();

	HAL_GPIO_WritePin(GPIOA, STROBE2_PA8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LATCH_PA9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, STROBE1_PB1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, DCF77_PON_PB5, GPIO_PIN_SET);

	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_TIM2_CLK_DISABLE();
	__HAL_RCC_ADC1_CLK_DISABLE();
	__HAL_RCC_DMA1_CLK_DISABLE();

	/*GPIO_InitTypeDef GPIO_InitStructure; TODO future maybe set all unused pins in analog mode --> saves power

	// Enable GPIOs clock
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	// Configure all GPIO port pins in Analog Input mode (floating input trigger OFF)
	GPIO_InitStructure.Pin = GPIO_PIN_All;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure); */



}
  
  
  
  
  
  
  
