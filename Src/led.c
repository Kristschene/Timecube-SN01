/**
  ******************************************************************************
  * @file           : led.c
  * @brief          : Library for LED control
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
#include "led.h"


/* Private variables ---------------------------------------------------------*/
static  TIM_HandleTypeDef htim2;
static char led_active = 0;
static enum eLedColor eColor;
static enum eLedType eType;

void led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

  // GPIO Ports Clock Enable -------------------------------------------------
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // TIM 2 -------------------------------------------------------------------
  __HAL_RCC_TIM2_CLK_ENABLE();

  // PWM Timer init  ---------------------------------------------------------
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = PERIOD;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

	 /**TIM2 GPIO Configuration
	 PA15     ------> TIM2_CH1
	 PB3     ------> TIM2_CH2
	 */
	 GPIO_InitStruct.Pin = LED_RED_PA15;
	 GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	 GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	 GPIO_InitStruct.Alternate = GPIO_AF5_TIM2;
	 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	 GPIO_InitStruct.Pin = LED_GREEN_PB3;
	 GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	 GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	 GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
	 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


void led_start()
{
	led_stop();
	if(eColor==eRed||eColor==eOrange)
	{
		if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
		{
		  /* Starting Error */
			_Error_Handler(__FILE__, __LINE__);
		}
	}

	if(eColor==eGreen||eColor==eOrange)
	{
		if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
		{
		  /* Starting Error */
		_Error_Handler(__FILE__, __LINE__);
		}
	}
}

void led_stop()
{
	if(HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1) != HAL_OK)
	{
	  /* Starting Error */
		_Error_Handler(__FILE__, __LINE__);
	}

	if(HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2) != HAL_OK)
	{
	  /* Starting Error */
	_Error_Handler(__FILE__, __LINE__);
	}
}

void led_change_color(uint8_t red_percent, uint8_t green_percent)
{
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	float period = PERIOD;

	if(eColor==eRed||eColor==eOrange)
	{
		// LED RED
		sConfigOC.Pulse = (period * (float) red_percent) / 100.0;
		if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}
	}

	if(eColor==eGreen||eColor==eOrange)
	{
		// LED GREEN
		sConfigOC.Pulse = (period * (float) green_percent) / 100.0;
		if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}
	}

	led_start();
}

/*void led_fade_in(TIM_HandleTypeDef * htim, uint16_t fade_in_time_ms, uint8_t red_percent, uint8_t green_percent)
{
	float red_percent_per_ms = (float) red_percent / (float) fade_in_time_ms;
	float green_percent_per_ms = (float) green_percent / (float) fade_in_time_ms;
	float red_percent_actual = 0;
	float green_percent_actual = 0;


	for( int i = 0; i<=fade_in_time_ms; i++)
	{
		red_percent_actual += red_percent_per_ms;
		green_percent_actual += green_percent_per_ms;

		led_change_color(htim, (uint8_t) red_percent_actual, (uint8_t) green_percent_actual);
		HAL_Delay(1); // 1ms steps
	}
} */
void led_enable(enum eLedType eLType, enum eLedColor eLColor)
{
	led_active = 1;
	eType = eLType;
	eColor = eLColor;

	// Led steady operated
	if(eType == eSteady)
	{
		if(eColor==eGreen) led_change_color(0, 100);
		else if(eColor==eRed) led_change_color(100, 0);
		else if(eColor==eOrange) led_change_color(100, 100);
	}
}

void led_disable(void)
{
	led_active = 0;
	led_stop();
}

// will be called every 1 ms
void LED_ProcessIT(void)
{
	static int i=0;
	static int cnt=0;
	static char flag_10ms;
	static char direction=1;

	if(led_active && eType == eFlash)
	{
		cnt++;
		if(cnt>=15) // changed to 15ms flags
		{
			flag_10ms = 1;
			cnt = 0;
		}
		else
		{
			flag_10ms = 0;
		}

		if(flag_10ms && led_active)
		{
			if(direction==1)
			{
				if(i<100) i++;
				else
				{
					i--;
					direction=0;
				}
			}
			else
			{
				if(i>0) i--;
				else
				{
					i++;
					direction=1;
				}
			}

			if(eColor==eGreen) led_change_color(0, i);
			else if(eColor==eRed) led_change_color(i, 0);
			else if(eColor==eOrange) led_change_color(i, i);
		}
	}
	else
	{
		i=0;
		cnt=0;
		direction=0;
		flag_10ms=0;
	}
}


