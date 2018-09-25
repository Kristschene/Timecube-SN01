/**
  ******************************************************************************
  * @file           : printer.c
  * @brief          : Library for Printer control
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
#include "printer.h"
#include "fonts.h"
#include "power.h"


/* Private variables ---------------------------------------------------------*/
void DMA_Init(void);
void ADC_Init(void);
void DMA_ADC_Init(void);


/* Private variables ---------------------------------------------------------*/
DMA_HandleTypeDef hdma_adc;
ADC_HandleTypeDef hadc;
uint16_t ADC1ConvertedValue;
char matrix_array[40][24]= {0};
char print_flag = 0;
char* pcPrinterMessage;
uint16_t adc_value_old = 3500u;


void printer_init()
{
	GPIO_InitTypeDef GPIO_InitStruct;

  // GPIO Ports Clock Enable -------------------------------------------------
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_ADC1_CLK_ENABLE();


  // Init Pins  ------------------------------------
  GPIO_InitStruct.Pin = MOT_AIN1_PA5 | MOT_AIN2_PA6 | MOT_BIN2_PA7 | CLK_PA10 | DATA_PA11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = STROBE2_PA8 |LATCH_PA9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, STROBE2_PA8, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, LATCH_PA9, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = MOT_BIN1_PB0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = STROBE1_PB1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, STROBE1_PB1, GPIO_PIN_SET);

  GPIO_InitStruct.Pin =  PAPER_SENS_PB6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // ADC
  GPIO_InitStruct.Pin = BLACK_MARK_SENS_PA2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  DMA_Init();
  ADC_Init();
  DMA_ADC_Init();
}


void printer_print_message(char* pcMessage)
{

	if(pcMessage == NULL)
	{
		print_flag = 0;
	}
	else
	{
		pcPrinterMessage = pcMessage;
		printer_prepare_message_for_print();
		print_flag = 1;
	}
}

char printer_get_printer_status(void)
{
	return print_flag;
}


void PRINTER_Print_ProcessIT(void)
{
	static int rows;
	static int columns;
	static char matrix_value;
	static int timer;
	static enum ePrinterState eState = ePrepare;

	if(print_flag!=0)
	{
		// State Machine for printing process
		switch(eState)
		{
			case ePrepare:
				rows = 0;
				columns = 0;
				matrix_value = 0;
				timer = 0;
				eState = ePrinting;
				break;

			case ePrinting:

				if(columns < 24) // load data into shift register
				{
					matrix_value = matrix_array[rows][columns];
					int shift_bit = 0x80;
					for(int bit_cnt = 0; bit_cnt < 8; bit_cnt++)
					{
						if((matrix_value & shift_bit) != 0x00)
						{
							HAL_GPIO_WritePin(GPIOA, DATA_PA11, GPIO_PIN_SET);
						}
						HAL_GPIO_WritePin(GPIOA, CLK_PA10, GPIO_PIN_SET);
						for(int i=0; i<=3; i++) // Need some time
						HAL_GPIO_WritePin(GPIOA, CLK_PA10, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOA, DATA_PA11, GPIO_PIN_RESET);
						shift_bit = shift_bit >> 1;
					}

				columns++;
				}
				else
				{
					if(rows < 40)
					{
						timer++; // increments every 1ms

						if(timer<=1) // first  activate thermal head
						{
							HAL_GPIO_WritePin(GPIOA, LATCH_PA9, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(GPIOB, STROBE1_PB1, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(GPIOA, STROBE2_PA8, GPIO_PIN_RESET);

						}
						else if(timer<=50)
						{

						}
						else if(timer <= 51) // after 40ms deactivate thermal head
						{
							HAL_GPIO_WritePin(GPIOB, STROBE1_PB1, GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOA, STROBE2_PA8, GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOA, LATCH_PA9, GPIO_PIN_SET);
						}
						else if(timer<=53)
						{
							mot_power_on();

						}
						else if(timer<=54)
						{
							printer_do_step(0u);
						}
						else if(timer<=60)
						{
						}
						else if(timer<=61)
						{
							printer_do_step(0u);
						}
						else if(timer<=67)
						{

						}
						else if(timer<=68)
						{
							//printer_motor_off();
							mot_power_off();
							rows++;
							columns = 0;
							timer=0;
						}

					}
					else
					{
						rows = 0;
						eState = eFinish;
					}

				}
				break;
			case eFinish:
				HAL_GPIO_WritePin(GPIOA, CLK_PA10, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, DATA_PA11, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, STROBE1_PB1, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, STROBE2_PA8, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, LATCH_PA9, GPIO_PIN_SET);
				print_flag = 0u;
				eState = ePrepare;
				break;

			default:
				break;
		}
	}
	/*else
	{
		HAL_GPIO_WritePin(GPIOA, CLK_PA10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, DATA_PA11, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, STROBE1_PB1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, STROBE2_PA8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, LATCH_PA9, GPIO_PIN_SET);
	} TODO */
}


void printer_do_step(char cDirection)
{

	static char cstep = 0;

	switch(cstep)
	{
		// Step 1
		case 0:
			HAL_GPIO_WritePin(GPIOA, MOT_AIN1_PA5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, MOT_AIN2_PA6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, MOT_BIN1_PB0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, MOT_BIN2_PA7, GPIO_PIN_RESET);
			if(cDirection==1) cstep=1;
			else cstep=3;
			break;

		// Step 2
		case 1:
			HAL_GPIO_WritePin(GPIOA, MOT_AIN1_PA5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, MOT_AIN2_PA6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, MOT_BIN1_PB0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, MOT_BIN2_PA7, GPIO_PIN_SET);
			if(cDirection==1) cstep=2;
			else cstep=0;
			break;

		// Step 3
		case 2:
			HAL_GPIO_WritePin(GPIOA, MOT_AIN1_PA5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, MOT_AIN2_PA6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, MOT_BIN1_PB0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, MOT_BIN2_PA7, GPIO_PIN_SET);
			if(cDirection==1) cstep=3;
			else cstep=1;
			break;

		// Step 4
		case 3:
			HAL_GPIO_WritePin(GPIOA, MOT_AIN1_PA5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, MOT_AIN2_PA6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, MOT_BIN1_PB0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, MOT_BIN2_PA7, GPIO_PIN_RESET);
			if(cDirection==1) cstep=0;
			else cstep=2;
			break;

		default:
			cstep = 0;
			break;
	}
}

void printer_motor_off(void)
{
	HAL_GPIO_WritePin(GPIOA, MOT_AIN1_PA5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MOT_AIN2_PA6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, MOT_BIN1_PB0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MOT_BIN2_PA7, GPIO_PIN_RESET);
}

void printer_prepare_message_for_print()
{
	int cnt = 0;
	int u = 0;
	unsigned char* array_pointer = NULL;
	for(int i = 0; i<12; i++) // up to 12
	{
		switch(pcPrinterMessage[i])
		{
			case ' ':
				array_pointer = array_spare;
				break;

			case '-':
				array_pointer = array_minus;
				break;

			case '0':
				array_pointer = array_zero;
				break;

			case '1':
				array_pointer = array_one;
				break;

			case '2':
				array_pointer = array_two;
				break;

			case '3':
				array_pointer = array_three;
				break;

			case '4':
				array_pointer = array_four;
				break;

			case '5':
				array_pointer = array_five;
				break;

			case '6':
				array_pointer = array_six;
				break;

			case '7':
				array_pointer = array_seven;
				break;

			case '8':
				array_pointer = array_eight;
				break;

			case '9':
				array_pointer = array_nine;
				break;
			default:
				array_pointer = array_spare;
				break;
		}
		cnt = 0;
		for(int z = 0; z<40; z++)
		{
			for(int s = u*2; s<u*2+2; s++)
			{
				matrix_array[z][s]=array_pointer[cnt];
				cnt++;
			}

		}
		u++;
	}
}


void printer_drive_to_next_label(void)
{
	uint16_t Alt = 3500; // TODO depends on battery voltage
	int i;

	printer_adc_start();
	HAL_Delay(5);

	// i >= 200 means max 200 steps when no label is found
	for(i = 0; i<= 200; i++)
	{
	  uint16_t Neu = printer_get_black_mark_adc_value();


	  //if(Neu <= 3850 && Alt > 3850 )
	  if((int16_t)(Alt - Neu) > 15)
	  {
		  // If next label is detect do 4 more steps
		  for(i=0; i<4; i++)
		  {
			  printer_do_step(0u);
		  }
		  break;
	  }

	  printer_do_step(0u);
	  Alt = Neu;
	  HAL_Delay(10);
	}
	printer_adc_stop();
	printer_motor_off();
}


GPIO_PinState printer_get_paper_status(void)
{
	return HAL_GPIO_ReadPin(GPIOB, PAPER_SENS_PB6);
}


void printer_adc_start(void)
{
	HAL_ADC_Start_DMA(&hadc, (uint32_t*) &ADC1ConvertedValue, 2u);
}


uint32_t printer_get_black_mark_adc_value(void)
{

	return ADC1ConvertedValue;
}


void printer_adc_stop(void)
{
	 HAL_ADC_Stop_DMA(&hadc);
}


///////////////////////////////////////////////////////////////////////////
// INIT etc.
///////////////////////////////////////////////////////////////////////////

void ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV10;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_3CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = ENABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


void DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void DMA_ADC_Init(void)
{

  /* ADC1 DMA Init */
  /* ADC Init */
  hdma_adc.Instance = DMA1_Channel1;
  hdma_adc.Init.Request = DMA_REQUEST_0;
  hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc.Init.Mode = DMA_CIRCULAR;
  hdma_adc.Init.Priority = DMA_PRIORITY_MEDIUM;
  if (HAL_DMA_Init(&hdma_adc) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  __HAL_LINKDMA(&hadc,DMA_Handle,hdma_adc);
}

void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

