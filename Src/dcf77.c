//--------------------------------------------------------------
// File     : stm32_ub_dcf77.c
// Datum    : 31.12.2015
// Version  : 1.0
// Autor    : UB
// EMail    : mc-4u(@)t-online.de
// Web      : www.mikrocontroller-4u.de
// CPU      : STM32F4
// IDE      : CooCox CoIDE 1.7.8
// GCC      : 4.9 2015q2
// Module   : GPIO, TIM, EXTI, SYSCFG, MISC
// Funktion : auswertung vom DCF-77 Signal an PB0
//
// Hinweis  : Das DCF-77 Signal wird per ExtInterrupt-0
//            ausgewertet, muss also an PA0, PB0, PC0...PI0 liegen
//
//            Timer2 wird zur Zeitmessung benutzt
//--------------------------------------------------------------


//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "dcf77.h"



//--------------------------------------------------------------
// interne Funktionen
//--------------------------------------------------------------
void P_init_TIM(void);
void P_init_EXTI(void);
void P_init_PON(void);
void P_Store_Value(uint8_t value, uint8_t pos);


//--------------------------------------------------------------
// interne Globale Variabeln
//--------------------------------------------------------------
uint32_t dcf77_1ms_value=0;
uint32_t dcf77_ms_counter=0;
DCF77_interal_t dcf77_int;
TIM_HandleTypeDef htim21;


//--------------------------------------------------------------
// init vom DCF-77 Modul
//--------------------------------------------------------------
void dcf77_init(void)
{
  // init vom Timer
  P_init_TIM();

  // init vom Exti-Pin
  P_init_EXTI();

  // init P_on-pin
  P_init_PON();

  dcf77_1ms_value=0;
  dcf77_ms_counter=0;

  dcf77_int.mode=M77_Init;
  dcf77_int.ok=0;
  dcf77_int.min=0;
  dcf77_int.std=0;
  dcf77_int.tag=0;
  dcf77_int.monat=0;
  dcf77_int.jahr=0;

  DCF77_TIME.sek=0;
  DCF77_TIME.min=0;
  DCF77_TIME.std=0;
  DCF77_TIME.tag=0;
  DCF77_TIME.monat=0;
  DCF77_TIME.jahr=0;

  // Timer enable
  HAL_TIM_Base_Start_IT(&htim21);
}

void dcf77_deinit(void)
{
	HAL_TIM_Base_MspDeInit(&htim21);
	HAL_GPIO_WritePin(GPIOB, DCF77_PON_PB5, GPIO_PIN_SET);
}


//--------------------------------------------------------------
// aktuelle Zeit vom DCF77-Modul auslesen
// ret_wert :
//   DCF77_NO_SIGNAL  = es wird kein Signal empfangen
//   DCF77_READING    = signal wird empfangen, warte auf uhrzeit
//   DCF77_TIME_ERROR = fehler beim empfang
//   DCF77_TIME_OK    = ok, Uhrzeit steht in "DCF77_TIME"
//--------------------------------------------------------------
DCF77_Status_t dcf77_read_time(void)
{
  DCF77_Status_t ret_wert=DCF77_NO_SIGNAL;

  if(dcf77_int.ok==1) {
    ret_wert=DCF77_TIME_OK;
  }
  else {
    if((dcf77_int.mode==M77_Init) || (dcf77_int.mode==M77_Wait4Signal)) {
      ret_wert=DCF77_NO_SIGNAL;
    }
    else if(dcf77_int.mode==M77_Error) {
      ret_wert=DCF77_TIME_ERROR;
    }
    else if(dcf77_int.mode==M77_Wait4Sync) {
      ret_wert=DCF77_WAIT_SYNC;
    }
    else {
      ret_wert=DCF77_READING;
    }
  }

  return(ret_wert);
}

//--------------------------------------------------------------
// interne Funktion zum init vom Timer
//--------------------------------------------------------------
void P_init_TIM(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;

	htim21.Instance = TIM21;
	htim21.Init.Prescaler = 0;
	htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim21.Init.Period = 31999;
	htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
	{
	  _Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}
}

//--------------------------------------------------------------
// interne Funktion zum init vom Ext-Interrupt
//--------------------------------------------------------------
void P_init_EXTI(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin : PB4 */
	GPIO_InitStruct.Pin = DCF77_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(DCF77_PORT, &GPIO_InitStruct);


	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}


void P_init_PON(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin : PB4 */
	GPIO_InitStruct.Pin = DCF77_PON_PB5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, DCF77_PON_PB5, GPIO_PIN_SET);

	HAL_Delay(200);

	HAL_GPIO_WritePin(GPIOB, DCF77_PON_PB5, GPIO_PIN_RESET);
}

//--------------------------------------------------------------
// bearbeite ein bit der uhrzeit
//--------------------------------------------------------------
void P_Store_Value(uint8_t value, uint8_t pos)
{
  static uint8_t bytewert=0;

  switch(pos) {
    // minuten
    case 21 : if(value==1) bytewert+=1;break;
    case 22 : if(value==1) bytewert+=2;break;
    case 23 : if(value==1) bytewert+=4;break;
    case 24 : if(value==1) bytewert+=8;break;
    case 25 : if(value==1) bytewert+=10;break;
    case 26 : if(value==1) bytewert+=20;break;
    case 27 :
      if(value==1) bytewert+=40;
      dcf77_int.min=bytewert;
      bytewert=0;
      break;
    // stunden
    case 29 : if(value==1) bytewert+=1;break;
    case 30 : if(value==1) bytewert+=2;break;
    case 31 : if(value==1) bytewert+=4;break;
    case 32 : if(value==1) bytewert+=8;break;
    case 33 : if(value==1) bytewert+=10;break;
    case 34 :
      if(value==1) bytewert+=20;
      dcf77_int.std=bytewert;
      bytewert=0;
      break;
    // tag
    case 36 : if(value==1) bytewert+=1;break;
    case 37 : if(value==1) bytewert+=2;break;
    case 38 : if(value==1) bytewert+=4;break;
    case 39 : if(value==1) bytewert+=8;break;
    case 40 : if(value==1) bytewert+=10;break;
    case 41 :
      if(value==1) bytewert+=20;
      dcf77_int.tag=bytewert;
      bytewert=0;
      break;
    // monat
    case 45 : if(value==1) bytewert+=1;break;
    case 46 : if(value==1) bytewert+=2;break;
    case 47 : if(value==1) bytewert+=4;break;
    case 48 : if(value==1) bytewert+=8;break;
    case 49 :
      if(value==1) bytewert+=10;
      dcf77_int.monat=bytewert;
      bytewert=0;
      break;
    // jahr
    case 50 : if(value==1) bytewert+=1;break;
    case 51 : if(value==1) bytewert+=2;break;
    case 52 : if(value==1) bytewert+=4;break;
    case 53 : if(value==1) bytewert+=8;break;
    case 54 : if(value==1) bytewert+=10;break;
    case 55 : if(value==1) bytewert+=20;break;
    case 56 : if(value==1) bytewert+=40;break;
    case 57 :
      if(value==1) bytewert+=80;
      dcf77_int.jahr=bytewert;
      bytewert=0;
      break;
    // ende
    case 58 :
      bytewert=0;
      dcf77_ms_counter=0; // syncronisiere sek_counter
      DCF77_TIME.sek=0;
      DCF77_TIME.min=dcf77_int.min;
      DCF77_TIME.std=dcf77_int.std;
      DCF77_TIME.tag=dcf77_int.tag;
      DCF77_TIME.monat=dcf77_int.monat;
      DCF77_TIME.jahr=dcf77_int.jahr;
      dcf77_int.mode=M77_Ready;
      dcf77_int.ok=1; // set flag
      break;
  }
}

//--------------------------------------------------------------
// ISR (Timer-21) [1ms]
//--------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    dcf77_ms_counter++;
    if(dcf77_ms_counter>=1000) {
      dcf77_ms_counter=0;
      if(DCF77_TIME.sek<59) DCF77_TIME.sek++;
    }

    dcf77_1ms_value++;
    if(dcf77_int.mode==M77_Init) {
      dcf77_1ms_value=0;
      dcf77_int.mode=M77_Wait4Signal;
    }
    else if(dcf77_int.mode==M77_Wait4Signal) {
      // nothing to do
    }
    else if(dcf77_int.mode==M77_Wait4Sync) {
      if(dcf77_1ms_value>DCF77_SYNC) {
        dcf77_1ms_value=0;
        dcf77_int.mode=M77_SyncOk;
      }
    }
    else if(dcf77_int.mode==M77_SyncOk) {
      if(dcf77_1ms_value>DCF77_TIMOUT) dcf77_int.mode=M77_Error;
    }
    else if(dcf77_int.mode==M77_Wait4Time) {
      if(dcf77_1ms_value>DCF77_TIMOUT) dcf77_int.mode=M77_Error;
    }
    else if(dcf77_int.mode==M77_Ready) {
      if(dcf77_1ms_value>DCF77_TIMOUT) dcf77_int.mode=M77_Error;
    }
    else if(dcf77_int.mode==M77_Error) {
      dcf77_int.ok=0;
    }
}


//--------------------------------------------------------------
// ISR (Exti-0) [Lo+Hi-Flanke]
//--------------------------------------------------------------
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin)
{
  uint32_t wert, signal_value=0;
  static uint8_t pos=0;

    wert=HAL_GPIO_ReadPin(DCF77_PORT, DCF77_PIN);
    if(wert==GPIO_PIN_RESET) {
      // hi Flanke
      signal_value=dcf77_1ms_value;
      if(dcf77_int.mode==M77_Wait4Time) {
        if((signal_value>=DCF77_LO_MIN) && (signal_value<=DCF77_LO_MAX)) {
          P_Store_Value(0,pos);
          pos++;
          if(dcf77_int.mode==M77_Ready) dcf77_int.mode=M77_Wait4Sync;
        }
        else if((signal_value>=DCF77_HI_MIN) && (signal_value<=DCF77_HI_MAX)) {
          P_Store_Value(1,pos);
          pos++;
          if(dcf77_int.mode==M77_Ready) dcf77_int.mode=M77_Wait4Sync;
        }
        else dcf77_int.mode=M77_Error;
      }
    }
    else {
      // lo Flanke
      dcf77_1ms_value=0;
      if(dcf77_int.mode==M77_SyncOk) {
        pos=0;
        dcf77_int.mode=M77_Wait4Time;
      }
      if((dcf77_int.mode==M77_Wait4Signal) || (dcf77_int.mode==M77_Error)) dcf77_int.mode=M77_Wait4Sync;
    }
}
