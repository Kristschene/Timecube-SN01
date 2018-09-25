//--------------------------------------------------------------
// File     : stm32_ub_dcf77.h
//--------------------------------------------------------------

//--------------------------------------------------------------
#ifndef DCF77_H
#define DCF77_H


//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32l0xx_hal.h"
#include "io_def.h"



//--------------------------------------------------------------
// define vom benutzten DCF77-Signal-Pin
// [PA0, PB0, PC0, PD0, PE0, PF0, PG0, PH0, PI0]
//--------------------------------------------------------------
#define  DCF77_PIN       DCF77_DATA_PB4
#define  DCF77_PORT      GPIOB

//--------------------------------------------------------------
// DCF77 time defines (ms)
// lo [soll=100ms, ist=138...150ms]
// hi [soll=200ms, ist=244...260ms]
//--------------------------------------------------------------
#define  DCF77_TIMOUT    2500
#define  DCF77_SYNC      1500
#define  DCF77_LO_MIN    75
#define  DCF77_LO_MAX    174
#define  DCF77_HI_MIN    175
#define  DCF77_HI_MAX    280

//--------------------------------------------------------------
// enum vom DCF77-Status
//--------------------------------------------------------------
typedef enum {
  DCF77_NO_SIGNAL =0, // there is no DCF77-Signal
  DCF77_WAIT_SYNC,	  	  //
  DCF77_READING,      // wait until time-data is complete
  DCF77_TIME_ERROR,   // error
  DCF77_TIME_OK       // time-data is ok
}DCF77_Status_t;

//--------------------------------------------------------------
// interne Struktur
//--------------------------------------------------------------
typedef enum {
  M77_Init = 0,     // init
  M77_Wait4Signal,  // waiting for DCF77-Signal
  M77_Wait4Sync,    // waiting for Sync Puls
  M77_SyncOk,       // Sync Puls ok
  M77_Wait4Time,    // waiting until Time is ready
  M77_Ready,        // Time is ready
  M77_Error,        // Error in DCF77-Signal
}DCF77_Mode_t;

typedef struct {
  DCF77_Mode_t mode;
  uint8_t ok;    // flag
  uint8_t min;   // minuten
  uint8_t std;   // stunden
  uint8_t tag;   // tag
  uint8_t monat; // monat
  uint8_t jahr; // jahr
}DCF77_interal_t;
//--------------------------------------------------------------

//--------------------------------------------------------------
// globale DCF77-Struktur
//--------------------------------------------------------------
typedef struct {
  uint8_t sek;   // sekunden [0..59]
  uint8_t min;   // minuten [0..59]
  uint8_t std;   // stunden [0..23]
  uint8_t tag;   // tag [1..31]
  uint8_t monat; // monat [1..12]
  uint8_t jahr;  // jahr [0..99]
}DCF77_TIME_t;
DCF77_TIME_t DCF77_TIME;


//--------------------------------------------------------------
// Globale Funktionen
//--------------------------------------------------------------
void dcf77_init(void);
DCF77_Status_t dcf77_read_time(void);
void dcf77_deinit(void);


//--------------------------------------------------------------
#endif // DCF77_H
