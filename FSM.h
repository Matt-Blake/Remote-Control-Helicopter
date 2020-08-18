/* ****************************************************************
 * FSM.c - Helicopter finite state machine
 *
 * Tue am Group 1
 * Creators: Brendain Hennessy   57190084
 *           Sarah Kennelly      76389950
 *           Matt Blake          58979250
 * Last modified: 9/05/2019
 *
 * pidController.c - This code was based off the FSM.c code from ENEL361.
 * It has been edited to include FreeRTOS functionality and has two extra
 * modes allowing the helicopter to reach the mid-point altitude and turn
 * 180 degrees.
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 10/08/2020
 * ***************************************************************/

//******************************************************
// Includes
//******************************************************
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "uart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"
#include "timers.h"
#include "pwm.h"

//******************************************************
// Constants
//******************************************************
#define ALT_TOLERANCE           2       // The tolerance in altitude value to trigger state change
#define YAW_TOLERANCE           2       // The tolerance in yaw value to trigger state change
#define LAND_TMR_PERIOD         200

//******************************************************
// Globals
//******************************************************


/*extern TaskHandle_t MainPWM;
extern TaskHandle_t TailPWM;
extern TaskHandle_t BtnCheck;
extern TaskHandle_t SwitchCheck;
extern TaskHandle_t FSMTask;*/

TaskHandle_t Blinky;
TaskHandle_t OLEDDisp;
TaskHandle_t BtnCheck;
TaskHandle_t SwitchCheck;
TaskHandle_t ADCTrig;
TaskHandle_t ADCMean;
TaskHandle_t MainPWM;
TaskHandle_t TailPWM;
TaskHandle_t FSMTask;
TaskHandle_t Blinky;

extern QueueHandle_t xFSMQueue;
extern QueueHandle_t xAltMesQueue;
extern QueueHandle_t xAltDesQueue;

extern QueueHandle_t xMainPWMQueue; // Delete this once debugging done

extern TimerHandle_t xLandingTimer;

extern controller_t g_alt_controller;
extern controller_t g_yaw_controller;


// CREATE DESCRIPTION
void GetStackUsage(void);

// CREATE DESCRIPTION
void vLandTimerCallback( TimerHandle_t xTimer );


/*
 * Function:    FSM
 * ------------------------
 * FreeRTOS task that periodically checks the current state of the
 * helicopter and runs the appropriate function.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void FSM(void *pvParameters);

