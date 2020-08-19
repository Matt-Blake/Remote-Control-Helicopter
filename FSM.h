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

#ifndef FSM_H
#define FSM_H

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "pwm.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"
#include "timers.h"
#include "uart.h"
#include "buttons.h"

#define ALT_TOLERANCE           2       // The tolerance in altitude value to trigger state change
#define YAW_TOLERANCE           2       // The tolerance in yaw value to trigger state change
#define FIND_REF_PWM_MAIN       18      // The main rotor PWM used to find the reference yaw
#define FIND_REF_PWM_TAIL       0       // The tail rotor PWM used to find the reference yaw
#define LAND_TMR_PERIOD         300
#define FSM_PERIOD              200

extern controller_t g_alt_controller;
extern controller_t g_yaw_controller;

QueueHandle_t xFSMQueue;
extern QueueHandle_t xAltMesQueue;
extern QueueHandle_t xAltDesQueue;
extern QueueHandle_t xMainPWMQueue;     // Delete this once debugging done

TaskHandle_t FSMTask;
TaskHandle_t OLEDDisp;
TaskHandle_t StatLED;
TaskHandle_t BtnCheck;
TaskHandle_t SwiCheck;
TaskHandle_t ADCTrig;
TaskHandle_t ADCMean;
TaskHandle_t MainPWM;
TaskHandle_t TailPWM;

TimerHandle_t xLandingTimer;


// CREATE DESCRIPTION
//void GetStackUsage(void);

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

#endif /*FSM_H*/
