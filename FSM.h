/* ****************************************************************
 * FSM.h
 *
 * Source file for the finite state machine (FSM) module
 * Control the tasks operated based on if the helicopter is trying
 * to take off, hover, land or is landed.
 *
 * Based on FSM.h
 * Tue AM Group 1
 * Creators: Brendain Hennessy   57190084
 *           Sarah Kennelly      76389950
 *           Matt Blake          58979250
 * Last modified: 9/05/2019
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 19/08/2020
 *
 * ***************************************************************/

#ifndef FSM_H
#define FSM_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"
#include "timers.h"
#include "pwm.h"
#include "uart.h"
#include "FreeRTOSCreate.h"

#define ALT_TOLERANCE           2       // The tolerance in altitude value to trigger state change
#define YAW_TOLERANCE           2       // The tolerance in yaw value to trigger state change
#define FIND_REF_PWM_MAIN       15      // The main rotor PWM used to find the reference yaw
#define FIND_REF_PWM_TAIL       0       // The tail rotor PWM used to find the reference yaw
#define TAKEOFF_ALT             15      // The desired altitude during the takeoff sequence
#define LANDING_ALT             30      // The inital desired altitude during the landing sequence
#define LAND_TMR_PERIOD         300
#define UART_MESSAGE_SIZE       17      // The number of chars that will be transmitted over UART


/*
 * Function:    vLandTimerCallback
 * --------------------------------
 * Callback function for the timer started during the
 * landing sequence.
 * Increases timer ID each time function is called.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
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
