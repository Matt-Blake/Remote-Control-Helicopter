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
#define FIND_REF_PWM_MAIN       15      // The main rotor PWM used to find the reference yaw
#define FIND_REF_PWM_TAIL       0       // The tail rotor PWM used to find the reference yaw
#define TAKEOFF_ALT             15      // The desired altitude during the takeoff sequence
#define LANDING_ALT             30      // The inital desired altitude during the landing sequence
#define LAND_TMR_PERIOD         300
#define FSM_PERIOD              200
#define UART_MESSAGE_SIZE       17      // The number of chars that will be transmitted over UART

extern controller_t g_alt_controller;
extern controller_t g_yaw_controller;

extern QueueHandle_t xFSMQueue;
extern QueueHandle_t xAltMesQueue;
extern QueueHandle_t xAltDesQueue;
extern QueueHandle_t xMainPWMQueue;     // Delete this once debugging done

extern TaskHandle_t FSMTask;
extern TaskHandle_t OLEDDisp;
extern TaskHandle_t UARTDisp;
extern TaskHandle_t StatLED;
extern TaskHandle_t BtnCheck;
extern TaskHandle_t SwiCheck;
extern TaskHandle_t ADCTrig;
extern TaskHandle_t ADCMean;
extern TaskHandle_t MainPWM;
extern TaskHandle_t TailPWM;

extern TimerHandle_t xLandingTimer;


/*
 * Function:    GetStackUsage
 * ---------------------------
 * Function that calculates and transmits over UART
 * statistics about the stack usage.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void GetStackUsage(void);

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
 * Function:    findYawRef
 * ------------------------
 * Disables the PWM control, buttons, and switches.
 * Sets the main PWM to be 50% duty cycle in order for the
 * helicopter to spin.
 * Once the yaw reference flag has been set by an interrupt,
 * all tasks resume.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void findYawRef(void);

/*
 * Function:    takeoff
 * ---------------------
 * If the reference has not be found, the findYawRef function is
 * called.
 * If the reference flag has been set, the helicopter ascends to
 * 20% height, and rotates to 0 degrees yaw.
 * Once this position has been reached, the state changes to FLYING.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void takeoff(void);

/*
 * Function:    hover
 * -------------------
 * Basic flying mode. All tasks are functional.
 * Movement is controlled by the GPIO buttons and the PID
 * controller.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void hover(void);

/*
 * Function:    land
 * ------------------
 * Function that sets the desired altitude to 10%, and then
 * decreases this value by 2% every second.
 * Once the desired position is reached, the state is changed to
 * LANDED.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void land(void);

/*
 * Function:    landed
 * --------------------
 * Disables all input with the exception of the switches.
 * Helicopter is in an idle state.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void landed(void);

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
