/* ****************************************************************
 * BUTTONS.h
 *
 * Header file for the buttons module
 * Supports buttons on the Tiva/Orbit.
 * Comprises of initialisers and button checks
 *
 * Author: P.J. Bones UCECE
 * Edited: Derrick Edward, Grayson Mynott, Ryan Earwaker
 * Thu AM Group 18
 * Last modified:  29.05.2019
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 05/08/2020
 *
 * ***************************************************************/

#ifndef BUTTONS_H
#define BUTTONS_H


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/tm4c123gh6pm.h"  // Board specific defines (for PF0)
#include "FSM.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "uart.h"


enum btnNames   {UP = 0, DOWN, LEFT, RIGHT, NUM_BTNS};
enum btnStates  {RELEASED = 0, PUSHED, NO_CHANGE};


#define BTN_TASK_PRIORITY   5                           // Button polling task priority
#define SWI_TASK_PRIORITY   5                           // Switch polling task priority
#define BTN_STACK_DEPTH     64
#define SWITCH_STACK_DEPTH  64


extern QueueHandle_t xAltBtnQueue;
extern QueueHandle_t xYawBtnQueue;
extern QueueHandle_t xAltDesQueue;
extern QueueHandle_t xYawDesQueue;
extern QueueHandle_t xFSMQueue;
extern SemaphoreHandle_t xAltMutex;
extern SemaphoreHandle_t xYawMutex;
extern SemaphoreHandle_t xUpBtnSemaphore;
extern SemaphoreHandle_t xYawFlipSemaphore;
extern TaskHandle_t BtnCheck;
extern TaskHandle_t SwitchCheck;
extern TimerHandle_t xUpBtnTimer;
extern TimerHandle_t xYawFlipTimer;


// Debounce algorithm: A state machine is associated with each button.
// A state change occurs only after NUM_BUT_POLLS consecutive polls have
// read the pin in the opposite condition, before the state changes and
// a flag is set.  Set NUM_BUT_POLLS according to the polling rate.


void vDblBtnTimerCallback( TimerHandle_t xTimer );

/* ******************************************************
 * initButtons: Initialise the variables associated with the set of buttons
 * defined by the constants above.
 * *****************************************************/
void initBtns (void);

/* ******************************************************
 * ButtonsCheck: checks if buttons associated with altitude and yaw have
 * been pushed and increments accordingly
 * *****************************************************/
void ButtonsCheck(void *pvParameters);

/* ******************************************************
 * SwitchesCheck: checks if switches associated with helicopter's state
 * been pushed and change the state accordingly
 * *****************************************************/
void SwitchesCheck(void *pvParameters);

#endif /*BUTTONS_H*/
