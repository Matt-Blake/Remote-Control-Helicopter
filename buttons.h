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
#include "uart.h"
#include "FSM.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/* ******************************************************
 * Constants
 * *****************************************************/

/*
 * Button Config
 */
enum btnNames   {UP = 0, DOWN, LEFT, RIGHT, NUM_BTNS};
enum btnStates  {RELEASED = 0, PUSHED, NO_CHANGE};

#define U_BTN_PERIPH        SYSCTL_PERIPH_GPIOE         // Up Peripheral
#define U_BTN_PORT_BASE     GPIO_PORTE_BASE             // Up Port Base
#define U_BTN_PIN           GPIO_PIN_0                  // Up Pin
#define U_BTN_NORMAL        false                       // Up Inactive State (Active HIGH)

#define D_BTN_PERIPH        SYSCTL_PERIPH_GPIOD         // Down Peripheral
#define D_BTN_PORT_BASE     GPIO_PORTD_BASE             // Down Port Base
#define D_BTN_PIN           GPIO_PIN_2                  // Down Pin
#define D_BTN_NORMAL        false                       // Down Inactive State (Active HIGH)

#define L_BTN_PERIPH        SYSCTL_PERIPH_GPIOF         // Left Peripheral
#define L_BTN_PORT_BASE     GPIO_PORTF_BASE             // Left Port Base
#define L_BTN_PIN           GPIO_PIN_4                  // Left Pin
#define L_BTN_NORMAL        true                        // Left Inactive State (Active LOW)

#define R_BTN_PERIPH        SYSCTL_PERIPH_GPIOF         // Right Peripheral
#define R_BTN_PORT_BASE     GPIO_PORTF_BASE             // Right Port Base
#define R_BTN_PIN           GPIO_PIN_0                  // Right Pin
#define R_BTN_NORMAL        true                        // Right Inactive State (Active LOW)

#define NUM_BTN_POLLS       3                           // Number Of Times To Poll The Buttons (For Debouncing)

/*
 * Switch Config
 */
#define SW_PERIPH           SYSCTL_PERIPH_GPIOA         // Switch Peripheral
#define SW_PORT_BASE        GPIO_PORTA_BASE             // Switch Port Base
#define L_SW_PIN            GPIO_PIN_6                  // Left Switch Pin
#define R_SW_PIN            GPIO_PIN_7                  // Right Switch Pin

/*
 * Control Config
 */
#define ALT_CHANGE          10                          // The altitude change on button press (percentage)
#define MAX_ALT             100                         // The maximum altitude (percentage)
#define MIN_ALT             0                           // The minimum altitude (percentage)
#define YAW_CHANGE          15                          // The yaw change on button press (degrees)
#define MAX_YAW             164                         // The maximum yaw (degrees) before increment
#define MIN_YAW             -165                        // The minimum yaw (degrees) before increment
#define DEGREES_CIRCLE      360                         // The number of degrees in a circle

/* ******************************************************
 * Globals
 * *****************************************************/
extern QueueHandle_t xAltBtnQueue;
extern QueueHandle_t xYawBtnQueue;
extern QueueHandle_t xAltDesQueue;
extern QueueHandle_t xYawDesQueue;
extern QueueHandle_t xFSMQueue;

extern TimerHandle_t xUpBtnTimer;
extern TimerHandle_t xYawFlipTimer;

extern SemaphoreHandle_t xAltMutex;
extern SemaphoreHandle_t xYawMutex;
extern SemaphoreHandle_t xUpBtnSemaphore;
extern SemaphoreHandle_t xYawFlipSemaphore;

extern controller_t g_yaw_controller;

// Debounce algorithm: A state machine is associated with each button.
// A state change occurs only after NUM_BUT_POLLS consecutive polls have
// read the pin in the opposite condition, before the state changes and
// a flag is set.  Set NUM_BUT_POLLS according to the polling rate.

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
