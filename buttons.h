#ifndef BUTTONS_H
#define BUTTONS_H

// ****************************************************************
// BUTTONS.c
//
// Module for the D-Pad Buttons (U/D/L/R)
// Supports buttons on the Tiva/Orbit.
// Comprises of initialisers and button checks
//
// Author: P.J. Bones UCECE
// Edited: Derrick Edward, Grayson Mynott, Ryan Earwaker
// Thu AM Group 18
// Last modified:  29.05.2019
//
// buttons.c - Code updated for use with FreeRTOS

// ENCE464 Assignment 1 Group 2
// Creators: Grayson Mynott      56353855
//           Ryan Earwaker       12832870
//           Matt Blake          58979250
// Last modified: 05/08/2020
//******************************************************


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/tm4c123gh6pm.h"  // Board specific defines (for PF0)

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "uart.h"

// ****************************************************************
// Constants
// ****************************************************************

// ** D-Pad Buttons ***********************************************
enum btnNames {UP = 0, DOWN, LEFT, RIGHT, NUM_BTNS};
enum btnStates {RELEASED = 0, PUSHED, NO_CHANGE};

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

#define SW_PERIPH           SYSCTL_PERIPH_GPIOA         // Switch Peripheral
#define SW_PORT_BASE        GPIO_PORTA_BASE             // Switch Port Base
#define L_SW_PIN            GPIO_PIN_6                  // Left Switch Pin
#define R_SW_PIN            GPIO_PIN_7                  // Right Switch Pin


extern QueueHandle_t xAltBtnQueue;
extern QueueHandle_t xYawBtnQueue;

extern SemaphoreHandle_t xAltMutex;
extern SemaphoreHandle_t xYawMutex;

//extern volatile int TARGET_YAW;     // Link the external variable TARGET_YAW
//extern volatile int TARGET_ALT;     // Link the external variable TARGET_ALT

// Debounce algorithm: A state machine is associated with each button.
// A state change occurs only after NUM_BUT_POLLS consecutive polls have
// read the pin in the opposite condition, before the state changes and
// a flag is set.  Set NUM_BUT_POLLS according to the polling rate.

// ****************************************************************
// initButtons: Initialise the variables associated with the set of buttons
// defined by the constants above.
void initBtns (void);

// ****************************************************************
// updateButtons: Function designed to be called regularly. It polls all
// buttons once and updates variables associated with the buttons if
// necessary.  It is efficient enough to be part of an ISR, e.g. from
// a SysTick interrupt.
void updateButtons (void);

// ****************************************************************
// checkButton: Function returns the new button state if the button state
// (PUSHED or RELEASED) has changed since the last call, otherwise returns
// NO_CHANGE.  The argument butName should be one of constants in the
// enumeration butStates, excluding 'NUM_BUTS'. Safe under interrupt.
uint8_t checkButton (uint8_t butName);
// @param   butname - Name of the button to compare the state of (UP/DOWN/LEFT/RIGHT)

// ****************************************************************
// buttonsCheck: checks if buttons associated with altitude and yaw have
// been pushed and increments accordingly
void ButtonsCheck(void *pvParameters);

#endif /*BUTTONS_H*/
