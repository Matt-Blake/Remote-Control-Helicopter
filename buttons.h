/* ****************************************************************
 * buttons.h
 *
 * Header file for the buttons module
 * Supports buttons on the Tiva/Orbit.
 * Comprises of initialisers and button checks
 *
 * Based on buttons4.h - P.J. Bones, UCECE
 *
 * Further based on BUTTONS.h
 * Thu AM Group 18
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Derrick Edward      18017758
 * Last modified: 29/05/2019
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
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "uart.h"
#include "FreeRTOSCreate.h"

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

#define ALT_CHANGE          10                          // The altitude change on button press (percentage)
#define MODE_1_ALT          50                          // The altitude to fly to on a double up button press
#define MAX_ALT             100                         // The maximum altitude (percentage)
#define MIN_ALT             0                           // The minimum altitude (percentage)

#define YAW_CHANGE          15                          // The yaw change on button press (degrees)
#define MODE_2_YAW_CHANGE   180                         // The change in yaw on double down button press
#define MAX_YAW             179                         // The maximum yaw (degrees)
#define MIN_YAW             -180                        // The minimum yaw (degrees)
#define DEGREES_CIRCLE      360                         // The number of degrees in a circle

#define BUTTON_PERIOD       25                          // The period used for the button polling FreeRTOS task (ms)
#define TIMER_EXPIRY        1                           // The value used to indicate a FreeRTOS time has expired

enum btnNames   {UP = 0, DOWN, LEFT, RIGHT, NUM_BTNS};
enum btnStates  {RELEASED = 0, PUSHED, NO_CHANGE};
typedef enum HELI_STATE {LANDED = 0, TAKEOFF = 1, FLYING = 2, LANDING = 3} HELI_STATE;

static bool btn_state[NUM_BTNS];    // Corresponds to the electrical state
static bool btn_normal[NUM_BTNS];   // Corresponds to the electrical state
static bool btn_flag[NUM_BTNS];
static uint8_t btn_count[NUM_BTNS];

extern QueueHandle_t xAltDesQueue;
extern QueueHandle_t xYawDesQueue;
extern QueueHandle_t xFSMQueue;

extern SemaphoreHandle_t xUpBtnSemaphore;
extern SemaphoreHandle_t xDownBtnSemaphore;
extern SemaphoreHandle_t xYawFlipSemaphore;

extern TimerHandle_t xUpBtnTimer;
extern TimerHandle_t xDownBtnTimer;


/*
 * Function:    vBtnTimerCallback
 * -------------------------------
 * Handler for the button timer.
 * When the button timer expires, this function
 * resets the timer ID.
 *
 * @params:
 *      - TimeHandle_t xTimer - the timer being handled
 * @return:
 *      - NULL
 * ---------------------
 */
void vDblBtnTimerCallback( TimerHandle_t xTimer );

/*
 * Function:    initBtns
 * ---------------------
 * Initialises the Tiva board and Orbit Boosterback's
 * buttons and switches for user input.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void initBtns (void);

/*
 * Function:    updateButtons
 * ---------------------
 * Polls the buttons to check for button presses.
 * This includes a FSM where a state change occurs
 * only after NUM_BTN_POLLS consecutive polls have
 * read the pin in the opposite condition, before the state changes and
 * a flag is set.  Set NUM_BTN_POLLS according to the polling rate.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void updateButtons(void);

/*
 * Function:    checkButton
 * ---------------------
 * Function that returns if the logic state of a button has
 * changed since last called.
 *
 * @params:
 *      - uint8_t btnName - the button being checked
 * @return:
 *      - uint8_t btnState - The chance in the button's state
 * ---------------------
 */
uint8_t checkButton(uint8_t btnName);

/*
 * Function:    upButtonPush
 * ---------------------
 * Handler for the up button.
 * Increments the helicopter's desired altitude by 10%
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void upButtonPush(void);

/*
 * Function:    downButtonPush
 * ---------------------
 * Handler for the down button.
 * Decrements the helicopter's desired altitude by 10%
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void downButtonPush(void);

/*
 * Function:    rightButtonPush
 * ---------------------
 * Handler for the right button.
 * Increments the helicopter's desired altitude by 10%
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void rightButtonPush(void);

/*
 * Function:    leftButtonPush
 * ---------------------
 * Handler for the left button.
 * decrements the helicopter's desired altitude by 10%
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void leftButtonPush(void);

/*
 * Function:    ButtonsCheck
 * ---------------------
 * FreeRTOS task which polls the buttons to check for button presses
 * For each button the following procedure is run:
 *
 * If Button State is PUSHED:
 *      Update Target Altitude/Yaw accordingly
 *      If Target Alt/Yaw is now beyond the limits:
 *          Update targets to be at limit (0->100 for Alt, -180->179 for Yaw).
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void ButtonsCheck(void *pvParameters);

/*
 * Function:    SwitchesCheck
 * ---------------------
 * FreeRTOS task which polls the switches to check for switch pushes
 * For each button the following procedure is run:
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void SwitchesCheck(void *pvParameters);

#endif /*BUTTONS_H*/
