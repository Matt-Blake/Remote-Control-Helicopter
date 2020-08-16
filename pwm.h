/* ****************************************************************
 *
 * pwm.c - Example code which generates a single PWM
 *     output on J4-05 (M0PWM7) with duty cycle fixed and
 *     the frequency controlled by UP and DOWN buttons in
 *     the range 50 Hz to 400 Hz.
 * 2017: Modified for Tiva and using straightforward, polled
 *     button debouncing implemented in 'buttons4' module.
 *
 * P.J. Bones   UCECE
 * Last modified:  07/02/2018
 *
 * pwmMainGen.c - This code was based off the pwmGen.c example
 *      code. We have changed the code only generate a single PWM signal on
 *      Tiva board pin J4-05 = PC5 (M0PWM7). This is the same PWM output as
 *      the helicopter main rotor.
 *
 * ENCETue am Group 1
 * Creators: Brendain Hennessy   57190084
 *           Sarah Kennelly      76389950
 *           Matt Blake          58979250
 * Last modified: 09/05/2019
 *
 * pwm.c - This code was based off the pwmMainGen.c code from ENCE361.
 *      This code has been changed to incopretate PWM of the tail rotor.
 *      This code has been changed to incorporate PWM of the tail rotor.
 *      This is part of ENCE464 Assignment 1.
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 31/07/2020
 * ***************************************************************/


// Includes
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"

#include "uart.h"
#include "yaw.h"
#include "pidController.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"


// Includes
#ifndef PWMMAINGEN_H_
#define PWMMAINGEN_H_

#define MAIN_ROTOR_FACTOR 1/4 // Factor used to compensate for the effect of main rotor


// Globals
extern controller_t g_alt_controller;
extern controller_t g_yaw_controller;

extern SemaphoreHandle_t xAltMutex;
extern SemaphoreHandle_t xYawMutex;

extern QueueHandle_t xAltMeasQueue;
extern QueueHandle_t xAltDesQueue;
extern QueueHandle_t xYawMeasQueue;
extern QueueHandle_t xYawDesQueue;
extern QueueHandle_t xMainPWMQueue;
extern QueueHandle_t xTailPWMQueue;


/*
 * Function:    initPWM
 * ---------------------
 * Initializes the PWM output used to drive the helicopter motors.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void initPWM (void);

/*
 * Function:    setRotorPWM
 * -------------------------
 * Sets the PWM duty cycle and period of each motor.
 *
 * @params:
 *      - uint32_t ui32Duty: The duty cycle to be set for the PWM.
 *      - bool SET_MAIN: If True, sets the main rotor PWM params.
 *      If False, sets the tail rotor PWM params
 * @return:
 *      - NULL
 * ---------------------
 */
void setRotorPWM (uint32_t ui32Duty, bool SET_MAIN);

/*
 * Function:    SetMainDuty
 * -------------------------
 * FreeRTOS task that periodically calls functions to set the PWM
 * duty cycle of the main rotor required to reach the desired
 * altitude.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void SetMainDuty(void *pvParameters);

/*
 * Function:    SetTailDuty
 * -------------------------
 * FreeRTOS task that periodically calls functions to set the PWM
 * duty cycle of the tail rotor required to reach the desired yaw.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void SetTailDuty(void *pvParameters);

#endif /* _PWM_H_ */
