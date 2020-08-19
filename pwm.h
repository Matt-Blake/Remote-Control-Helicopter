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

#ifndef PWM_H_
#define PWM_H_


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "yaw.h"
#include "pidController.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "uart.h"

//  PWM Hardware Details M0PWM7 (gen 3)
#define PWM_START_RATE_HZ       200
#define PWM_FIXED_DUTY          0
#define PWM_DIVIDER_CODE        SYSCTL_PWMDIV_16
#define PWM_DIVIDER             16

//  Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_PERIPH_PWM     SYSCTL_PERIPH_PWM0
#define PWM_MAIN_BASE           PWM0_BASE
#define PWM_MAIN_GEN            PWM_GEN_3
#define PWM_MAIN_OUTNUM         PWM_OUT_7
#define PWM_MAIN_OUTBIT         PWM_OUT_7_BIT

#define PWM_MAIN_PERIPH_GPIO    SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE      GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG    GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN       GPIO_PIN_5

//  Tail Rotor PWM: PF1, J4-05
#define PWM_TAIL_PERIPH_PWM     SYSCTL_PERIPH_PWM1
#define PWM_TAIL_BASE           PWM1_BASE
#define PWM_TAIL_GEN            PWM_GEN_2
#define PWM_TAIL_OUTNUM         PWM_OUT_5
#define PWM_TAIL_OUTBIT         PWM_OUT_5_BIT

#define PWM_TAIL_PERIPH_GPIO    SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE      GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG    GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN       GPIO_PIN_1

#define MAIN_ROTOR_FACTOR       64/100 // Factor used to compensate for the effect of main rotor

controller_t g_alt_controller;
controller_t g_yaw_controller;

extern QueueHandle_t xAltMeasQueue;
extern QueueHandle_t xAltDesQueue;
extern QueueHandle_t xYawMeasQueue;
extern QueueHandle_t xYawDesQueue;
QueueHandle_t xMainPWMQueue;
QueueHandle_t xTailPWMQueue;

extern SemaphoreHandle_t xAltMutex;
extern SemaphoreHandle_t xYawMutex;

extern TaskHandle_t MainPWM;
extern TaskHandle_t TailPWM;


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
 * Function:    initControllers
 * -----------------------------
 * Initializes the structs used to hold the PWM PID controller
 * gains and errors.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
initControllers(void);
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
