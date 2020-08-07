/* ******************************************************
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
 * *****************************************************/

#ifndef PWMMAINGEN_H_
#define PWMMAINGEN_H_

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

//******************************************************
// Constants
//******************************************************

// PWM configuration
#define PWM_START_RATE_HZ       200
#define PWM_FIXED_DUTY          0
#define PWM_DIVIDER_CODE        SYSCTL_PWMDIV_16
#define PWM_DIVIDER             16

//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_PERIPH_PWM     SYSCTL_PERIPH_PWM0
#define PWM_MAIN_BASE           PWM0_BASE
#define PWM_MAIN_GEN            PWM_GEN_3
#define PWM_MAIN_OUTNUM         PWM_OUT_7
#define PWM_MAIN_OUTBIT         PWM_OUT_7_BIT

#define PWM_MAIN_PERIPH_GPIO    SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE      GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG    GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN       GPIO_PIN_5

//  ---Tail Rotor PWM: PF1, J4-05
#define PWM_TAIL_PERIPH_PWM     SYSCTL_PERIPH_PWM1
#define PWM_TAIL_BASE           PWM1_BASE
#define PWM_TAIL_GEN            PWM_GEN_2
#define PWM_TAIL_OUTNUM         PWM_OUT_5
#define PWM_TAIL_OUTBIT         PWM_OUT_5_BIT

#define PWM_TAIL_PERIPH_GPIO    SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE      GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG    GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN       GPIO_PIN_1

#define CONTROL_PERIOD          20      // Period used in the control loops (ms)

// Initalise controllers
controller_t g_alt_controller;
controller_t g_yaw_controller;

extern SemaphoreHandle_t xAltMutex;
extern SemaphoreHandle_t xYawMutex;

extern QueueHandle_t xAltMeasQueue;
extern QueueHandle_t xAltRefQueue;
extern QueueHandle_t xYawRefQueue;

 /* ******************************************************
 * Function to set the freq, duty cycle of M0PWM7.
 * *****************************************************/
void
setRotorPWM (uint32_t ui32Duty, bool SET_MAIN);

 /* ******************************************************
 * Initialise PWM for the main rotor.
 * M0PWM7 (J4-05, PC5) is used for the main rotor motor
 * *****************************************************/
void
initPWM (void);

/* ******************************************************
 * Turns on the main rotor so a duty cycle can be passed to it
 * *****************************************************/
void
turnOnMainPWM (void);

/* ******************************************************
 * Turns on the tail rotor so a duty cycle can be passed to it
 * *****************************************************/
void
turnOnTailPWM (void);

void
Set_Main_Duty(void *pvParameters);

void
Set_Tail_Duty(void *pvParameters);

#endif /* _PWM_H_ */
