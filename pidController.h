/* ****************************************************************
 * pidController.h
 *
 * Header file of the PID Controller module.
 * Adapted for use for a helicopter's yaw.
 *
 * Based on piController.h
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

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "uart.h"
#include "math.h"

#define ALT_KP              45//30          // Altitude proportional gain
#define ALT_KI              15//20          // Altitude integral gain
#define ALT_KD              10//20          // Altitude derivative gain

#define YAW_KP              30          // Yaw proportional gain
#define YAW_KI              7           // Yaw integral gain
#define YAW_KD              1           // Yaw derivative gain

#define CONTROL_DIVISOR     100         // Divisor used to achieve certain gains without the use of floating point numbers

#define DEGREES_CIRCLE      360         // The number of degrees in a circle
#define MS_TO_SECONDS       1000        // Conversion factor from ms to s
#define MAX_DUTY            98          // The maximum duty cycle for the rotors
#define MIN_DUTY            2           // The minimum duty cycle for the rotors
#define CONTROL_PERIOD      20         // Period used in the control loops (ms)


/* ******************************************************
 * Sets up PID controller struct values that will be used
 * for both the altitude control and the yaw control.
 * *****************************************************/
typedef struct Controllers {
    int32_t     Kp;             // Proportional gain
    int32_t     Ki;             // Integral gain
    int32_t     Kd;             // Derivative gain
    uint32_t    timeStep;       // The time step used to calculate derivative and integral control (in ms)
    int32_t     divisor;        // Divisor used to correct gains without the use of floating point numbers

    int32_t     previousError;
    int32_t     integratedError;
} controller_t;


/*
 * Function:    initController
 * ----------------------------
 * Initializes controller struct values.
 *
 * @params:
 *      - controller_t* controllerPointer: Pointer to the relevant
 *      conroller struct.
 *      - bool isYaw: True if the controller is for yaw. False if
 *      controller is for altitude.
 *      - uint32_t timeStep: Control period in ms.
 * @return:
 *      - NULL
 * ---------------------
 */
void initController(controller_t* controllerPointer, bool isYAw);

/*
 * Function:    getControlSignal
 * ------------------------------
 * Function reverses error signal for yaw and processes
 * the error signal so it will work with the method used
 * to log yaw which is from 0 to 179 and -180 to 0.
 *
 * The control signal is calculated, using PID gains and error signal
 *
 * Duty cycle limits are set for altitude and yaw so as
 * to not overload the helicopter rig and emulator.
 *
 * @params:
 *      - controller_t* piController: Pointer to the relevant
 *      conroller struct.
 *      - int32_t reference: Target yaw/altitude
 *      - int32_t measurement: Actual yaw/altitude
 *      - bool isYaw: True if the controller is for yaw. False if
 *      controller is for altitude.
 * @return:
 *      - int32_t dutyCycle: Appropriate duty cycle for the relevant
 *      PWM output as calculated by the control system
 * ---------------------
 */
int32_t getControlSignal(controller_t *piController, int32_t reference, int32_t measurement, bool isYaw);

#endif /* PIDCONTROLLER_H_ */
