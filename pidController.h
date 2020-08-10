/* ******************************************************
 * piController.c
 *
 * Simple PID Controller
 *
 * Tue am Group 1
 * Creators: Brendain Hennessy   57190084
 *           Sarah Kennelly      76389950
 *           Matt Blake          58979250
 * Last modified: 9/05/2019
 *
 * pidController.c - This code was based off the piController.c code from ENCE361.
 *      This code has been changed to incopretate error signal calculation
 *      inside of the control signal calculation.
 *      This is part of ENCE464 Assignment 1.
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 05/08/2020
 * *****************************************************/

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include <stdint.h>
#include <stdbool.h>

/* ******************************************************
 * Constants
 * *****************************************************/
#define DEGREES_CIRCLE      360         // The number of degrees in a circle
#define MAX_DUTY            98          // The maximum duty cycle for the rotors
#define MIN_DUTY            2           // The minimum duty cycle for the rotors
#define MS_TO_SECONDS       1000        // Conversion factor from ms to s
#define CONTROL_DIVISOR     1           // Divisor used to achieve certain gains without the use of floating point numbers


#define ALT_KP                  1       // Altitude proportional gain
#define ALT_KI                  0       // Altitude integral gain
#define ALT_KD                  0       // Altitude derivative gain
#define YAW_KP                  1       // Yaw proportional gain
#define YAW_KI                  0       // Yaw integral gain
#define YAW_KD                  0       // Yaw derivative gain

/* ******************************************************
 * Sets up PID controller struct values that will be used
 * for both the altitude control and the yaw control.
 * *****************************************************/
typedef struct Controllers {
    uint32_t    Kp;             // Proportional gain
    uint32_t    Ki;             // Integral gain
    uint32_t    Kd;             // Derivative gain
    uint32_t    timeStep;       // The time step used to calculate derivative and integral control (in ms)
    int32_t     divisor;        // Divisor used to correct gains without the use of floating point numbers

    int32_t     previousError;
    int32_t     integratedError;
} controller_t;

/* ******************************************************
 * Sets all initial PID Controller struct values
 * *****************************************************/
void initController(controller_t* controllerPointer, uint32_t K_P, uint32_t K_I, uint32_t K_D, uint32_t time_step, int32_t divisor_value);

/* ******************************************************
 * Function reverses error signal for yaw and processes
 * the error signal so it will work with the method used
 * to log yaw which is from 0 to 179 and -180 to 0.
 *
 * The control signal is calculated, using PID gains and error signal
 *
 * Duty cycle limits are set for altitude and yaw so as
 * to not overload the helicopter rig and emulator.
 * *****************************************************/
int32_t getControlSignal(controller_t *piController, int32_t reference, int32_t measurement, bool isYaw);

#endif /* PIDCONTROLLER_H_ */
