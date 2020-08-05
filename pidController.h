//*****************************************************
// piController.c
//
// Simple PID Controller

// Tue am Group 1
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          58979250
// Last modified: 9/05/2019

// pidController.c - This code was based off the piController.c code from ENCE361.
//      This code has been changed to incopretate error signal calculation
//      inside of the control signal calculation.
//      This is part of ENCE464 Assignment 1.

// ENCE464 Assignment 1 Group 2
// Creators: Grayson Mynott      56353855
//           Ryan Earwaker       12832870
//           Matt Blake          58979250
// Last modified: 05/08/2020
//******************************************************

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include <stdint.h>
#include <stdbool.h>

//******************************************************
// Sets up PID controller struct values that will be used
// for both the altitude control and the yaw control.
//******************************************************
typedef struct Controllers {
    uint32_t Kp; // Proportional gain
    uint32_t Ki; // Integral gain
    uint32_t Kd; // Derivative gain
    uint32_t timeStep; // The time step used to calculate derivative and integral control
    int32_t divisor; // Divisor used to correct gains without the use of floating point numbers

    int32_t previousError;
    int32_t intergratedError;
} Controller;

//******************************************************
// Sets all initial PID Controller struct values
//******************************************************
void initController(Controller* controllerPointer, uint32_t K_P, uint32_t K_I, uint32_t K_D, uint32_t time_step, int32_t divisor_value);

//******************************************************
// Function reverses error signal for yaw and processes
// the error signal so it will work with the method used
// to log yaw which is from 0 to 179 and -180 to 0.
//
// The control signal is calculated, using PID gains and error signal
//
// Duty cycle limits are set for altitude and yaw so as
// to not overload the helicopter rig and emulator.
//******************************************************
int16_t getControlSignal(Controller *piController, int16_t reference, int16_t measurement, bool isYaw);

#endif /* PIDCONTROLLER_H_ */
