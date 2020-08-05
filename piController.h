//*****************************************************
// piController.h
//
// Simple PI Controller

// Tue am Group 1
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          58979250
// Last modified: 9/05/2019
//******************************************************

#ifndef PICONTROLLER_H_
#define PICONTROLLER_H_

#include <stdint.h>
#include <stdbool.h>

//******************************************************
// Sets up PID controller struct values that will be used
// for both the altitude control and the yaw control.
//******************************************************
typedef struct Controllers {
    uint32_t Kp; //Proportional gain
    uint32_t Ki; //Integral gain
    uint32_t Kd; //Derivative gain
    uint32_t timeStep;
    int32_t divisor; //Divisor used to correct gains without the use of floating point numbers

    int32_t previousError;
    int32_t intergratedError;
} Controller;

//******************************************************
// Sets all initial PID Controller struct values
//******************************************************
void init_controller(Controller* controllerPointer, uint32_t K_P, uint32_t K_I, uint32_t K_D, uint32_t time_step, int32_t divisor_value);

//******************************************************
// calculates the error signal as the reference altitude or yaw
// minus the current measured (desired) altitude or yaw.
//******************************************************
int32_t getErrorSignal(int32_t reference, int32_t measurement);

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
int32_t getControlSignal(Controller *piController, int32_t errorSignal, bool isYaw);

#endif /* PICONTROLLER_H_ */
