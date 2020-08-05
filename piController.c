//*****************************************************
// piController.c - Simple PI Controller
//
// Tue am Group 1
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          58979250
// Last modified: 9/05/2019
//******************************************************

#include "piController.h"

//******************************************************
// Sets all initial PID Controller struct values
//******************************************************
void init_controller(Controller* controllerPointer, uint32_t K_P, uint32_t K_I, uint32_t K_D, uint32_t time_step, int32_t divisor_value)
{
    controllerPointer ->Kp = K_P;
    controllerPointer ->Ki= K_I;
    controllerPointer ->Kd= K_D;
    controllerPointer ->timeStep = time_step;
    controllerPointer ->divisor = divisor_value;

    controllerPointer ->previousError = 0;
    controllerPointer ->intergratedError = 0;
}

//******************************************************
// calculates the error signal as the reference altitude or yaw
// minus the current measured (desired) altitude or yaw.
//******************************************************
int32_t getErrorSignal(int32_t reference, int32_t measurement)
{
    int32_t errorSignal;

    errorSignal = reference - measurement;
    return errorSignal;
}

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
int32_t getControlSignal(Controller *piController, int32_t errorSignal, bool isYaw)
{
    int32_t controlSignal;
    int32_t dutyCycle;
    int32_t derivativeError;

    //Clockwise rotation corresponds to low power in motors
    if(isYaw) {
        errorSignal = -errorSignal;

        //If the error would cause a rotation in the wrong direction
        if(errorSignal > 180) {
            errorSignal = errorSignal - 360;
        } else if(errorSignal < -180) {
            errorSignal = 360 - errorSignal;
        }
    }

    //Calculate the control signal using PID methods and duty cycle
    derivativeError = (errorSignal - piController->previousError)/(piController->timeStep);

    piController->intergratedError += piController->timeStep * errorSignal;

    controlSignal = (piController->Kp * errorSignal)  + (piController->Ki * piController->intergratedError) + (piController->Kd) * derivativeError;

    dutyCycle = controlSignal/(piController->divisor);

    piController->previousError = errorSignal;

    //Enforce duty cycle output limits
    if(dutyCycle > 70 && isYaw) {
        dutyCycle = 70;
        piController->intergratedError -= piController->timeStep * errorSignal;
    } else if(dutyCycle > 80) {
        dutyCycle = 80;
        piController->intergratedError -= piController->timeStep * errorSignal;
    } else if(dutyCycle < 0) {
        dutyCycle = 0;
        piController->intergratedError -= piController->timeStep * errorSignal;
    }
    return dutyCycle;
}
