//*****************************************************
// piController.c - Simple PI Controller
//
// Tue am Group 1
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          58979250
// Last modified: 9/05/2019

// pidController.c - This code was based off the piController.c code from ENCE361.
//      This code has been changed to incorporate error signal calculation
//      inside of the control signal calculation.
//      This is part of ENCE464 Assignment 1.

// ENCE464 Assignment 1 Group 2
// Creators: Grayson Mynott      56353855
//           Ryan Earwaker       12832870
//           Matt Blake          58979250
// Last modified: 05/08/2020
//******************************************************

#include <pidController.h>

//******************************************************
// Sets all initial PID Controller struct values
//******************************************************
void initController(controller_t* controllerPointer, uint32_t K_P, uint32_t K_I, uint32_t K_D, uint32_t time_step, int32_t divisor_value)
{
    controllerPointer->Kp = K_P;
    controllerPointer->Ki = K_I;
    controllerPointer->Kd = K_D;
    controllerPointer->timeStep = time_step;
    controllerPointer->divisor = divisor_value;

    controllerPointer->previousError = 0;
    controllerPointer->integratedError = 0;
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
int16_t getControlSignal(controller_t* piController, int16_t reference, int16_t measurement, bool isYaw)
{
    int16_t dutyCycle;
    int16_t controlSignal;
    int16_t errorSignal;
    int32_t derivativeError;


    // Calculate error signal
    errorSignal = reference - measurement;

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

    piController->integratedError += piController->timeStep * errorSignal;

    controlSignal = (piController->Kp * errorSignal)  + (piController->Ki * piController->integratedError) + (piController->Kd) * derivativeError;

    dutyCycle = controlSignal/(piController->divisor);

    piController->previousError = errorSignal;

    //Enforce duty cycle output limits
    if(dutyCycle > 70 && isYaw) {
        dutyCycle = 70;
        piController->integratedError -= piController->timeStep * errorSignal;
    } else if(dutyCycle > 80) {
        dutyCycle = 80;
        piController->integratedError -= piController->timeStep * errorSignal;
    } else if(dutyCycle < 0) {
        dutyCycle = 0;
        piController->integratedError -= piController->timeStep * errorSignal;
    }
    return dutyCycle;
}
