/* ****************************************************************
 * pidController.c
 *
 * Source file of the PID Controller module.
 * Adapted for use for a helicopter's yaw.
 *
 * Based on piController.c
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

#include "pidController.h"

controller_t g_alt_controller;
controller_t g_yaw_controller;

/*
 * Function:    initController
 * ----------------------------
 * Initializes controller struct values.
 *
 * @params:
 *      - controller_t* controllerPointer: Pointer to the relevant
 *      controller struct.
 *      - bool isYaw: True if the controller is for yaw. False if
 *      controller is for altitude.
 *      - uint32_t timeStep: Control period in ms.
 * @return:
 *      - NULL
 * ---------------------
 */
void
initController(controller_t* controllerPointer, bool isYaw)
{
    if (isYaw){
        controllerPointer->Kp = YAW_KP;
        controllerPointer->Ki = YAW_KI;
        controllerPointer->Kd = YAW_KD;
    } else {
        controllerPointer->Kp = ALT_KP;
        controllerPointer->Ki = ALT_KI;
        controllerPointer->Kd = ALT_KD;
    }
    controllerPointer->timeStep = CONTROL_PERIOD;
    controllerPointer->divisor = CONTROL_DIVISOR;

    controllerPointer->previousError = 0;
    controllerPointer->integratedError = 0;
}

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
int32_t
getControlSignal(controller_t* piController, int32_t reference, int32_t measurement, bool isYaw)
{
    int32_t dutyCycle;
    int32_t controlSignal;

    double errorSignal;
    double derivativeError;

    // Calculate error signal
    errorSignal = reference - measurement;

    //Clockwise rotation corresponds to low power in motors
    if(isYaw)
    {
        //usnprintf(string, sizeof(string), "ref   %4d\n\r", round(reference));
        //UARTSend(string);
        //usnprintf(string, sizeof(string), "meas   %4d\n\r", round(measurement));
        //UARTSend(string);
        //usnprintf(string, sizeof(string), "Error   %4d\n", round(errorSignal));
        //UARTSend(string);
        //errorSignal = -errorSignal;
        // If the error would cause a rotation in the wrong direction
        if(errorSignal >= (DEGREES_CIRCLE/2))
        {
            errorSignal = errorSignal - DEGREES_CIRCLE;
        } else if(errorSignal < (-(DEGREES_CIRCLE/2)))
        {
            errorSignal = DEGREES_CIRCLE + errorSignal;
        }
    }

    //Calculate the control signal using PID methods and duty cycle
    derivativeError = (errorSignal - piController->previousError)/(piController->timeStep);
    piController->integratedError += piController->timeStep * errorSignal;

    controlSignal = (piController->Kp * errorSignal)  + (piController->Ki * piController->integratedError)/MS_TO_SECONDS + (piController->Kd) * derivativeError * MS_TO_SECONDS;
    dutyCycle = (controlSignal/(piController->divisor));

    piController->previousError = errorSignal;

    //Enforce duty cycle output limits
    if(dutyCycle > MAX_DUTY)
    {
        dutyCycle = MAX_DUTY;
        piController->integratedError -= piController->timeStep * errorSignal/MS_TO_SECONDS;
    } else if(dutyCycle < MIN_DUTY)
    {
        dutyCycle = MIN_DUTY;
    }

    return dutyCycle;
}
