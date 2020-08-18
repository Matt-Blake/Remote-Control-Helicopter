/*
 * piController.c - Simple PI Controller
 *
 * Tue am Group 1
 * Creators: Brendain Hennessy   57190084
 *           Sarah Kennelly      76389950
 *           Matt Blake          58979250
 * Last modified: 9/05/2019
 *
 * pidController.c - This code was based off the piController.c code from ENCE361.
 *      This code has been changed to incorporate error signal calculation
 *      inside of the control signal calculation.
 *      This is part of ENCE464 Assignment 1.
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 05/08/2020
 */

#include "pidController.h"

#define ALT_KP              30          // Altitude proportional gain
#define ALT_KI              50          // Altitude integral gain
#define ALT_KD              20          // Altitude derivative gain
#define YAW_KP              60          // Yaw proportional gain
#define YAW_KI              30          // Yaw integral gain
#define YAW_KD              20          // Yaw derivative gain
#define CONTROL_DIVISOR     100         // Divisor used to achieve certain gains without the use of floating point numbers

#define DEGREES_CIRCLE      360         // The number of degrees in a circle
#define MS_TO_SECONDS       1000        // Conversion factor from ms to s

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

    float errorSignal;
    float derivativeError;


    // Calculate error signal
    errorSignal = reference - measurement;

    //Clockwise rotation corresponds to low power in motors
    if(isYaw)
    {
        errorSignal = -errorSignal;

        // If the error would cause a rotation in the wrong direction
        if(errorSignal > (DEGREES_CIRCLE/2))
        {
            errorSignal = errorSignal - DEGREES_CIRCLE;
        } else if(errorSignal < (-(DEGREES_CIRCLE/2)))
        {
            errorSignal = DEGREES_CIRCLE - errorSignal;
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
