/* ****************************************************************
 * pwm.h
 *
 * Header file of the pulse width modulation (PWM) module.
 * Designed to control the speed of the helicopter's motors.
 *
 * Based off pwm.h - P.J. Bones, UCECE, 2018
 *
 * Further based on pwmMainGen.h
 * Tue AM Group 1
 * Creators: Brendain Hennessy   57190084
 *           Sarah Kennelly      76389950
 *           Matt Blake          58979250
 * Last modified: 9/05/2019
 *
 * Code changed to incorporate the tail rotor
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 19/08/2020
 *
 * ***************************************************************/

#include "pwm.h"

controller_t g_alt_controller;
controller_t g_yaw_controller;


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
void
initPWM (void)
{

    SysCtlPWMClockSet(PWM_DIVIDER_CODE);

    // Enable PWM peripherals
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    while(!SysCtlPeripheralReady(PWM_MAIN_PERIPH_PWM));

    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);
    while(!SysCtlPeripheralReady(PWM_TAIL_PERIPH_PWM));

    // Enable GPIO peripherals
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);
    while(!SysCtlPeripheralReady(PWM_MAIN_PERIPH_GPIO));

    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);
    while(!SysCtlPeripheralReady(PWM_TAIL_PERIPH_GPIO));

    // Set PWM pin types
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);

    // Set PWM GPIO
    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);

    // Configure PWM generators
    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set initial duty cycles
    setRotorPWM(PWM_FIXED_DUTY, IS_MAIN_ROTOR);         // Set the initial main PWM parameters
    setRotorPWM(PWM_FIXED_DUTY, IS_TAIL_ROTOR);         // Set the initial tail PWM parameters

    // Enable PWM generators
    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);
    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    // Disable the output.  Repeat this call with 'true' to turn output on.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);

}

/*
 * Function:    initControllers
 * -----------------------------
 * Initializes the structs used to hold the PID controllers
 * gains and errors.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
initControllers(void)
{
    initController(&g_alt_controller, false);
    initController(&g_yaw_controller, true);
}

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
void
setRotorPWM (uint32_t ui32Duty, bool SET_MAIN)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period = SysCtlClockGet() / PWM_DIVIDER / PWM_START_RATE_HZ;

    if (SET_MAIN == true) // Set PWM freq/duty of the main rotor
    {
        PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
        PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM, ui32Period * ui32Duty / CONVERT_TO_PERCENTAGE);
    }
    else // Set PWM freq/duty of the tail rotor
    {
        PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
        PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM, ui32Period * ui32Duty / CONVERT_TO_PERCENTAGE);
    }
}


/*
 * Function:    turnOnMainPWM
 * ---------------------------
 * Turns on the main motor PWM output.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
turnOnMainPWM(void)
{
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
}


/*
 * Function:    turnOnTailPWM
 * ---------------------------
 * Turns on the tail motor PWM output.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
turnOnTailPWM(void)
{
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
}


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
void
SetMainDuty(void *pvParameters)
{
    int32_t alt_PWM = 0;
    int32_t alt_meas = 0;
    int32_t alt_desired = 0;

    while (1)
    {
        // Retrieve altitude information
        xQueuePeek(xAltMeasQueue, &alt_meas,    TICKS_TO_WAIT); // Retrieve measured altitude data from the RTOS queue
        xQueuePeek(xAltDesQueue,  &alt_desired, TICKS_TO_WAIT); // Retrieve desired altitude data from the RTOS queue

        // Set PWM duty cycle of main rotor in order to hover to the desired altitude
        alt_PWM = getControlSignal(&g_alt_controller, alt_desired, alt_meas, false); // Use the error to calculate a PWM duty cycle for the main rotor
        setRotorPWM(alt_PWM, IS_MAIN_ROTOR); // Set main rotor to calculated PWM

        vTaskDelay(CONTROL_PERIOD / portTICK_RATE_MS); // Block task so lower priority tasks can run
    }
}


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
void
SetTailDuty(void *pvParameters)
{
    int32_t yaw_PWM = 0;
    int32_t yaw_meas = 0;
    int32_t yaw_desired = 0;
    int32_t alt_PWM = 0;

    while (1)
    {
        // Retrieve yaw information
        xQueuePeek(xYawMeasQueue, &yaw_meas,   TICKS_TO_WAIT); // Retrieve measured yaw data from the RTOS queue
        xQueuePeek(xYawDesQueue, &yaw_desired, TICKS_TO_WAIT); // Retrieve desired yaw data from the RTOS queue

        // Set PWM duty cycle of tail rotor in order to spin to target yaw
        yaw_PWM = getControlSignal(&g_yaw_controller, yaw_desired, yaw_meas, true); // Use the error to calculate a PWM duty cycle for the tail rotor
        yaw_PWM = yaw_PWM + (alt_PWM * MAIN_ROTOR_FACTOR); // Compensate tail PWM due to effect of main rotor duty cycle
        setRotorPWM(yaw_PWM, IS_TAIL_ROTOR); // Set tail rotor to calculated PWM

        vTaskDelay(CONTROL_PERIOD / portTICK_RATE_MS); // Block task so lower priority tasks can run
    }
}
