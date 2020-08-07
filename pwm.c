//*****************************************************
//
// pwm.c - Example code which generates a single PWM
//     output on J4-05 (M0PWM7) with duty cycle fixed and
//     the frequency controlled by UP and DOWN buttons in
//     the range 50 Hz to 400 Hz.
// 2017: Modified for Tiva and using straightforward, polled
//     button debouncing implemented in 'buttons4' module.
//
// P.J. Bones   UCECE
// Last modified:  07/02/2018
//
// pwmMainGen.c - This code was based off the pwmGen.c example
//      code. We have changed the code only generate a single PWM signal on
//      Tiva board pin J4-05 = PC5 (M0PWM7). This is the same PWM output as
//      the helicopter main rotor.
//
// ENCE Tue am Group 1
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          58979250
// Last modified: 09/05/2019
//
// pwm.c - This code was based off the pwmMainGen.c code from ENCE361.
//      This code has been changed to incorporate PWM of the tail rotor.
//      This is part of ENCE464 Assignment 1.
//
// ENCE464 Assignment 1 Group 2
// Creators: Grayson Mynott      56353855
//           Ryan Earwaker       12832870
//           Matt Blake          58979250
// Last modified: 31/07/2020
//******************************************************

#include "pwm.h"

/*
 * initialise PWM for the main rotor.
 * M0PWM7 (J4-05, PC5) is used for the main rotor motor
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
    setRotorPWM(PWM_FIXED_DUTY, 1);         // Set the initial main PWM parameters
    setRotorPWM(PWM_FIXED_DUTY, 0);         // Set the initial tail PWM parameters

    // Enable PWM generators
    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);
    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    // Disable the output.  Repeat this call with 'true' to turn output on.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);

}


/*
 * Function to set the freq, duty cycle of M0PWM7.
 */
void
setRotorPWM (uint32_t ui32Duty, bool SET_MAIN)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period = SysCtlClockGet() / PWM_DIVIDER / PWM_START_RATE_HZ;

    if (SET_MAIN == true) // Set PWM freq/duty of the main rotor
    {
        PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
        PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM, ui32Period * ui32Duty / 100);
    }
    else // Set PWM freq/duty of the tail rotor
    {
        PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
        PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM, ui32Period * ui32Duty / 100);
    }
}

/*
 * Turns on the main rotor so a duty cycle can be passed to it
 */
void
turnOnMainPWM(void)
{
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
}

/*
 * Turns on the tail rotor so a duty cycle can be passed to it
 */
void
turnOnTailPWM(void)
{
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
}

/*
 * RTOS task that controls the main rotor speed in order to reach the desired altitude.
 */
void
Set_Main_Duty(void *pvParameters)
{
    int32_t alt_PWM;
    int32_t alt_meas;
    int32_t alt_desired;

    while (1)
    {
        if(xSemaphoreTake(xAltMutex, 0/portTICK_RATE_MS) == pdPASS){ // If the altitude mutex is free, apply the desired main rotor duty cycle

            // Retrieve altitude information
            xQueuePeek(xAltMeasQueue, &alt_meas,    10); // Retrieve measured altitude data from the RTOS queue
            xQueuePeek(xAltRefQueue,  &alt_desired, 10); // Retrieve desired altitude data from the RTOS queue

            // Set PWM duty cycle of main rotor in order to hover to the desired altitude
            alt_PWM = getControlSignal(&g_alt_controller, alt_desired, alt_meas, false); // Use the error to calculate a PWM duty cycle for the main rotor
            setRotorPWM(alt_PWM, 1); // Set main rotor to calculated PWM
            //xQueueOverwrite(xAltRefQueue, &alt_desired);
            xSemaphoreGive(xAltMutex); // Give alt mutex so other mutually exclusive altitude tasks can run
        }
        vTaskDelay(CONTROL_PERIOD / portTICK_RATE_MS); // Block task so lower priority tasks can run
    }
}

/*
 * RTOS task that controls the tail rotor speed in order to reach the desire yaw.
 */
void
Set_Tail_Duty(void *pvParameters)
{
    int16_t yaw_PWM;
    int16_t yaw_meas;
    int16_t yaw_desired;

    while (1)
    {
        if(xSemaphoreTake(xYawMutex, 0/portTICK_RATE_MS) == pdPASS){ // If the yaw mutex is free, apply the desired tail rotor duty cycle

            // Retrieve yaw information
            yaw_meas = getYawDegrees(); // Retrieve measured yaw data
            xQueueReceive(xYawRefQueue, &yaw_desired, 10); // Retrieve desired yaw data from the RTOS queue

            // Set PWM duty cycle of tail rotor in order to spin to target yaw
            yaw_PWM = getControlSignal(&g_yaw_controller, yaw_desired, yaw_meas, true); // Use the error to calculate a PWM duty cycle for the tail rotor
            setRotorPWM(yaw_PWM, 0); // Set tail rotor to calculated PWM

            xSemaphoreGive(xYawMutex); // Give yaw mutex so other mutually exclusive yaw tasks can run
        }
        vTaskDelay(CONTROL_PERIOD / portTICK_RATE_MS); // Block task so lower priority tasks can run
    }
}
