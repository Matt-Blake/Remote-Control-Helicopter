//*****************************************************
//
// pwmGen.c - Example code which generates a single PWM
//     output on J4-05 (M0PWM7) with duty cycle fixed and
//     the frequency controlled by UP and DOWN buttons in
//     the range 50 Hz to 400 Hz.
// 2017: Modified for Tiva and using straightforward, polled
//     button debouncing implemented in 'buttons4' module.
//
// P.J. Bones   UCECE
// Last modified:  7.2.2018
//
// pwm.c - This code was based off the pwmGen.c example
//      code. We have changed the code only generate a single PWM signal on
//      Tiva board pin J4-05 = PC5 (M0PWM7). This is the same PWM output as
//      the helicopter main rotor.
//
// Tue am Group 1
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          58979250
// Last modified: 9/05/2019
//******************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "buttons.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "utils/ustdlib.h"

//******************************************************
// Constants
//******************************************************

// PWM configuration
#define PWM_START_RATE_HZ  200
#define PWM_FIXED_DUTY     0
#define PWM_DIVIDER_CODE   SYSCTL_PWMDIV_4
#define PWM_DIVIDER        1
#define PWM_MAIN_START_RATE_HZ  200
#define PWM_MAIN_FIXED_DUTY     0
#define PWM_DIVIDER_CODE        SYSCTL_PWMDIV_4
#define PWM_DIVIDER             1

//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_BASE           PWM0_BASE
#define PWM_MAIN_GEN            PWM_GEN_3
#define PWM_MAIN_OUTNUM         PWM_OUT_7
#define PWM_MAIN_OUTBIT         PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM     SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO    SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE      GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG    GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN       GPIO_PIN_5

//  ---Tail Rotor PWM: PF1, J4-05
#define PWM_TAIL_BASE        PWM1_BASE
#define PWM_TAIL_GEN         PWM_GEN_2
#define PWM_TAIL_OUTNUM      PWM_OUT_5
#define PWM_TAIL_OUTBIT      PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM  SYSCTL_PERIPH_PWM1
#define PWM_TAIL_PERIPH_GPIO SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE   GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN    GPIO_PIN_1


//********************************************************
// Function to set the freq, duty cycle of M0PWM7.
//********************************************************
void
setMainRotorPWM (uint32_t ui32Duty)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period = SysCtlClockGet() / PWM_DIVIDER / PWM_MAIN_START_RATE_HZ;
}


//******************************************************
// initialise PWM for the main rotor.
// M0PWM7 (J4-05, PC5) is used for the main rotor motor
//******************************************************
void
initialiseMainRotorPWM (void)
{
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    setMainRotorPWM (PWM_MAIN_FIXED_DUTY);

    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
}


//******************************************************
// initialise PWM for tail rotor.
// M0PWM7 (PF1, J4-05) is used for the tail rotor motor.
//******************************************************
void
initialiseTailRotorPWM (void)
{
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);

    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);

    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    setTailPWM(PWM_TAIL_FIXED_DUTY);

    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
}


//********************************************************
// Function to set the freq, duty cycle of M0PWM7.
//********************************************************
void
setRotorPWM (uint32_t ui32Duty, uint8_t set_main_rotor)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
        SysCtlClockGet() / PWM_DIVIDER / PWM_START_RATE_HZ;

    if (set_main_rotor == 1) // Perform PWM of the main rotor
    {
        PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
        PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM,
                         ui32Period * ui32Duty / 100);
    }
    else // Perform PWM of the tail rotor
    {
        PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
        PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM,
                         ui32Period * ui32Duty / 100);

    }
}


//*********************************************************
// Turns on the main rotor so a duty cycle can be passed to it
//*********************************************************
void
turnOnMainPWM(void)
{
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
}


//******************************************************
// Turns on the tail rotor so a duty cycle can be passed to it
//******************************************************
void
turnOnTailPWM(void)
{
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
}
