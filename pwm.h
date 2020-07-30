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
// ENCETue am Group 1
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          58979250
// Last modified: 09/05/2019
//
// pwm.c - This code was based off the pwmMainGen.c code from ENCE361.
//      This code has been changed to incorporate PWM of the tail rotor.
//      This is part of ENCE464 Assignment 1.

// ENCE464 Assignment 1 Group 2
// Creators: Grayson Mynott      56353855
//           Ryan Earwaker       12832870
//           Matt Blake          58979250
// Last modified: 31/07/2020
//******************************************************

#ifndef PWMMAINGEN_H_
#define PWMMAINGEN_H_

#include <stdint.h>
#include <stdbool.h>

//********************************************************
// Function to set the freq, duty cycle of M0PWM7.
//********************************************************
void
setRotorPWM (uint32_t ui32Duty);


//******************************************************
// initialise PWM for the main rotor.
// M0PWM7 (J4-05, PC5) is used for the main rotor motor
//******************************************************
void
initialiseMainRotorPWM (void);


//******************************************************
// initialisePWM for tail rotor.
// M0PWM7 (PF1, J4-05) is used for the tail rotor motor.
//******************************************************
void
initialiseTailRotorPWM (void);


//*********************************************************
//Turns on the main rotor so a duty cycle can be passed to it
//*********************************************************
void
turnOnMainPWM (void);


//******************************************************
// Turns on the tail rotor so a duty cycle can be passed to it
//******************************************************
void
turnOnTailPWM (void);


#endif // __PWM_H__
