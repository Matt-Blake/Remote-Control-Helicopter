//*****************************************************
//
// uartDemo.c - Sends information using the serial UART.
//
// P.J. Bones UCECE
// last modified on 16.4.2018
//
// uart.h - This code is an adaptation of uartDemo.h,
//       the code is tailored to send the information
//       that is required for heli-copter project.
//
// Tue am Group 1
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          58979250
// Last modified: 9/05/2019
//******************************************************

#ifndef UART_H_
#define UART_H_

#include "driverlib/uart.h"

//********************************************************
// initialiseUSB_UART - 8 bits, 1 stop bit, no parity
//********************************************************
void
initialiseUSB_UART (void);

//********************************************************
// Puts current yaw degrees, desired yaw degrees, current altitude,
// desired altitude, main duty cycle, tail duty cycle and current state
// into a string form which can be send through the UART to the heli rig or
// serial terminal.
//********************************************************
void
UARTDisplay (int32_t yaw_degrees, int32_t yaw_desired, int32_t altitude, int32_t altitude_desired, uint32_t duty_main, uint32_t duty_tail, uint8_t g_heliState);

#endif /* UART_H_ */
