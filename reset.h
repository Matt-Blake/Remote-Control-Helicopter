//*****************************************************
//
// reset.h - Initialises the helicopter to have a hard reset interrupt
//
// Tue am Group 1
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          58979250
// Last modified: 9/05/2019
//******************************************************

#ifndef RESET_H_
#define RESET_H_

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "uart.h"

#define RESET_GPIO_BASE  GPIO_PORTA_BASE //Sets the base for pins J1-03 (PB0, channel A) and J1-04 (PB1, channel B)
#define RESET_GPIO_PIN   GPIO_INT_PIN_6


/*
 * Function:    initReset
 * ----------------------------
 * Initializes the reset interrupt on Port A Pin 6.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void initReset(void);

#endif /* RESET_H_ */
