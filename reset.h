/* ****************************************************************
 * reset.h
 *
 * Header file for reset module
 * Triggers a hard rest interrupt for the Tiva board
 * Includes initialisers, interrupts and calculation functions
 *
 * Based on reset.h
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
