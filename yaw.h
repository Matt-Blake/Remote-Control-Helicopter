//*****************************************************
//
// yaw.c - Initializes quadrature decoder to deal with yaw.
// Controls the increase and decrease of yaw using button
// based interrupts. Finds the yaw zero reference. Converts
// quadrature values into degrees.
//
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          57190084
// Last modified: 9/05/2019
//
//******************************************************

#ifndef YAW_H_
#define YAW_H_

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "uart.h"

/* ******************************************************
 * Converts reference yaw to degrees
 * *****************************************************/
int32_t getReferenceYaw(void);

/* ******************************************************
 * Flag for having found the zero reference
 * *****************************************************/
int16_t haveFoundZeroReferenceYaw(void);

/* ******************************************************
 * Initialization of the GPIO peripherals and interrupts
 * used for the quadrature decoding.
 * *****************************************************/
void initQuadrature(void);

/* ******************************************************
 * Converts yaw to degrees.
 * *****************************************************/
int32_t getYawDegrees(void);

/* ******************************************************
 * Initializes the quadrature decoders used to calculate
 * the yaw.
 * *****************************************************/
void initReferenceYaw(void);

#endif /* YAW_H_ */
