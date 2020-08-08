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

#define MOUNTSLOTCOUNT      112
#define DEGREES             180

#define YAW_GPIO_BASE       GPIO_PORTB_BASE //Sets the base for pins J1-03 (PB0, channel A) and J1-04 (PB1, channel B)
#define YAW_PIN0_GPIO_PIN   GPIO_INT_PIN_0
#define YAW_PIN1_GPIO_PIN   GPIO_INT_PIN_1

#define YAW_REFERENCE_BASE  GPIO_PORTC_BASE
#define YAW_REFERENCE_PIN   GPIO_INT_PIN_4

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
