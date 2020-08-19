/*
 *
 * yaw.c - Initializes quadrature decoder to deal with yaw.
 * Controls the increase and decrease of yaw using button
 * based interrupts. Finds the yaw zero reference. Converts
 * quadrature values into degrees.
 *
 * Creators: Brendain Hennessy   57190084
 *           Sarah Kennelly      76389950
 *           Matt Blake          57190084
 * Last modified: 9/05/2019
 *
 */

#ifndef YAW_H_
#define YAW_H_

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"
#include "uart.h"

#define YAW_REFERENCE_FLAG  (1 << 0)
#define YAW_REF_TMR_PERIOD  1000
#define MAX_YAW_SLOTS       448
#define MIN_YAW_SLOTS       -448
#define DEGREES_HALF_CIRCLE 180                         // The number of degrees in a half circle
#define DEGREES_CIRCLE      360                         // The number of degrees in a circle
#define MAX_YAW_LIMIT       179                         // The maximum yaw (degrees)
#define MIN_YAW_LIMIT       -180                        // The minimum yaw (degrees)
#define YAW_GPIO_BASE       GPIO_PORTB_BASE             // Sets the base for pins J1-03 (PB0, channel A) and J1-04 (PB1, channel B)
#define QEI_PIN0            GPIO_INT_PIN_0
#define QEI_PIN1            GPIO_INT_PIN_1
#define YAW_REFERENCE_BASE  GPIO_PORTC_BASE
#define YAW_REFERENCE_PIN   GPIO_INT_PIN_4

extern EventGroupHandle_t xFoundYawReference;
QueueHandle_t xYawMeasQueue;
QueueHandle_t xYawDesQueue;
QueueHandle_t xYawSlotQueue;


/*
 * Function:    initQuadrature
 * ----------------------------
 * Initialises the pins and interrupts for quadrature decoding.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void initQuadrature(void);

/*
 * Function:    initReferenceYaw
 * ------------------------------
 * Initialises the pins and interrupt for the yaw reference.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void initReferenceYaw(void);

#endif /* YAW_H_ */
