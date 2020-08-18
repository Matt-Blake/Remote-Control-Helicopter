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


extern EventGroupHandle_t xFoundYawReference;
extern QueueHandle_t xYawMeasQueue;
extern QueueHandle_t xYawDesQueue;
extern QueueHandle_t xYawSlotQueue;


void vYawRefCallback( TimerHandle_t xTimer );

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
