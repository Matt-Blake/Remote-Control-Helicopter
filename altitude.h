/* ****************************************************************
 * altitude.h
 *
 * Header file for calculating the altitude of the helicopter
 *
 * Grayson Mynott, Matt Blake, Ryan Earwaker
 * Group 2
 * Last modified:  13.08.2020
 *
 * ***************************************************************/

#ifndef ALTITUDE_H_
#define ALTITUDE_H_


#include <stdint.h>
#include <stdbool.h>

#include "ADC.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "utils/ustdlib.h"
#include "OrbitOLED/circBufT.h"
#include "uart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

extern QueueHandle_t    xAltMeasQueue;
extern QueueHandle_t    xFSMQueue;
extern circBuf_t        g_inBuffer;


/*
 * Function:    Mean_ADC
 * ----------------------
 * FreeRTOS task that periodically calls functions to calculate the
 * current average value of the circular buffer, and then converts
 * to percentage height.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void Mean_ADC(void *pvParameters);

#endif /* ALTITUDE_H_ */
