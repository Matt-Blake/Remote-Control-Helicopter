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
#include "utils/ustdlib.h"
#include "OrbitOLED/circBufT.h"
#include "uart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"

#define MEAN_STACK_DEPTH        64
#define MEAN_TASK_PRIORITY      7       // Mean calculation priority

extern QueueHandle_t    xAltMeasQueue;
extern QueueHandle_t    xFSMQueue;
extern circBuf_t        g_inBuffer;
extern TaskHandle_t     ADCMean;

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
void MeanADC(void *pvParameters);

#endif /* ALTITUDE_H_ */
