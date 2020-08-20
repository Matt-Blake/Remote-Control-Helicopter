/* ****************************************************************
 * altitude.h
 *
 * Header file for the altitude module
 * Calculates the altitude of the helicopter based on buffer values
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 19/08/2020
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

#define HUNDRED_PERCENT     100         // 100 percent value used for percentage calculations

extern QueueHandle_t    xAltMeasQueue;
extern QueueHandle_t    xFSMQueue;
extern TaskHandle_t     ADCMean;
extern circBuf_t g_inBuffer;


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
