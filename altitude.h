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

#define ALTITUDE_PERIOD     200         // Period used to average and calculate the altitude (ms)

QueueHandle_t    xAltMeasQueue;
QueueHandle_t    xFSMQueue;
TaskHandle_t     ADCMean;


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
