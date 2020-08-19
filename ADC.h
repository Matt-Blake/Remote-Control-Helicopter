/* ****************************************************************
 * ADC.h
 *
 * Header file for Analogue to Digital Conversion Module
 * Uses Successive-Approximation Quantiser
 * Includes initialisers, interrupts and calculation functions
 *
 * Based off ADCdemo1.h - P.J. Bones, UCECE, 2018
 *
 * Further based on ADC.h
 * Thu AM Group 18
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Derrick Edward      18017758
 * Last modified: 29/05/2019

 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 19/08/2020
 *
 * ***************************************************************/

#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/adc.h"
#include "OrbitOLED/circBufT.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "uart.h"

#define SEQ_NUM                 3
#define ADC_PERIPH              SYSCTL_PERIPH_ADC0
#define ADC_BASE                ADC0_BASE
#define ADC_PRIORITY            1
#define ADC_CHANNEL             ADC_CTL_CH9
#define ADC_BUF_SIZE            8                   // Size of the circular buffer used for moving averages
#define SAMPLING_PERIOD         10                  // Period used to sample the altitude (ms)
#define VOLTAGE_DROP_ADC        1200                // Voltage drop value found on HeliRig
#define GROUND_NOT_FOUND        (0 << 0)            // Flag value to indicate that ground reference hasn't been found
#define GROUND_BUFFER_FULL      (1 << 0)            // Flag value to indicate that the ADC buffer is full
#define GROUND_FOUND            (1 << 1)            // Flag value to indicate that ground reference has been found

circBuf_t g_inBuffer;

extern EventGroupHandle_t xFoundAltReference;                                                                                               // Should init xFoundAltRef in altitude.c or ADC.c, not main


/*
 * Function:    initADC
 * ---------------------
 * Initializes the Analog-Digital conversion.
 * Enables the ADC0 peripheral.
 * Configures the ADC0 sequence on Channel 9.
 * Configures and enables the ADC interrupt.
 * Initializes the circular buffer used to store ADC values.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void initADC(void);

/*
 * Function:    Trigger_ADC
 * -------------------------
 * FreeRTOS task that periodically triggers the ADC interrupt in
 * order to run the ADC interrupt handler.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void TriggerADC(void *pvParameters);



#endif /*ADC_H*/
