/* ****************************************************************
 * ADC.h
 *
 * Header file for Analogue to Digital Conversion and Altitude Calculations
 * Uses Successive-Approximation Quantiser
 * Includes initialisers, interrupts and calculation functions
 *
 * Derrick Edward, Grayson Mynott, Ryan Earwaker
 * Thu AM Group 18
 * Last modified:  29.05.2019
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


#define ADC_BUF_SIZE            8               // Size of the circular buffer used for moving averages
//#define VOLTAGE_DROP_ADC    4096                // FOR ORBIT BOARD POTENTIOMETER
#define VOLTAGE_DROP_ADC        1200            // FOR RIG
#define GROUND_NOT_FOUND        (0 << 0)        // Flag value to indicate that ground reference hasn't been found
#define GROUND_BUFFER_FULL      (1 << 0)        // Flag value to indicate that the ADC buffer is full
#define GROUND_FOUND            (1 << 1)        // Flag value to indicate that ground reference has been found

#define ADC_STACK_DEPTH         32              // ADC trigger task stack depth


extern circBuf_t g_inBuffer;
extern EventGroupHandle_t xFoundAltReference;                                                                                               // Should init xFoundAltRef in altitude.c or ADC.c, not main
extern TaskHandle_t ADCTrig;


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
