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

/*
 * Include files.
 */
#include <stdint.h>
#include <stdbool.h>

#include "driverlib/adc.h"
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



// ************************* GLOBALS ******************************
#define ADC_PERIOD              80
#define ALTITUDE_PERIOD         200

circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
extern QueueHandle_t xAltMeasQueue;
#define BUF_SIZE 20

static int8_t groundFound = -1;

// *********************** PROTOTYPES *****************************
// ****************************************************************
// ADCIntHandler: The handler for the ADC conversion complete interrupt
// Writes to the circular buffer.
void ADCIntHandler(void);

// ****************************************************************
// initADC: Initializes Analog to Digital Conversion
void initADC(void);

// ****************************************************************
// calculateMean: Calculate the mean ADC from sensor input
int calculateMean(void);
// @return  Mean value of all contents of circular buffer

// ****************************************************************
// percentageHeight: Calculate the percentage height (0% = Grounded, 100% = Maximum Height)
int percentageHeight(int32_t ground_level, int32_t current);
// @param   ground_level  - The value calculated by calculateMean() when the helicopter is started/turned on.
// @param   current       - The value calculated by calculateMean() at the moment percentageHeight is called.
// @return  percent       - The current height as a percentage of the total/maximum height

void
Trigger_ADC(void *pvParameters);

void
Mean_ADC(void *pvParameters);

#endif /*ADC_H*/
