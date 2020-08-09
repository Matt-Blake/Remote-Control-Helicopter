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

/*
 * DEFINES
 */
#define ADC_PERIOD          80
#define ALTITUDE_PERIOD     200
#define BUF_SIZE            20
#define SAMPLE_RATE_HZ      200
#define VOLTAGE_DROP_ADC    1241    // Voltage drop between ground and maximum height

/*
 * GLOAL VARIABLES
 */
extern QueueHandle_t xAltMeasQueue;
//extern int8_t groundFound;

/*
 * PROTOTYPES
 */

/*
 * ADCIntHandler: The handler for the ADC interrupt.
 * Reads value from PE4 and writes to the circular buffer.
 */
void ADCIntHandler(void);

/*
 * initADC: Initializes Analog to Digital Conversion on Pin PE4.
 */
void initADC(void);

/*
 * calculateMean: Calculate the mean value of the circular buffer.
 * @return  Mean value of all contents of circular buffer
 */
int calculateMean(void);

/*
 * percentageHeight: Calculate the percentage height (0% = Grounded, 100% = Maximum Height)
 * @param   ground_level  - The value calculated by calculateMean() when the helicopter is started/turned on.
 * @param   current       - The value calculated by calculateMean() at the moment percentageHeight is called.
 * @return  percent       - The current height as a percentage of the total/maximum height
 */
int percentageHeight(int32_t ground_level, int32_t current);

/*
 * Trigger_ADC: RTOS task that periodically triggers the ADC interrupt.
 */
void Trigger_ADC(void *pvParameters);

/*
 * Mean_ADC: RTOS task that periodically calls the calculateMean function.
 */
void Mean_ADC(void *pvParameters);

#endif /*ADC_H*/
