/* ****************************************************************
 * ADC.c
 *
 * Module for Analogue to Digital Conversion and Altitude Calculations
 * Uses Successive-Approximation Quantiser
 * Includes initialisers, interrupts and calculation functions
 *
 * Derrick Edward, Grayson Mynott, Ryan Earwaker
 * Thu AM Group 18
 * Last modified:  29.05.2019
 *
 * Based off ADCdemo1.c - P.J. Bones, UCECE, 2018
 * ***************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "utils/ustdlib.h"
#include "driverlib/pin_map.h"
#include "circBufT.h"
#include "ADC.h"
#include "UART.h"
#include "DEFINES.h"

// ************************* GLOBALS *******************************
circBuf_t g_inBuffer;

// ********************** ADC FUNCTIONS ****************************
/* Handles the ADC Interrupts (Occur Every SysTick) */
void ADCIntHandler(void)
{
    uint32_t ulValue;                                                   // Initialise variable to be used to store ADC value

    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);                         // Runs the A-D Conversion and stores the value in ulValue
    writeCircBuf(&g_inBuffer, ulValue);                                 // Writes the ADC value to the Circular Buffer
    ADCIntClear(ADC0_BASE, 3);                                          // Clears the interrupt
}

/* Initialises the ADC module */
void initADC(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);                         // Enables ADC peripheral
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);       // Sets module, sample sequence, trigger, and priority
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0,                           // Configures the module, sample sequence, step, and channel
                             ADC_CTL_CH9 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);                                    // Enables Sequencing on ADC module
    ADCIntRegister(ADC0_BASE, 3, ADCIntHandler);                        // Registers the interrupt and sets ADCIntHandler to handle the interrupt
    ADCIntEnable(ADC0_BASE, 3);                                         // Enables interrupts on ADC module
}

/* Calculates the mean value of the circular buffer */
int calculateMean(void)
{
    uint16_t i;
    int32_t sum = 0;                                                    // Initialise sum

    for (i = 0; i < BUF_SIZE; i++)
        sum = sum + readCircBuf(&g_inBuffer);                           // Sum all values in circBuf

    return (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;                         // Returns mean value
}

/* Calculates the altitude as a percentage of the maximum height */
int percentageHeight(int16_t ground_level, int32_t current)
{
    int16_t vDropADC = 1275;                                            // Voltage drop between ground and maximum height - This value is accurate for the emulator
    int16_t maxHeight = ground_level - vDropADC;                        // ADC value at maximum height
    int8_t percent = 100 - (100 * (current - maxHeight) / (vDropADC));  // Calculates percentage

    return percent;                                                     // Returns percentage value
}
