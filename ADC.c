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

#include "ADC.h"


// ********************** ADC FUNCTIONS ****************************
/*
 * Handles the ADC Interrupts (Occur Every SysTick)
 */
void
ADCIntHandler(void)
{
    uint32_t ulValue;                                                   // Initialise variable to be used to store ADC value
    uint32_t XD;
    XD = xEventGroupGetBits(xFoundAltReference);
    if ((g_inBuffer.windex) == 19 && (xEventGroupGetBits(xFoundAltReference) == GROUND_NOT_FOUND)) {
        xEventGroupSetBits(xFoundAltReference, GROUND_BUFFER_FULL);      // Set flag indicating the buffer is full and can now be averaged
        UARTSend("Buff_Full\n");
    }
    XD = GROUND_NOT_FOUND;
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);                         // Runs the A-D Conversion and stores the value in ulValue
    writeCircBuf(&g_inBuffer, ulValue);                                 // Writes the ADC value to the Circular Buffer
    ADCIntClear(ADC0_BASE, 3);                                          // Clears the interrupt
}


/* Initialises the ADC module */
void initADC(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);                         // Enables ADC peripheral
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 1);       // Sets module, sample sequence, trigger, and priority
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0,                           // Configures the module, sample sequence, step, and channel            // Change to CH9 for heli
                             ADC_CTL_CH9 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);                                    // Enables Sequencing on ADC module
    ADCIntRegister(ADC0_BASE, 3, ADCIntHandler);                        // Registers the interrupt and sets ADCIntHandler to handle the interrupt
    ADCIntEnable(ADC0_BASE, 3);                                         // Enables interrupts on ADC module

    initCircBuf (&g_inBuffer, BUF_SIZE);
}

/* Calculates the mean value of the circular buffer */
int32_t calculateMean(void)
{
    uint8_t i;
    int32_t sum = 0;                                                    // Initialize sum
    int32_t reading;

    for (i = 0; i < BUF_SIZE; i++){
        reading = readCircBuf(&g_inBuffer);
        sum = sum + reading;
    }// Sum all values in circBuf

    return (sum /BUF_SIZE) ;                       // Returns mean value
}

/* Calculates the altitude as a percentage of the maximum height */
int32_t
percentageHeight(int32_t ground_level, int32_t current)
{
    int32_t maxHeight;
    int32_t percent;

    maxHeight = ground_level - VOLTAGE_DROP_ADC;                        // ADC value at maximum height
    percent = 100 - (100 * (current - maxHeight) / (VOLTAGE_DROP_ADC));  // Calculates percentage

    return percent;                                                     // Returns percentage value
}

/*
 * RTOS task that periodically triggers the ADC interrupt. - Aim to merge the ADC handler into this.
 */
void
Trigger_ADC(void *pvParameters)
{
    while(1){
        ADCProcessorTrigger(ADC0_BASE, 3);
        vTaskDelay(ADC_PERIOD / portTICK_RATE_MS);
    }
}

/*
 * RTOS task that periodically calculates the average value of the circular ADC buffer.
 */
void
Mean_ADC(void *pvParameters)
{
    char cMessage[17];
    int32_t mean;
    int32_t altitude;
    static int32_t ground;
    int32_t XD;

    while(1){

        if (xEventGroupGetBits(xFoundAltReference) == GROUND_BUFFER_FULL) {
            ground = calculateMean();
            xEventGroupSetBits(xFoundAltReference, GROUND_FOUND); // Set flag indicating that the ground reference has been set
            UARTSend("GroundFound\n");
        } else if(xEventGroupGetBits(xFoundAltReference) == GROUND_FOUND){
            mean = calculateMean();
            altitude = percentageHeight(ground, mean);
            usnprintf(cMessage, sizeof(cMessage), "Alt: %d\n", altitude);
            UARTSend(cMessage);
        }
        XD = xEventGroupGetBits(xFoundAltReference);
        xQueueOverwrite(xAltMeasQueue, &altitude);
        XD = GROUND_FOUND;
        vTaskDelay(ALTITUDE_PERIOD / portTICK_RATE_MS);
    }
}
