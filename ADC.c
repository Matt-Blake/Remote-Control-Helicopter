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

// ************************* GLOBALS *******************************
//circBuf_t g_inBuffer;

// ** Analog-Digital Conversion ***********************************
//#define BUF_SIZE 20
#define SAMPLE_RATE_HZ 200


// ********************** ADC FUNCTIONS ****************************
/* Handles the ADC Interrupts (Occur Every SysTick) */
/*void
ADCIntHandler (void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32TaskDelay = 100;
    while(1){
        ui16LastTime = xTaskGetTickCount();
        uint32_t ulValue;                                                   // Initialise variable to be used to store ADC value

        ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);                         // Runs the A-D Conversion and stores the value in ulValue
        writeCircBuf(&g_inBuffer, ulValue);                                 // Writes the ADC value to the Circular Buffer
        //UARTSend("ADC sample.\n");
        char cMessage[20];
        usnprintf(cMessage, sizeof(cMessage), "%d\n", ulValue);
        UARTSend(cMessage);
        ADCIntClear(ADC0_BASE, 3); //clear int
        vTaskDelayUntil(&ui16LastTime, ui32TaskDelay / portTICK_RATE_MS);
    }
}
*/
void
ADCIntHandler(void)
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

    for (i = 0; i < BUF_SIZE; i++)
        sum = sum + readCircBuf(&g_inBuffer);                           // Sum all values in circBuf


    return (sum / BUF_SIZE);                         // Returns mean value
}

/* Calculates the altitude as a percentage of the maximum height */
int percentageHeight(int32_t ground_level, int32_t current)
{
    int32_t vDropADC = 1275;                                            // Voltage drop between ground and maximum height - This value is accurate for the emulator
    int32_t maxHeight = ground_level - vDropADC;                        // ADC value at maximum height
    int8_t percent = 100 - (100 * (current - maxHeight) / (vDropADC));  // Calculates percentage

    return percent;                                                     // Returns percentage value
}
