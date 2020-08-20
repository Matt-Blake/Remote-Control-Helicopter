/* ****************************************************************
 * ADC.c
 *
 * Source file for Analogue to Digital Conversion Module
 * Uses Successive-Approximation Quantiser
 * Includes initialisers, interrupts and calculation functions
 *
 * Based off ADCdemo1.c - P.J. Bones, UCECE, 2018
 *
 * Further based on ADC.c
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

#include "ADC.h"

circBuf_t g_inBuffer;


/*
 * Function:    ADCIntHandler
 * ---------------------------
 * Handles each interrupt caused by the ADC trigger task.
 * Reads data from the ADC channel and writes to the circular
 * buffer.
 * Sets flag when buffer is filled.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
ADCIntHandler(void)
{
    uint32_t ulValue;
    uint32_t ground_flag;

    ground_flag = xEventGroupGetBits(xFoundAltReference);                               // Calculate the current state of the ground flag

    // Check if the ground (0% altitude) value can and should be initalised
    if ((g_inBuffer.windex) == (ADC_BUF_SIZE - 1) && (ground_flag == GROUND_NOT_FOUND)) {
        xEventGroupSetBitsFromISR(xFoundAltReference, GROUND_BUFFER_FULL, pdFALSE);     // Set flag indicating the buffer is full and can now be averaged
        UARTSend("ADC Buffer Full\n\r");
    }

    // Sample the analog waveform and store the value in a circular buffer
    ADCSequenceDataGet(ADC0_BASE, ADC_SEQ_NUM, &ulValue);                         // Runs the A-D Conversion and stores the value in ulValue
    writeCircBuf(&g_inBuffer, ulValue);                                           // Writes the ADC value to the Circular Buffer
    ADCIntClear(ADC0_BASE, ADC_SEQ_NUM);                                          // Clears the interrupt
}


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
void
initADC(void)
{
    SysCtlPeripheralEnable(ADC_PERIPH);                                 // Enables ADC peripheral
    while(!SysCtlPeripheralReady(ADC_PERIPH));

    ADCSequenceConfigure(ADC_BASE, ADC_SEQ_NUM,
                         ADC_TRIGGER_PROCESSOR, ADC_PRIORITY);          // Sets module, sample sequence, trigger, and priority
    ADCSequenceStepConfigure(ADC_BASE, ADC_SEQ_NUM, ADC_STEP,           // Configures the module, sample sequence, step, and channel            // Change to CH9 for heli
                         ADC_CTL_CH9 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC_BASE, ADC_SEQ_NUM);                           // Enables Sequencing on ADC module
    ADCIntRegister(ADC_BASE, ADC_SEQ_NUM, ADCIntHandler);               // Registers the interrupt and sets ADCIntHandler to handle the interrupt
    ADCIntEnable(ADC_BASE, ADC_SEQ_NUM);                                // Enables interrupts on ADC module

    initCircBuf (&g_inBuffer, ADC_BUF_SIZE);
}


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
void
TriggerADC(void *pvParameters)
{
    while(1){
        ADCProcessorTrigger(ADC0_BASE, ADC_SEQ_NUM);                // Trigger an ADC reading

        vTaskDelay(SAMPLING_PERIOD / portTICK_RATE_MS);
    }
}


