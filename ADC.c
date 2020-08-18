/*
 * ADC.c
 *
 * Module for Analogue to Digital Conversion and Altitude
 * Calculations.
 * Uses Successive-Approximation Quantiser
 * Includes initialisers, interrupts and calculation functions
 *
 * Derrick Edward, Grayson Mynott, Ryan Earwaker
 * Thu AM Group 18
 * Last modified:  29.05.2019
 *
 * Based off ADCdemo1.c - P.J. Bones, UCECE, 2018
 */

#include "ADC.h"


#define SEQ_NUM             3
#define ADC_PERIPH          SYSCTL_PERIPH_ADC0
#define ADC_BASE            ADC0_BASE
#define ADC_PRIORITY        1
#define ADC_CHANNEL         ADC_CTL_CH9

#define ADC_PERIOD          10 // 100Hz

circBuf_t g_inBuffer;
TaskHandle_t ADCTrig;

/*********************** ADC FUNCTIONS ****************************/
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
    uint32_t ulValue;                                                   // Initialize variable to be used to store ADC value
    uint32_t ground_flag;

    ground_flag = xEventGroupGetBits(xFoundAltReference);               // Calculate the current state of the ground flag

    if ((g_inBuffer.windex) == (ADC_BUF_SIZE-1) && (ground_flag == GROUND_NOT_FOUND)) {
        xEventGroupSetBitsFromISR(xFoundAltReference, GROUND_BUFFER_FULL, pdFALSE);     // Set flag indicating the buffer is full and can now be averaged
        UARTSend("ADC Buffer Full\n\r");
    }
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);                         // Runs the A-D Conversion and stores the value in ulValue
    writeCircBuf(&g_inBuffer, ulValue);                                 // Writes the ADC value to the Circular Buffer
    ADCIntClear(ADC0_BASE, 3);                                          // Clears the interrupt
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

    ADCSequenceConfigure(ADC_BASE, SEQ_NUM, ADC_TRIGGER_PROCESSOR,
                         ADC_PRIORITY);                                 // Sets module, sample sequence, trigger, and priority
    ADCSequenceStepConfigure(ADC_BASE, SEQ_NUM, 0,                      // Configures the module, sample sequence, step, and channel            // Change to CH9 for heli
                         ADC_CTL_CH9 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC_BASE, SEQ_NUM);                               // Enables Sequencing on ADC module
    ADCIntRegister(ADC_BASE, SEQ_NUM, ADCIntHandler);                   // Registers the interrupt and sets ADCIntHandler to handle the interrupt
    ADCIntEnable(ADC_BASE, SEQ_NUM);                                    // Enables interrupts on ADC module

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
        ADCProcessorTrigger(ADC0_BASE, 3);

        vTaskDelay(ADC_PERIOD / portTICK_RATE_MS);
    }
}


