/* ****************************************************************
 * altitude.c
 *
 * Source file for the altitude module
 * Calculates the altitude of the helicopter based on buffer values
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 19/08/2020
 *
 * ***************************************************************/

#include "altitude.h"


/*
 * Function:    calculateMean
 * ---------------------------
 * Sums up all values in the circular buffer and then divides by the
 * number of elements in order to calculate the average value of
 * the circular buffer.
 *
 * @params:
 *      - NULL
 * @return:
 *      - int32_t mean: The average value of the circular
 *      buffer.
 * ---------------------
 */
int32_t calculateMean(void)
{
    uint8_t i;
    int32_t sum = 0;        // Initialize sum
    int32_t reading;
    int32_t mean = 0;       // Initialize mean

    for (i = 0; i < ADC_BUF_SIZE; i++){
        reading = readCircBuf(&g_inBuffer);
        sum = sum + reading;
    }// Sum all values in circBuf

    mean = (sum/ADC_BUF_SIZE);
    return mean;                       // Returns mean value
}


/*
 * Function:    percentageHeight
 * ------------------------------
 * Converts the average value of the circular buffer to a percentage
 * value of the maximum height.
 *
 * @params:
 *      - int32_t groundLevel: The ADC value when the helicopter is
 *      grounded. Used as a reference to 0% height.
 *      - int32_t currentValue: The current circular buffer average
 *      to be converted to a percentage height.
 * @return:
 *      - int32_t percent: The current circular buffer average as a
 *      percentage of the total height range.
 * ---------------------
 */
int32_t
percentageHeight(int32_t groundLevel, int32_t currentValue)
{
    //char cMessage[17];
    int32_t maxHeight = 0;
    int32_t percent = 0;

    maxHeight = groundLevel - VOLTAGE_DROP_ADC;                             // ADC value at maximum height
    percent = HUNDRED_PERCENT - HUNDRED_PERCENT *
            (currentValue - maxHeight)/(VOLTAGE_DROP_ADC);                  // Calculates percentage altitude

    return percent;                                                         // Returns percentage value
}


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
void
MeanADC(void *pvParameters)
{
    char cMessage[17];
    int32_t mean;
    int32_t altitude = 0;
    static int32_t ground;
    int32_t ground_flag;

    while(1){
        ground_flag = xEventGroupGetBits(xFoundAltReference); // Retrieve the current state of the ground reference
        if (ground_flag == GROUND_BUFFER_FULL) {
            ground = calculateMean();
            xEventGroupClearBits(xFoundAltReference, GROUND_BUFFER_FULL); // Clear previous flag
            xEventGroupSetBits(xFoundAltReference, GROUND_FOUND); // Set flag indicating that the ground reference has been set
            usnprintf(cMessage, sizeof(cMessage), "Ground Found: %d\n\r", ground);
            UARTSend(cMessage);
        } else if (ground_flag == GROUND_FOUND) {
            mean = calculateMean();
            altitude = percentageHeight(ground, mean);
        }
        xQueueOverwrite(xAltMeasQueue, &altitude);

        vTaskDelay(ALTITUDE_PERIOD / portTICK_RATE_MS);
    }
}
