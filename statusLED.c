/* ****************************************************************
 * statusLED.c
 *
 * Source file for the statusLED module
 * Flash the blue LED on the Tiva board to indicate that the program
 * is running.
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 19/08/2020
 *
 * ***************************************************************/

#include "statusLED.h"

/*
 * Function:    StatusLED
 * -----------------------
 * FreeRTOS task that periodically toggles the on/off state of
 * a built in LED. Assists in determining if the system has
 * stalled.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
StatusLED(void *pvParameters)
{
    uint8_t state = 0;

    while(1)
    {
        state ^= GPIO_PIN_2;
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, state);

        vTaskDelay(200 / portTICK_RATE_MS);
    }
}


/*
 * Function:    initLED
 * ---------------------
 * Initializes the status LED pin as a GPIO output.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
initLED(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                // Activate internal bus clocking for GPIO port F
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));        // Busy-wait until GPIOF's bus clock is ready
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);         // PF_2 as output
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);    // Doesn't need too much drive strength as the RGB LEDs on the TM4C123 launchpad are switched via N-type transistors
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);               // Off by default
}


