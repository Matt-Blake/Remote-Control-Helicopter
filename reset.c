/* ****************************************************************
 * reset.c
 *
 * Source file for reset module
 * Triggers a hard rest interrupt for the Tiva board
 * Includes initialisers, interrupts and calculation functions
 *
 * Based on reset.c
 * Tue AM Group 1
 * Creators: Brendain Hennessy   57190084
 *           Sarah Kennelly      76389950
 *           Matt Blake          58979250
 * Last modified: 9/05/2019
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 19/08/2020
 *
 * ***************************************************************/

#include "reset.h"


/*
 * Function:    resetInterrupt
 * ----------------------------
 * Handler for the reset interrupt.
 * Performs a system reset.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
resetInterrupt(void)
{
    int32_t resetRead = GPIOPinRead(GPIO_PORTB_BASE, RESET_GPIO_PIN);
    UARTSend("RESET/n/r");
    if (resetRead == 0){
        SysCtlReset();
    }
}

/*
 * Function:    initReset
 * -----------------------
 * Initializes the reset interrupt on Port A Pin 6.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
initReset(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // YAW_GPIO_BASE holds the value for Port A base
    GPIOIntRegister(RESET_GPIO_BASE, resetInterrupt);

    // YAW_PIN0_GPIO_PIN, YAW_PIN0_GPIO_PIN have the value for pin 0 and pin 1
    GPIOPinTypeGPIOInput(RESET_GPIO_BASE, RESET_GPIO_PIN);

    GPIOIntTypeSet(RESET_GPIO_BASE, RESET_GPIO_PIN,
    GPIO_BOTH_EDGES);

    GPIOIntEnable(RESET_GPIO_BASE, RESET_GPIO_PIN);
}
