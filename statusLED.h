/* ****************************************************************
 * statusLED.h
 *
 * Header file for the statusLED module
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

#ifndef STATUSLED_H_
#define STATUSLED_H_

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "FreeRTOS.h"
#include "task.h"


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
void StatusLED(void *pvParameters);

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
void initLED(void);


#endif /* STATUSLED_H_ */
