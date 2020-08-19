/*
 * statusLED.h
 *
 *  Created on: 19/08/2020
 *      Author: mattblake
 */

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
