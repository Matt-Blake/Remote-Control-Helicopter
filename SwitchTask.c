/*
 * SwitchTask.c
 *
 *  Created on: 30/07/2020
 *      Author: grayson
 */
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

#include "OrbitOLED/OrbitOLEDInterface.h"

#include "FreeRTOS.h"
#include "queue.h"

#include "buttons.h"
#include "inc/hw_gpio.h"
#include "SwitchTask.h"

uint32_t
SwitchTaskInit(void)
{
    // Unlock the GPIO LOCK register for Right button to work.
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0xFF;

    //
    // Initialize the buttons
    //
    ButtonsInit();

    return(1);
}

void
SwitchTask(void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32SwitchDelay = 25;
    uint8_t ui8CurButtonState, ui8PrevButtonState;
    uint16_t led_blink_rate = 500;

    ui8CurButtonState = ui8PrevButtonState = 0;

    // Get the current tick count.
    ui16LastTime = xTaskGetTickCount();

    // Loop forever.
    while(1)
    {
        // Poll the debounced state of the buttons.
        ui8CurButtonState = ButtonsPoll(0, 0);

        // Check if previous debounced state is equal to the current state.
        if(ui8CurButtonState != ui8PrevButtonState)
        {
            ui8PrevButtonState = ui8CurButtonState;

            // Check to make sure the change in state is due to button press and not due to button release.
            if((ui8CurButtonState & ALL_BUTTONS) != 0)
            {
                if((ui8CurButtonState & ALL_BUTTONS) == LEFT_BUTTON)
                {
                    if (led_blink_rate < 1000) {
                        led_blink_rate += 100;
                    }
                }
                else if((ui8CurButtonState & ALL_BUTTONS) == RIGHT_BUTTON)
                {
                    if (led_blink_rate > 0) {
                        led_blink_rate -= 100;
                    }
                }

                // Pass the value of the button pressed to LEDTask.
                if(xQueueSend(xButtonQueue, &led_blink_rate, portMAX_DELAY) != pdPASS) {
                    // Error. The queue should never be full. If so print the error message on UART and wait for ever.
                    while(1){}
                }
            }
        }

        // Wait for the required amount of time to check back.
        vTaskDelayUntil(&ui16LastTime, ui32SwitchDelay / portTICK_RATE_MS);
    }
}
