/* ****************************************************************
 * OLED.c
 *
 * Source file for the OLED module
 * Print's to the Orbit Boosterpack's OLED display to indicate the
 * helicopter program's status.
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 19/08/2020
 *
 * ***************************************************************/

#include "OLED.h"


/*
 * Function:    OLEDDisplay
 * -------------------------
 * FreeRTOS task that periodically displays flight
 * information on the Orbit BoosterPack OLED display.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
OLEDDisplay (void *pvParameters)
{
    char string[DISPLAY_SIZE];  // String of the correct size to be displayed on the OLED screen
    char UARTstring[20];            // String to be sent over UART
    int32_t    des_alt;         // Desired altitude
    int32_t    act_alt;         // Actual altitude
    int32_t    des_yaw;         // Desired yaw
    int32_t    act_yaw;         // Actual yaw
    uint32_t   main_PWM;        // Current main duty cycle
    uint32_t   tail_PWM;        // Current tail duty cycle
    uint32_t   state;           // Current state in the FSM

    char* states[4] = {"Landed", "Take Off", "Flying", "Landing"};

    while(1)
    {
        // Retrieve altitude, yaw and PWM information
        xQueuePeek(xAltDesQueue, &des_alt, 10);
        xQueuePeek(xAltMeasQueue, &act_alt, 10);
        xQueuePeek(xYawDesQueue, &des_yaw, 10);
        xQueuePeek(xYawMeasQueue, &act_yaw, 10);
        main_PWM = PWMPulseWidthGet(PWM0_BASE, PWM_OUT_7);
        tail_PWM = PWMPulseWidthGet(PWM1_BASE, PWM_OUT_5);

        xQueuePeek(xFSMQueue, &state, 10);
        UARTSend("------------\n");
        usnprintf(string, sizeof(string), "Alt(%%) %3d|%3d ", des_alt, act_alt);
        OLEDStringDraw(string, COLUMN_ZERO, ROW_ZERO);
        usnprintf(UARTstring, sizeof(UARTstring), "Alt(%%) %3d|%3d\n", des_alt, act_alt);
        UARTSend(UARTstring);

        usnprintf(string, sizeof(string), "Yaw   %4d|%3d ", des_yaw, act_yaw);
        OLEDStringDraw(string, COLUMN_ZERO, ROW_ONE);
        usnprintf(UARTstring, sizeof(UARTstring), "Yaw   %4d|%3d\n", des_yaw, act_yaw);
        UARTSend(UARTstring);

        usnprintf(string, sizeof(string), "PWM(%%) %3d|%3d ", main_PWM, tail_PWM);
        OLEDStringDraw(string, COLUMN_ZERO, ROW_TWO);
        usnprintf(UARTstring, sizeof(UARTstring), "PWM %3d|%3d\n", main_PWM/250, tail_PWM/250);
        UARTSend(UARTstring);

        usnprintf(string, sizeof(string), "%s     ", states[state]);
        OLEDStringDraw(string, COLUMN_ZERO, ROW_THREE);
        usnprintf(UARTstring, sizeof(UARTstring), "%s\n", states[state]);
        UARTSend(UARTstring);
        UARTSend("------------\n");

        vTaskDelay(DISPLAY_PERIOD / portTICK_RATE_MS);
    }
}
