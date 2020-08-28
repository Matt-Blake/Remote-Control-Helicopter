/* ****************************************************************
 * uart.c
 *
 * Header file of the UART module.
 * Sends serial information over UART to provide user feedback
 *
 * Based off uartDemo.c - P.J. Bones, UCECE, 2018
 *
 * Further based on uart.c
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

#include "uart.h"


/*
 * Function:    initialiseUSB_UART
 * ------------------------
 * Initialises the peripherals and pins used for serial UART
 * communication.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
initialiseUSB_UART (void)
{
    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(UART_USB_PERIPH_UART);
    while(!SysCtlPeripheralReady(UART_USB_PERIPH_UART));
    SysCtlPeripheralEnable(UART_USB_PERIPH_GPIO);
    while(!SysCtlPeripheralReady(UART_USB_PERIPH_GPIO));

    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(UART_USB_GPIO_BASE, UART_USB_GPIO_PINS);
    GPIOPinConfigure (GPIO_PA0_U0RX);
    GPIOPinConfigure (GPIO_PA1_U0TX);

    UARTConfigSetExpClk(UART_USB_BASE, SysCtlClockGet(), BAUD_RATE, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART_USB_BASE);
    UARTEnable(UART_USB_BASE);
}


/*
 * Function:    UARTSend
 * ------------------------
 * Transmits a string over UART protocol via USB connection.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
UARTSend (char *pucBuffer)
{
    if(xSemaphoreTake(xUARTMutex, 0/portTICK_RATE_MS) == pdPASS){ // Mutex used to avoid race conditions with pucBuffer
        // Loop while there are more characters to send.
        while(*pucBuffer)
        {
            // Write the next character to the UART Tx FIFO.
            UARTCharPut(UART_USB_BASE, *pucBuffer);
            pucBuffer++;
        }
        xSemaphoreGive(xUARTMutex); // Give UART mutex so other tasks can access UART
    }
}


/*
 * Function:    UARTDisplay
 * ------------------------
 * Puts current yaw degrees, desired yaw degrees, current altitude,
 * desired altitude and current state into a string form which can
 * be send through the UART to the HeliRig or serial terminal.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
UARTDisplay (void *pvParameters)
{

    int32_t    des_alt;         // Desired altitude
    int32_t    act_alt;         // Actual altitude
    int32_t    des_yaw;         // Desired yaw
    int32_t    act_yaw;         // Actual yaw
    uint32_t   state;           // Current state in the FSM

    char UARTstring[20];        // String to be sent over UART
    char* states[4] = {"Landed", "Take Off", "Flying", "Landing"};
    while(1)
    {
        // Retrieve altitude, yaw and PWM information
        xQueuePeek(xAltDesQueue,  &des_alt, TICKS_TO_WAIT);
        xQueuePeek(xAltMeasQueue, &act_alt, TICKS_TO_WAIT);
        xQueuePeek(xYawDesQueue,  &des_yaw, TICKS_TO_WAIT);
        xQueuePeek(xYawMeasQueue, &act_yaw, TICKS_TO_WAIT);
        xQueuePeek(xFSMQueue,     &state,   TICKS_TO_WAIT);

        // Send information over UART
        UARTSend("------------\n");
        usnprintf(UARTstring, sizeof(UARTstring), "Alt(%%) %3d|%3d\n", des_alt, act_alt);
        UARTSend(UARTstring);
        usnprintf(UARTstring, sizeof(UARTstring), "Yaw   %4d|%3d\n", des_yaw, act_yaw);
        UARTSend(UARTstring);
        usnprintf(UARTstring, sizeof(UARTstring), "%s\n", states[state]);
        UARTSend(UARTstring);
        UARTSend("------------\n");

        vTaskDelay(UART_PERIOD / portTICK_RATE_MS);
    }
}
