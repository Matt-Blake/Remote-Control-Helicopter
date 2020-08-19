/* ****************************************************************
 * uart.h
 *
 * Header file of the UART module.
 * Sends serial information over UART to provide user feedback
 *
 * Based off uartDemo.h - P.J. Bones, UCECE, 2018
 *
 * Further based on uart.h
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

#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "utils/ustdlib.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define MAX_STR_LEN             32
#define BAUD_RATE               9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX

extern SemaphoreHandle_t xUARTMutex;
extern QueueHandle_t xAltMeasQueue;
extern QueueHandle_t xAltDesQueue;
extern QueueHandle_t xYawMeasQueue;
extern QueueHandle_t xYawDesQueue;
extern QueueHandle_t xFSMQueue;


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
initialiseUSB_UART (void);

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
UARTSend (char *pucBuffer);

/*
 * Function:    UARTDisplay
 * ------------------------
 * Puts current yaw degrees, desired yaw degrees, current altitude,
 * desired altitude, main duty cycle, tail duty cycle and current
 * state into a string form which can be send through the UART to
 * the helirig or serial terminal.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
UARTDisplay (void *pvParameters);

#endif /* UART_H_ */
