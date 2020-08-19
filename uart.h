/* ******************************************************
 *
 * uartDemo.c - Sends information using the serial UART.
 *
 * P.J. Bones UCECE
 * last modified on 16.4.2018
 *
 * uart.h - This code is an adaptation of uartDemo.h,
 *       the code is tailored to send the information
 *       that is required for heli-copter project.
 *
 * Tue am Group 1
 * Creators: Brendain Hennessy   57190084
 *           Sarah Kennelly      76389950
 *           Matt Blake          58979250
 * Last modified: 9/05/2019
 * *****************************************************/

#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "utils/ustdlib.h"
#include "stdio.h"
#include "stdlib.h"
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
char statusStr[MAX_STR_LEN + 1];


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
UARTDisplay (int32_t yaw_degrees, int32_t yaw_desired, int32_t altitude, int32_t altitude_desired, uint32_t duty_main, uint32_t duty_tail, uint8_t g_heliState);

#endif /* UART_H_ */
