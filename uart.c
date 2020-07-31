//*****************************************************
//
// uartDemo.c - Sends information using the serial UART.
//
// P.J. Bones UCECE
// last modified on 16.4.2018
//
// uart.c - This code is an adaptation of uartDemo.h,
//       the code is tailored to send the information
//       that is required for helicopter project.
//
// Tue am Group 1
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          58979250
// Last modified: 9/05/2019
//
// ENCE464 Assignment 1 Group 2
// Creators: Grayson Mynott      56353855
//           Ryan Earwaker       12832870
//           Matt Blake          58979250
// Last modified: 31/07/2020
//******************************************************

#include "uart.h"

//********************************************************
// Constants
//********************************************************
#define MAX_STR_LEN 32

//---USB Serial comms: UART0, Rx:PA0 , Tx:PA1
#define BAUD_RATE 9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX

//********************************************************
// Global variables
//********************************************************
char statusStr[MAX_STR_LEN + 1];
volatile uint8_t slowTick = false;

//********************************************************
// initialiseUSB_UART - 8 bits, 1 stop bit, no parity
//********************************************************
void
initialiseUSB_UART (void)
{
    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(UART_USB_PERIPH_UART);
    SysCtlPeripheralEnable(UART_USB_PERIPH_GPIO);

    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(UART_USB_GPIO_BASE, UART_USB_GPIO_PINS);
    GPIOPinConfigure (GPIO_PA0_U0RX);
    GPIOPinConfigure (GPIO_PA1_U0TX);

    UARTConfigSetExpClk(UART_USB_BASE, SysCtlClockGet(), BAUD_RATE, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART_USB_BASE);
    UARTEnable(UART_USB_BASE);
}

//**********************************************************************
// Transmit a string via UART0
//**********************************************************************
void
UARTSend (char *pucBuffer)
{
    // Loop while there are more characters to send.
    while(*pucBuffer)
    {
        // Write the next character to the UART Tx FIFO.
        UARTCharPut(UART_USB_BASE, *pucBuffer);
        pucBuffer++;
    }
}

//********************************************************
// Puts current yaw degrees, desired yaw degrees, current altitude,
// desired altitude, main duty cycle, tail duty cycle and current state
// into a string form which can be send through the UART to the heli rig or
// serial terminal.
//********************************************************
void
UARTDisplay (int32_t yaw_degrees, int32_t yaw_desired, int32_t altitude, int32_t altitude_desired, uint32_t duty_main, uint32_t duty_tail, uint8_t g_heliState)
{
    usprintf (statusStr, "Yaw: %d\r\n", yaw_degrees);
    UARTSend (statusStr);

    usprintf (statusStr, "Desired Yaw: %d\r\n", yaw_desired);
    UARTSend (statusStr);

    usprintf (statusStr, "Altitude: %d%%\r\n", altitude);
    UARTSend (statusStr);

    usprintf (statusStr, "Desired Altitude: %d%%\r\n", altitude_desired);
    UARTSend (statusStr);

    usprintf (statusStr, "Duty cycle main: %d%%\r\n", duty_main);
    UARTSend (statusStr);

    usprintf (statusStr, "Duty cycle tail: %d%%\r\n", duty_tail);
    UARTSend (statusStr);

    if (g_heliState == 0){
        usprintf(statusStr, "Mode: Landed");
    } else if (g_heliState == 1){
        usprintf(statusStr, "Mode: Take Off");
    } else if (g_heliState == 2){
        usprintf(statusStr, "Mode: Flying");
    } else {
        usprintf(statusStr, "Mode: Landing");
    }
    UARTSend (statusStr);

    usprintf (statusStr, "\r\n");
    UARTSend (statusStr);
}
