/* ****************************************************************
 * hookFunctions.h
 *
 * Header file for the hookFunctions module
 * Create FreeRTOS hook functions to monitor stack and CPU usage
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 19/08/2020
 *
 * ***************************************************************/


#ifndef HOOKFUNCTIONS_H_
#define HOOKFUNCTIONS_H_

#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"

#define STAT_BUFFER_SIZE    512     // The UART buffer size used to send CPU load information


/*
 * Function:    vApplicationStackOverflowHook
 * -------------------------------------------
 * Hook for when the stack overflows.
 * Sends error message and enters infinte loop.
 *
 * @params:
 *      - xTask: Task that triggered the stack
 *      overflow.
 * @return:
 *      - NULL
 * ---------------------
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);

/*
 * Function:    vApplicationIdleHook
 * ------------------
 * Idle hook
 * Calculates CPU load for each task
 * This function is run when no other task is running.
 *
 *
 * @params:
 *      - xTask: Task that triggered the stack
 *      overflow.
 * @return:
 *      - NULL
 * ---------------------
 */
void vApplicationIdleHook( void );


#endif /* HOOKFUNCTIONS_H_ */
