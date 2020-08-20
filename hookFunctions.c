/* ****************************************************************
 * hookFunctions.c
 *
 * Source file for the hookFunctions module
 * Create FreeRTOS hook functions to monitor stack and CPU usage
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 19/08/2020
 *
 * ***************************************************************/

#include "hookFunctions.h"


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
void
vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    UARTSend("STACK OVERFLOW\n");
    UARTSend(pcTaskName);
    while (1){}
}


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
void
vApplicationIdleHook( void )
{
    static char runtime_stats_buffer[STAT_BUFFER_SIZE];

    vTaskGetRunTimeStats(runtime_stats_buffer); // Calculate CPU load stats
    UARTSend(runtime_stats_buffer); // Print CPU load stats to UART
}
