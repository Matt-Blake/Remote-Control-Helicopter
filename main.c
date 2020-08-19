/* ****************************************************************
 * main.c
 *
 * Main source file for ENCE464 Assignment 1 - HeliRig Project
 * This program facilitates the stable flight of a remote
 * control helicopter attached to a HeliRig.
 * This program is designed for the Texas Instruments Tiva Board
 * and the Orbit Boosterpack.
 *
 * ENCE464 Assig nment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 19/08/2020
 *
 * ***************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include "driverlib/interrupt.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "timers.h"
#include "statusLED.h"
#include "OLED.h"
#include "FreeRTOSCreate.h"
#include "pwm.h"
#include "reset.h"
#include "yaw.h"
#include "uart.h"
#include "altitude.h"
#include "buttons.h"
#include "pidController.h"
#include "FSM.h"


/*
 * Function:    initClk
 * ---------------------
 * Initialises the main system clock to 80MHz
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
static void
initClk(void)
{
    // Initialise clock frequency to 80MHz
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
}

/*
 * Function:    initSystem
 * ------------------------
 * Calls to all Initialising fuctions required to
 * fully Initialise the system.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
static void
initSystem(void)
{
    IntMasterDisable();         // Disable system interrupts while the program is initializing.
    initClk();                  // Initialise the system clock
    initialiseUSB_UART();       // Initialise UART communication over USB
    //initReset();              // Initialise the hard reset of the system
    initLED();                  // Initialise the status LED
    OLEDInitialise();           // Initialise the OLED display
    initBtns();                 // Initialise the GPIO buttons and switches
    initADC();                  // Initialise the Analog-Digital converter
    initQuadrature();           // Initialise the quadrature decoding interrupts
    initReferenceYaw();         // Initialise the reference yaw interrupt
    initPWM();                  // Initialise the PWM modules
    IntMasterEnable();          // Re-enable system interrupts

}


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
    //static char runtime_stats_buffer[512];

    //vTaskGetRunTimeStats(runtime_stats_buffer); // Calculate CPU load stats
    //UARTSend(runtime_stats_buffer); // Print CPU load stats to UART
}

/*
 * Function:    main
 * ------------------
 * Main program. Calls initializing functions,
 * sends starting message over UART, and then
 * starts FreeRTOS scheduler.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
int
main(void)
 {
    initSystem();
    createTasks();
    createQueues();
    initControllers();          // Initalaize the PWM duty controllers
    createSemaphores();
    createTimers();
    UARTSend("Starting...\n");

    vTaskStartScheduler();
    while(1);
}
//End
