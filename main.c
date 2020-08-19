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


/*
 * TODO:
 *      - Refine control
 *      - Work out why the 180 turn fucks out when pressed twice
 *      - Finish all comments throughout project.
 */

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
#include "pwm.h"
#include "reset.h"
#include "yaw.h"
#include "uart.h"
#include "altitude.h"
#include "buttons.h"
#include "pidController.h"
#include "FSM.h"

// Task stack sizes in words, calculated experimentally based on uxTaskGetStackHighWaterMark()
#define LED_STACK_DEPTH         32
#define OLED_STACK_DEPTH        128
#define BTN_STACK_DEPTH         64
#define SWITCH_STACK_DEPTH      64
#define ADC_STACK_DEPTH         32
#define MEAN_STACK_DEPTH        64
#define MAIN_PWM_STACK_DEPTH    128
#define TAIL_PWM_STACK_DEPTH    128
#define FSM_STACK_DEPTH         128

// Task priorities. Max priority is 8
#define LED_TASK_PRIORITY       4
#define OLED_TASK_PRIORITY      4
#define BTN_TASK_PRIORITY       5
#define SWI_TASK_PRIORITY       5
#define ADC_TASK_PRIORITY       8
#define MEAN_TASK_PRIORITY      7
#define MAIN_PWM_TASK_PRIORITY  6
#define TAIL_PWM_TASK_PRIORITY  6
#define FSM_TASK_PRIORITY       5

// Timer periods
#define DBL_BTN_TMR_PERIOD      1000
#define YAW_FLIP_TMR_PERIOD     1000

EventGroupHandle_t xFoundAltReference;
EventGroupHandle_t xFoundYawReference;

QueueHandle_t xOLEDQueue;

SemaphoreHandle_t xUARTMutex;


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
 * Function:    createTasks
 * -------------------------
 * Creates all FreeRTOS tasks used in the system.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
static void
createTasks(void)
{
    xTaskCreate(StatusLED,      "LED Task",     LED_STACK_DEPTH,        NULL,       LED_TASK_PRIORITY,      &StatLED);
    xTaskCreate(OLEDDisplay,    "OLED Task",    OLED_STACK_DEPTH,       NULL,       OLED_TASK_PRIORITY,     &OLEDDisp);
    xTaskCreate(ButtonsCheck,   "Btn Poll",     BTN_STACK_DEPTH,        NULL,       BTN_TASK_PRIORITY,      &BtnCheck);
    xTaskCreate(SwitchesCheck,  "Switch Poll",  SWITCH_STACK_DEPTH,     NULL,       SWI_TASK_PRIORITY,      &SwiCheck);
    xTaskCreate(TriggerADC,     "ADC Handler",  ADC_STACK_DEPTH,        NULL,       ADC_TASK_PRIORITY,      &ADCTrig);
    xTaskCreate(MeanADC,        "ADC Mean",     MEAN_STACK_DEPTH,       NULL,       MEAN_TASK_PRIORITY,     &ADCMean);
    xTaskCreate(SetMainDuty,    "Main PWM",     MAIN_PWM_STACK_DEPTH,   NULL,       MAIN_PWM_TASK_PRIORITY, &MainPWM);
    xTaskCreate(SetTailDuty,    "Tail PWM",     TAIL_PWM_STACK_DEPTH,   NULL,       TAIL_PWM_TASK_PRIORITY, &TailPWM);
    xTaskCreate(FSM,            "FSM",          FSM_STACK_DEPTH,        NULL,       FSM_TASK_PRIORITY,      &FSMTask);
}


/*
 * Function:    createQueues
 * -------------------------
 * Creates all FreeRTOS queues used in the system and
 * Initialises the values to zero.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
createQueues(void)
{
    int32_t queue_init = 0; // Value used to initalise queues

    // Create queues
    xOLEDQueue      = xQueueCreate(1, sizeof( uint32_t ) );
    xAltBtnQueue    = xQueueCreate(1, sizeof( uint32_t ) );
    xYawBtnQueue    = xQueueCreate(1, sizeof( uint32_t ) );
    xAltMeasQueue   = xQueueCreate(1, sizeof( int32_t ) );
    xAltDesQueue    = xQueueCreate(2, sizeof( int32_t ) );
    xYawMeasQueue   = xQueueCreate(1, sizeof( int32_t ) );
    xYawDesQueue    = xQueueCreate(1, sizeof( int32_t ) );
    xMainPWMQueue   = xQueueCreate(1, sizeof( int32_t ) );
    xTailPWMQueue   = xQueueCreate(1, sizeof( int32_t ) );
    xFSMQueue       = xQueueCreate(1, sizeof( int32_t ) );
    xYawSlotQueue   = xQueueCreate(1, sizeof( int32_t ) );

    // Initalise queues
    xQueueOverwrite(xAltBtnQueue,        &queue_init);
    xQueueOverwrite(xYawBtnQueue,        &queue_init);
    xQueueOverwrite(xAltMeasQueue,       &queue_init);
    xQueueOverwrite(xAltDesQueue,        &queue_init);
    xQueueOverwrite(xYawMeasQueue,       &queue_init);
    xQueueOverwrite(xYawDesQueue,        &queue_init);
    xQueueOverwrite(xMainPWMQueue,       &queue_init);
    xQueueOverwrite(xTailPWMQueue,       &queue_init);
    xQueueOverwrite(xFSMQueue,           &queue_init);
    xQueueOverwrite(xYawSlotQueue,       &queue_init);
}


/*
 * Function:    createSemaphores
 * -------------------------
 * Creates all FreeRTOS semaphores, mutexes and event groups
 * used in the system.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
static void
createSemaphores(void)
{
    int event_init = 0; //  Value used to initalise event groups

    // Create mutexs to avoid race conditions
    xUARTMutex =  xSemaphoreCreateMutex();

    xUpBtnSemaphore = xSemaphoreCreateCounting(2, 0);
    xYawFlipSemaphore = xSemaphoreCreateCounting(2, 0);

    // Create event groups to act as flags
    xFoundAltReference = xEventGroupCreate();
    xFoundYawReference = xEventGroupCreate();

    // Initalise event groups to zero
    xEventGroupSetBits(xFoundAltReference, event_init);
    xEventGroupSetBits(xFoundYawReference, event_init);
}


/*
 * Function:    createTimers
 * --------------------------
 * Creates all FreeRTOS timers used in the system.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
static void
createTimers(void)
{
    xUpBtnTimer     = xTimerCreate( "Button Timer", DBL_BTN_TMR_PERIOD
                                    / portTICK_RATE_MS, pdFALSE, ( void * ) 0, vDblBtnTimerCallback );
    xYawFlipTimer   = xTimerCreate( "Yaw Flip Timer", YAW_FLIP_TMR_PERIOD
                                    / portTICK_RATE_MS, pdFALSE, ( void * ) 0, vDblBtnTimerCallback );
    xLandingTimer   = xTimerCreate( "Land Timer", LAND_TMR_PERIOD
                                    / portTICK_RATE_MS, pdTRUE,  ( void * ) 0, vLandTimerCallback );
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
