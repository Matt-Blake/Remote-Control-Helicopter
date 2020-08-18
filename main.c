/*
 * TODO:
 *      - Change the gains for 180 degree yaw rotation and alt to 50%
 *      - Refine control
 *      - Code abstraction. (Separate altitude/ADC into two files, yaw/quadDecode, etc.)
 *      - Limit integral error or something.
 *      - Justify task priorities
 * KNOWN CONTROL ISSUES Heli 1:
        -
 * KNOWN CONTROL ISSUES Heli 2:
 *      -
 * KNOWN CONTROL ISSUES Heli 3:
 *      -
 * KNOWN CONTROL ISSUES Heli 4:
 *      -
 */

//******************************************************
// Includes
//******************************************************

// Basic/Board includes
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
//#include "utils/ustdlib.h"
#include "OrbitOLED/OrbitOLEDInterface.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "timers.h"

// Local includes
#include "pwm.h"
#include "reset.h"
#include "yaw.h"
#include "uart.h"
#include "altitude.h"
#include "buttons.h"
#include "pidController.h"
#include "FSM.h"

//******************************************************
// Constants
//******************************************************

// Stack sizes in words, calculated experimentally based on uxTaskGetStackHighWaterMark()
#define LED_STACK_DEPTH         32
#define OLED_STACK_DEPTH        128

#define TASK_STACK_DEPTH        128

// Max priority is 8
#define LED_TASK_PRIORITY       5       // LED task priority
#define OLED_TASK_PRIORITY      5       // OLED priority

//#define TIMER_TASK_PRIORITY     5       // Time module priority
#define STACK_TASK_PRIORITY     5

#define ROW_ZERO                0       // Row zero on the OLED display
#define ROW_ONE                 1       // Row one on the OLED display
#define ROW_TWO                 2       // Row two on the OLED display
#define ROW_THREE               3       // Row three on the OLED display
#define COLUMN_ZERO             0       // Column zero on the OLED display
#define DISPLAY_SIZE            17      // Size of strings for the OLED display

#define DISPLAY_PERIOD          200
#define DBL_BTN_TMR_PERIOD      1000//250
#define YAW_FLIP_TMR_PERIOD     1000//250


//******************************************************
// Globals
//******************************************************
QueueHandle_t xOLEDQueue;

SemaphoreHandle_t xAltMutex;
SemaphoreHandle_t xYawMutex;
SemaphoreHandle_t xUARTMutex;
SemaphoreHandle_t xUpBtnSemaphore;
SemaphoreHandle_t xYawFlipSemaphore;

EventGroupHandle_t xFoundAltReference;
EventGroupHandle_t xFoundYawReference;


//******************************************************
// Tasks
//******************************************************
/*
 * Function:    StatusLED
 * -----------------------
 * FreeRTOS task that periodically toggles the on/off state of
 * a built in LED. Assists in determining if the system has
 * stalled.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
static void
StatusLED(void *pvParameters)
{
    uint8_t state = 0;

    while(1)
    {
        state ^= GPIO_PIN_2;

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, state);

        vTaskDelay(200 / portTICK_RATE_MS);
    }
}


/*
 * Function:    initLED
 * ---------------------
 * Initializes the status LED pin as a GPIO output.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
static void
initLED(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                // Activate internal bus clocking for GPIO port F
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));        // Busy-wait until GPIOF's bus clock is ready
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);         // PF_2 as output
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);    // Doesn't need too much drive strength as the RGB LEDs on the TM4C123 launchpad are switched via N-type transistors
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);               // Off by default
}


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
static void
OLEDDisplay (void *pvParameters)
{
    //char string[DISPLAY_SIZE];  // String of the correct size to be displayed on the OLED screen
    char string[20];  // String of the correct size to be displayed on the OLED screen
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
        xQueuePeek(xAltDesQueue, &des_alt, 10);
        xQueuePeek(xAltMeasQueue, &act_alt, 10);

        xQueuePeek(xYawDesQueue, &des_yaw, 10);
        xQueuePeek(xYawMeasQueue, &act_yaw, 10);

        //xQueuePeek(xMainPWMQueue, &main_PWM, 10);
        //xQueuePeek(xTailPWMQueue, &tail_PWM, 10);
        main_PWM = PWMPulseWidthGet(PWM0_BASE, PWM_OUT_7);
        tail_PWM = PWMPulseWidthGet(PWM1_BASE, PWM_OUT_5);

        xQueuePeek(xFSMQueue, &state, 10);

        //usnprintf(string, sizeof(string), "Alt(%%) %3d|%3d ", des_alt, act_alt);
        //OLEDStringDraw(string, COLUMN_ZERO, ROW_ZERO);
        usnprintf(string, sizeof(string), "Alt(%%) %3d|%3d\n\r", des_alt, act_alt);
        UARTSend(string);

        //usnprintf(string, sizeof(string), "Yaw   %4d|%3d ", des_yaw, act_yaw);
        //OLEDStringDraw(string, COLUMN_ZERO, ROW_ONE);
        usnprintf(string, sizeof(string), "Yaw   %4d|%3d\n\r", des_yaw, act_yaw);
        UARTSend(string);

        //usnprintf(string, sizeof(string), "PWM(%%) %3d|%3d ", main_PWM, tail_PWM);
        //OLEDStringDraw(string, COLUMN_ZERO, ROW_TWO);
        usnprintf(string, sizeof(string), "PWM %3d|%3d\n\r", main_PWM/250, tail_PWM/250);
        UARTSend(string);

        //usnprintf(string, sizeof(string), "%s     ", states[state]);
        //OLEDStringDraw(string, COLUMN_ZERO, ROW_THREE);
        usnprintf(string, sizeof(string), "%s\r\n", states[state]);
        UARTSend(string);

        vTaskDelay(DISPLAY_PERIOD / portTICK_RATE_MS);
    }
}


/*
 * Function:    initClk
 * ---------------------
 * Initializes the main system clock to 80MHz
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
 * Function:    initControllers
 * -----------------------------
 * Initializes the structs used to hold the PWM PID controller
 * gains and errors.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
static void
initControllers(void)
{
    initController(&g_alt_controller, false);   // Create altitude controller based of preset gains
    initController(&g_yaw_controller, true);    // Create yaw controller based of preset gains
}


/*
 * Function:    initSystem
 * ------------------------
 * Calls to all initializing fuctions required to
 * fully initialize the system.
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
    initClk();                  // Initialize the system clock
    initialiseUSB_UART();       // Initialize UART communication over USB
    //initReset();              // Initialize the soft reset of the system
    initLED();                  // Initialize the status LED
    OLEDInitialise();           // Initialize the OLED display
    initBtns();                 // Initialize the GPIO buttons and switches
    initADC();                  // Initialize the Analog-Digital converter
    initQuadrature();           // Initialize the quadrature decoding interrupts
    initReferenceYaw();         // Initialize the reference yaw interrupt
    initControllers();          // Initalaize the PWM duty controllers
    initPWM();                  // Initialize the PWM modules
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
    xTaskCreate(SwitchesCheck,  "Switch Poll",  SWITCH_STACK_DEPTH,     NULL,       SWI_TASK_PRIORITY,      &SwitchCheck);
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
 * initializes the values to zero.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
static void
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
    xQueueOverwrite(xAltBtnQueue, &queue_init);
    xQueueOverwrite(xYawBtnQueue, &queue_init);
    xQueueOverwrite(xAltMeasQueue, &queue_init);
    xQueueOverwrite(xAltDesQueue, &queue_init);
    xQueueOverwrite(xYawMeasQueue, &queue_init);
    xQueueOverwrite(xYawDesQueue, &queue_init);
    xQueueOverwrite(xMainPWMQueue, &queue_init);
    xQueueOverwrite(xTailPWMQueue, &queue_init);
    xQueueOverwrite(xFSMQueue, &queue_init);
    xQueueOverwrite(xYawSlotQueue, &queue_init);
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
    xAltMutex = xSemaphoreCreateMutex();
    xYawMutex = xSemaphoreCreateMutex();
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
                                    / portTICK_RATE_MS, pdFALSE, ( void * ) 0, vBtnTimerCallback );
    xYawFlipTimer   = xTimerCreate( "Yaw Flip Timer", YAW_FLIP_TMR_PERIOD
                                    / portTICK_RATE_MS, pdFALSE, ( void * ) 0, vBtnTimerCallback );
    xLandingTimer   = xTimerCreate( "Land Timer", LAND_TMR_PERIOD
                                    / portTICK_RATE_MS, pdTRUE,  ( void * ) 0, vLandTimerCallback );
    xYawRefTimer   = xTimerCreate( "Yaw Ref Timer", YAW_REF_TMR_PERIOD
                                    / portTICK_RATE_MS, pdFALSE, ( void * ) 0, vYawRefCallback );
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
    UARTSend("OVERFLOW\n");
    UARTSend(pcTaskName);
    while (1){}
}


/*
 * Function:    main
 * ------------------
 * Main program. Calls initializing functions,
 * sends starting message over UART, and then
 * starts FreeRTOS scheduler.
 *
 * @params:
 *      - xTask: Task that triggered the stack
 *      overflow.
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
    createSemaphores();
    createTimers();
    UARTSend("Starting...\n");

    vTaskStartScheduler();
    while(1);
}
//End
