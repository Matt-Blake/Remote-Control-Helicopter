/*
 * TODO:
 *      - Figure out what the stack sizes should be
 *      - Suss interrupts
 *      - double click to do 180
 *      - refine control
 *      - Make it so that tail PWM duty doesn't go negative.
 *      - For some reason the main duty sometimes drops to 2..
 */

//******************************************************
// Includes
//******************************************************

// Basic/Board includes
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "utils/ustdlib.h"
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
#include "ADC.h"
#include "buttons.h"
#include "pidController.h"
#include "FSM.h"

//******************************************************
// Constants
//******************************************************

// Stack sizes in words, calculated experimentally using uxTaskGetStackHighWaterMark()
#define LED_STACK_DEPTH         32
#define OLED_STACK_DEPTH        128
#define BTN_STACK_DEPTH         128
#define SWITCH_STACK_DEPTH      128
#define ADC_STACK_DEPTH         128
#define ALT_STACK_DEPTH         128
#define YAW_STACK_DEPTH         128
#define TASK_STACK_DEPTH        128

// Max priority is 8
#define LED_TASK_PRIORITY       5       // LED task priority
#define OLED_TASK_PRIORITY      5       // OLED priority
#define BTN_TASK_PRIORITY       6       // Button polling task priority
#define SWI_TASK_PRIORITY       6       // Switch polling task priority
#define ADC_TASK_PRIORITY       7       // ADC sampling priority
#define ALT_TASK_PRIORITY       8       // Altitude PWM priority
#define YAW_TASK_PRIORITY       8       // Yaw PWM priority
#define FSM_TASK_PRIORITY       8       // FSM priority
#define TIMER_TASK_PRIORITY     5       // Time module priority
#define STACK_TASK_PRIORITY     3

#define ROW_ZERO                0       // Row zero on the OLED display
#define ROW_ONE                 1       // Row one on the OLED display
#define ROW_TWO                 2       // Row two on the OLED display
#define ROW_THREE               3       // Row three on the OLED display
#define COLUMN_ZERO             0       // Column zero on the OLED display
#define DISPLAY_SIZE            17      // Size of strings for the OLED display

#define DISPLAY_PERIOD          200

#define DBL_BTN_TMR_PERIOD      250
#define LAND_TMR_PERIOD         1000


//******************************************************
// Globals
//******************************************************
QueueHandle_t xOLEDQueue;
QueueHandle_t xAltBtnQueue;
QueueHandle_t xYawBtnQueue;
QueueHandle_t xModeQueue;
QueueHandle_t xAltMeasQueue;
QueueHandle_t xAltDesQueue;
QueueHandle_t xYawMeasQueue;
QueueHandle_t xYawDesQueue;
QueueHandle_t xMainPWMQueue;
QueueHandle_t xTailPWMQueue;
QueueHandle_t xFSMQueue;
QueueHandle_t xYawSlotQueue;

TimerHandle_t xUpBtnTimer;
TimerHandle_t xLandingTimer;

TaskHandle_t Blinky;
TaskHandle_t OLEDDisp;
TaskHandle_t BtnCheck;
TaskHandle_t SwitchCheck;
TaskHandle_t ADCTrig;
TaskHandle_t ADCMean;
TaskHandle_t MainPWM;
TaskHandle_t TailPWM;
TaskHandle_t FSMTask;

SemaphoreHandle_t xAltMutex;
SemaphoreHandle_t xYawMutex;
SemaphoreHandle_t xUpBtnSemaphore;

EventGroupHandle_t xFoundAltReference;
EventGroupHandle_t xFoundYawReference;


//******************************************************
// Tasks
//******************************************************
/*
 * RTOS task that toggles LED state based off button presses
 */
static void
BlinkLED(void *pvParameters)
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
 * Initialise the LED pin and peripherals.
 */
void
initLED(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                // Activate internal bus clocking for GPIO port F
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));        // Busy-wait until GPIOF's bus clock is ready
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);         // PF_2 as output
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);    // Doesn't need too much drive strength as the RGB LEDs on the TM4C123 launchpad are switched via N-type transistors
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);               // Off by default
}


/*
 * Calculate the maximum stack usage all of the RTOS tasks.
 */
void
GetStackUsage(void *pvParameters)
{
    char cMessage[17];
    uint32_t Blinky_stack;
    uint32_t OLEDDisp_stack;
    uint32_t BtnCheck_stack;
    uint32_t SwitchCheck_stack;
    uint32_t ADCTrig_stack;
    uint32_t ADCMean_stack;
    uint32_t MainPWM_stack;
    uint32_t TailPWM_stack;
    uint32_t FSMTask_stack;

    while(1){

        // Retrieve stack usage information from each task
        Blinky_stack      = uxTaskGetStackHighWaterMark(Blinky);
        OLEDDisp_stack    = uxTaskGetStackHighWaterMark(OLEDDisp);
        BtnCheck_stack    = uxTaskGetStackHighWaterMark(BtnCheck);
        SwitchCheck_stack = uxTaskGetStackHighWaterMark(SwitchCheck);
        ADCTrig_stack     = uxTaskGetStackHighWaterMark(ADCTrig);
        ADCMean_stack     = uxTaskGetStackHighWaterMark(ADCMean);
        MainPWM_stack     = uxTaskGetStackHighWaterMark(MainPWM);
        TailPWM_stack     = uxTaskGetStackHighWaterMark(TailPWM);
        FSMTask_stack     = uxTaskGetStackHighWaterMark(FSMTask);

        // Send stack information via UART
        usnprintf(cMessage, sizeof(cMessage), "Blinky unused: %d words\n",      Blinky_stack);
        usnprintf(cMessage, sizeof(cMessage), "OLEDDisp unused: %d words\n",    OLEDDisp_stack);
        usnprintf(cMessage, sizeof(cMessage), "BtnCheck Unused: %d words\n",    BtnCheck_stack);
        usnprintf(cMessage, sizeof(cMessage), "SwitchCheck Unused: %d words\n", SwitchCheck_stack);
        usnprintf(cMessage, sizeof(cMessage), "ADCTrig Unused: %d words\n",     ADCTrig_stack);
        usnprintf(cMessage, sizeof(cMessage), "ADCMean Unused: %d words\n",     ADCMean_stack);
        usnprintf(cMessage, sizeof(cMessage), "MainPWM Unused: %d words\n",     MainPWM_stack);
        usnprintf(cMessage, sizeof(cMessage), "TailPWM Unused: %d words\n",     TailPWM_stack);
        usnprintf(cMessage, sizeof(cMessage), "FSMTask Unused: %d words\n",     FSMTask_stack);
    }
}

/*
 * WRITE DESCRIPTION
 */
void vBtnTimerCallback( TimerHandle_t xTimer )
{
    uint32_t ulCount;

    /* Optionally do something if the pxTimer parameter is NULL. */
    configASSERT( xTimer );

    /* The number of times this timer has expired is saved as the
    timer's ID.  Obtain the count. */
    ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );

    /* Increment the count, then test to see if the timer has expired
    ulMaxExpiryCountBeforeStopping yet. */
    //ulCount++;

    /* If the timer has expired stop it from running. */
    if( ulCount == 1 )
    {
        /* Do not use a block time if calling a timer API function
        from a timer callback function, as doing so could cause a
        deadlock! */
        vTimerSetTimerID( xTimer, 0 ); //( void * ) ulCount
        xTimerStop( xTimer, 0 );
    }
    UARTSend("Button Timer Callback\n");
}

/*
 * WRITE DESCRIPTION
 */
void vLandTimerCallback( TimerHandle_t xTimerLand )
{
    uint32_t ulCount;
    UARTSend("Landing Timer Callback\n");

    ulCount = ( uint32_t ) pvTimerGetTimerID( xTimerLand );
    ulCount++;
    vTimerSetTimerID( xTimerLand, (void *) ulCount );
}

/*
 * RTOS task that displays number of LED flashes on the OLED display. - Remove in final version.
 */
static void
OLEDDisplay (void *pvParameters)
{
    char string[DISPLAY_SIZE];
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

        xQueuePeek(xMainPWMQueue, &main_PWM, 10);
        xQueuePeek(xTailPWMQueue, &tail_PWM, 10);

        xQueuePeek(xFSMQueue, &state, 10);

        usnprintf(string, sizeof(string), "Alt(%%) %3d|%3d ", des_alt, act_alt);
        OLEDStringDraw(string, COLUMN_ZERO, ROW_ZERO);

        usnprintf(string, sizeof(string), "Yaw   %4d|%3d ", des_yaw, act_yaw);
        OLEDStringDraw(string, COLUMN_ZERO, ROW_ONE);

        usnprintf(string, sizeof(string), "PWM(%%) %3d|%3d ", main_PWM, tail_PWM);
        OLEDStringDraw(string, COLUMN_ZERO, ROW_TWO);

        //usnprintf(string, sizeof(string), "State = %d", state);
        usnprintf(string, sizeof(string), "%s     ", states[state]);
        OLEDStringDraw(string, COLUMN_ZERO, ROW_THREE);

        vTaskDelay(DISPLAY_PERIOD / portTICK_RATE_MS);
    }
}

/*
 * Initialise the main clock.
 */
void
initClk(void)
{
    // Initialise clock frequency to 80MHz
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
}

/*
 * Initialize the altitude and yaw PI controllers
 */
void
initControllers(void)
{
    initController(&g_alt_controller, ALT_KP, ALT_KI, ALT_KD, CONTROL_PERIOD, CONTROL_DIVISOR); // Create altitude controller based of preset gains
    initController(&g_yaw_controller, YAW_KP, YAW_KI, YAW_KD, CONTROL_PERIOD, CONTROL_DIVISOR); // Create yaw controller based of preset gains
}

/*
 * Collection of all initialise functions.
 */
void
init(void)
{
    initClk();
    //initReset();
    initPWM();
    initLED();
    OLEDInitialise();
    initBtns();
    initialiseUSB_UART();
    initADC();
    initQuadrature();
    initReferenceYaw();
    initControllers();

}

/*
 * Create all of the RTOS tasks.
 */
void
createTasks(void)
{
    xTaskCreate(BlinkLED,       "LED Task",     LED_STACK_DEPTH,        NULL,       LED_TASK_PRIORITY,      &Blinky);
    xTaskCreate(OLEDDisplay,    "OLED Task",    OLED_STACK_DEPTH,       NULL,       OLED_TASK_PRIORITY,     &OLEDDisp);
    xTaskCreate(ButtonsCheck,   "Btn Poll",     BTN_STACK_DEPTH,        NULL,       BTN_TASK_PRIORITY,      &BtnCheck);
    xTaskCreate(SwitchesCheck,  "Switch Poll",  SWITCH_STACK_DEPTH,     NULL,       SWI_TASK_PRIORITY,      &SwitchCheck);
    xTaskCreate(Trigger_ADC,    "ADC Handler",  ADC_STACK_DEPTH,        NULL,       ADC_TASK_PRIORITY,      &ADCTrig);
    xTaskCreate(Mean_ADC,       "ADC Mean",     ADC_STACK_DEPTH,        NULL,       ADC_TASK_PRIORITY,      &ADCMean);
    xTaskCreate(Set_Main_Duty,  "Altitude PWM", ALT_STACK_DEPTH,        NULL,       ALT_TASK_PRIORITY,      &MainPWM);
    xTaskCreate(Set_Tail_Duty,  "Yaw PWM",      YAW_STACK_DEPTH,        NULL,       YAW_TASK_PRIORITY,      &TailPWM);
    xTaskCreate(FSM,            "FSM",          YAW_STACK_DEPTH,        NULL,       FSM_TASK_PRIORITY,      &FSMTask);
    //xTaskCreate(GetStackUsage,  "Stack usage",  TASK_STACK_DEPTH,       NULL,       STACK_TASK_PRIORITY,    NULL);
}

/*
 * Create all of the RTOS queues.
 */
void
createQueues(void)
{
    int32_t queue_init = 0; // Value used to initalise queues

    // Create queues
    xOLEDQueue      = xQueueCreate(1, sizeof( uint32_t ) );
    xAltBtnQueue    = xQueueCreate(1, sizeof( uint32_t ) );
    xYawBtnQueue    = xQueueCreate(1, sizeof( uint32_t ) );
    xModeQueue      = xQueueCreate(1, sizeof( uint32_t ) );
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
    xQueueOverwrite(xModeQueue, &queue_init);
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
 * Create all of the RTOS semaphores, event groups and mutexes.
 */
void
createSemaphores(void)
{
    int event_init = 0; //  Value used to initalise event groups

    // Create mutexs to avoid race conditions with the altitude and yaw values
    xAltMutex = xSemaphoreCreateMutex();
    xYawMutex = xSemaphoreCreateMutex();
    xUpBtnSemaphore = xSemaphoreCreateCounting(2, 0);

    // Create event groups to act as flags
    xFoundAltReference = xEventGroupCreate();
    xFoundYawReference = xEventGroupCreate();

    // Initalise event groups to zero
    xEventGroupSetBits(xFoundAltReference, event_init);
    xEventGroupSetBits(xFoundYawReference, event_init);
}

void
createtimers(void)
{
    xUpBtnTimer     = xTimerCreate( "Button Timer", DBL_BTN_TMR_PERIOD / portTICK_RATE_MS, pdFALSE, ( void * ) 0, vBtnTimerCallback );
    xLandingTimer   = xTimerCreate( "Land Timer", LAND_TMR_PERIOD / portTICK_RATE_MS, pdTRUE, ( void * ) 0, vLandTimerCallback );
}

/*
 * Stack overflow hook
 * This function is triggered when a stack overflow occurs
 */
void
vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    UARTSend("OVERFLOW\n");
    UARTSend(pcTaskName);
    while (1){}
}

int
main(void)
 {
    init();
    createTasks();
    createQueues();
    createSemaphores();
    createtimers();
    UARTSend("Starting...\n");

    vTaskStartScheduler();
    while(1);
}
//End
