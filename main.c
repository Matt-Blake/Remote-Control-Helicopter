/*
 * TODO:
 *      - Figure out what the stack sizes should be
 *      - Suss interrupts
 *      - Suss yaw
 *      - Suss ADC
 *
 */

// Mutexes are for shared resources
// Binary semaphores are used for interrupts I think

//******************************************************
// Includes
//******************************************************
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

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "pwm.h"
#include "reset.h"
#include "yaw.h"
#include "uart.h"
#include "ADC.h"
#include "buttons.h"
#include "pidController.h"

//******************************************************
// Constants
//******************************************************
//#define OLED_REFRESH_RATE       200     // OLED Screen refresh rate (I think)
//#define SAMPLE_RATE_HZ          200
#define LED_PIN_RED             1       // RED LED pin

#define LED_STACK_DEPTH         32
#define OLED_STACK_DEPTH        128
#define BTN_STACK_DEPTH         128     // Stack size in words
#define ADC_STACK_DEPTH         128     // Stack size in words
#define ALT_STACK_DEPTH         128     // Stack size in words
#define YAW_STACK_DEPTH         128     // Stack size in words

// Max priority is 8
#define LED_TASK_PRIORITY       5       // LED task priority
#define OLED_TASK_PRIORITY      5       // OLED priority
#define BTN_TASK_PRIORITY       6       // Button polling task priority
#define ADC_TASK_PRIORITY       7       // ADC sampling priority
#define ALT_TASK_PRIORITY       8       // Altitude PWM priority
#define YAW_TASK_PRIORITY       8       // Yaw PWM priority

#define ALT_KP                  130000  // Altitude proportional gain
#define ALT_KI                  2       // Altitude integral gain
#define ALT_KD                  4       // Altitude derivative gain
#define YAW_KP                  98000   // Yaw proportional gain
#define YAW_KI                  11      // Yaw integral gain
#define YAW_KD                  3       // Yaw derivative gain
#define CONTROL_DIVISOR         1       // Divisor used to achieve certain gains without the use of floating point numbers
#define MS_TO_S                 1000    // The conversion factor from milliseconds to seconds

#define DISPLAY_PERIOD          200

#define MAX_BUTTON_PRESSES      10      // The maximum number of concurrent button presses than can be stored for servicing

/*
 * Globals
*/
QueueHandle_t xOLEDQueue;
QueueHandle_t xAltBtnQueue;
QueueHandle_t xYawBtnQueue;
QueueHandle_t xModeQueue;
QueueHandle_t xAltMeasQueue;
QueueHandle_t xAltRefQueue;
QueueHandle_t xYawRefQueue;

SemaphoreHandle_t xAltMutex;
SemaphoreHandle_t xYawMutex;
SemaphoreHandle_t xLBtnSemaphore;
SemaphoreHandle_t xRBtnSemaphore;

// Just number of LED flashes
uint32_t value = 0;

//******************************************************
// Tasks
//******************************************************
/*
 * RTOS task that toggles LED state based off button presses
 */
/*
static void
BlinkLED(void *pvParameters)
{
    uint8_t currentValue = 0;
    static uint8_t state = 0;
    uint8_t prev_state = 0;

    while(1)
    {
        if(xSemaphoreTake(xAltMutex, 0/portTICK_RATE_MS) == pdPASS){
            prev_state = state;
            //UARTSend("Blink\n");
            xQueueReceive(xAltBtnQueue, &state, 10);
            if(state == 0){currentValue &= !2;}
            if(state == 1){currentValue |= 2;}
            if(prev_state == 0 && state == 1){
                value++;
            }

            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, currentValue);
            xQueueOverwrite(xOLEDQueue, &value);

            xSemaphoreGive(xAltMutex);
        }
        vTaskDelay(200 / portTICK_RATE_MS);
    }
}
*/

/*
 * RTOS task that displays number of LED flashes on the OLED display. - Remove in final version.
 */
static void
OLEDDisplay (void *pvParameters)
{
    char cMessage[17];
    uint32_t  num_flashes;
    while(1)
    {
        xQueueReceive(xOLEDQueue, &num_flashes, 10);

        usnprintf(cMessage, sizeof(cMessage), "%d flashes", num_flashes);
        OLEDStringDraw(cMessage, 0, 0);

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
 * Initialise the LED pin and peripherals.
 */
void
initLED(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                // Activate internal bus clocking for GPIO port F
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));        // Busy-wait until GPIOF's bus clock is ready
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);         // PF_2 as output
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);    // Doesn't need too much drive strength as the RGB LEDs on the TM4C123 launchpad are switched via N-type transistors
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);               // Off by default
}

/*
 * Initialize the altitude and yaw PI controllers
 */
void
initControllers(void)
{
    initController(&g_alt_controller, ALT_KP, ALT_KI, ALT_KD, CONTROL_PERIOD * MS_TO_S, CONTROL_DIVISOR); // Create altitude controller based of preset gains
    initController(&g_yaw_controller, YAW_KP, YAW_KI, YAW_KD, CONTROL_PERIOD * MS_TO_S, CONTROL_DIVISOR); // Create yaw controller based of preset gains
}

/*
 * Collection of all initialise functions.
 */
void
init(void)
{
    initClk();
    initLED();
    initPWM();
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
    //xTaskCreate(BlinkLED,       "LED Task",     LED_STACK_DEPTH,        NULL,       LED_TASK_PRIORITY,      NULL);
    xTaskCreate(OLEDDisplay,    "OLED Task",    OLED_STACK_DEPTH,       NULL,       OLED_TASK_PRIORITY,     NULL);
    xTaskCreate(ButtonsCheck,   "Btn Poll",     BTN_STACK_DEPTH,        NULL,       BTN_TASK_PRIORITY,      NULL);
    xTaskCreate(Trigger_ADC,     "ADC Handler",  ADC_STACK_DEPTH,        NULL,       ADC_TASK_PRIORITY,      NULL);
    xTaskCreate(Mean_ADC,       "ADC Mean",     ADC_STACK_DEPTH,        NULL,       ADC_TASK_PRIORITY,      NULL);
    xTaskCreate(Set_Main_Duty,  "Altitude PWM", ALT_STACK_DEPTH,        NULL,       ALT_TASK_PRIORITY,      NULL);
    xTaskCreate(Set_Tail_Duty,  "Yaw PWM",      YAW_STACK_DEPTH,        NULL,       YAW_TASK_PRIORITY,      NULL);
}

/*
 * Create all of the RTOS queues.
 */
void
createQueues(void)
{
    xOLEDQueue      = xQueueCreate(1, sizeof( uint32_t ) );
    xAltBtnQueue    = xQueueCreate(1, sizeof( uint32_t ) );
    xYawBtnQueue    = xQueueCreate(1, sizeof( uint32_t ) );
    xModeQueue      = xQueueCreate(1, sizeof( uint32_t ) );
    xAltMeasQueue   = xQueueCreate(1, sizeof( uint32_t ) );
    xAltRefQueue    = xQueueCreate(2, sizeof( uint32_t ) );
    xYawRefQueue    = xQueueCreate(1, sizeof( uint32_t ) );
}

/*
 * Create all of the RTOS semaphores and mutexes.
 */
void
createSemaphores(void)
{
    // Create mutexs to avoid race conditions with the altitude and yaw values
    xAltMutex = xSemaphoreCreateMutex();
    xYawMutex = xSemaphoreCreateMutex();

    // Create semaphores to keep track of how many times the yaw buttons have been pushed
    xLBtnSemaphore = xSemaphoreCreateCounting(MAX_BUTTON_PRESSES, 0);
    xRBtnSemaphore = xSemaphoreCreateCounting(MAX_BUTTON_PRESSES, 0);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
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
    UARTSend("Starting...\n");

    vTaskStartScheduler();
    while(1);
}
//End
