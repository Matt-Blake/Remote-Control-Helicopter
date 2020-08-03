/*
 * TODO:
 *      - Suss UART
 *      - Figure out what the stack sizes should be
 *      - Suss interrupts
 */

// Mutexes are for shared resources
// Binary semaphores are used for interrupts I think

/*
 * INCLUDES
 */
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

#include "SwitchTask.h"
#include "pwm.h"
#include "reset.h"
#include "yaw.h"
#include "uart.h"
#include "buttons.h"


/*
 * DEFINITIONS
 */
#define OLED_REFRESH_RATE       200     // OLED Screen refresh rate (I think)
#define SAMPLE_RATE_HZ          200
#define LED_PIN_RED             1       // RED LED pin
#define BLINK_STACK_DEPTH       32
#define OLED_STACK_DEPTH        32
#define SWITCH_STACK_DEPTH      128     // Stack size in words
#define LED_TASK_PRIORITY       4       // Blinky priority
#define OLED_TASK_PRIORITY      5       // OLED priority
#define SWITCH_TASK_PRIORITY    6       // Switch task priority

QueueHandle_t xOLEDQueue;

QueueHandle_t xYawBtnQueue;
QueueHandle_t xAltBtnQueue;

SemaphoreHandle_t xAltMutex;
SemaphoreHandle_t xYawMutex;

SemaphoreHandle_t xTokenMutex;

int value = 1;

/*
 * BLINKY FUNCTION
 */
static void
BlinkLED(void *pvParameters)
{
    //uint8_t whichLed = *((uint8_t *)pvParameters);              // pvParameters is a pointer to an unsigned 8 bit integer - the LED pin number
    static uint8_t on = 0;
    //const uint8_t whichBit = 1 << whichLed;                     // TivaWare GPIO calls require the pin# as a binary bitmask, not a simple number.
                                                                // Alternately, we could have passed the bitmask into pvParameters instead of a simple number.
    uint8_t currentValue = 0;
    uint16_t led_blink_rate = 500;

    while(1)
    {
        if(xSemaphoreTake(xAltMutex, 200/portTICK_RATE_MS) == pdPASS){
            xQueueReceive(xAltBtnQueue, &led_blink_rate, 10);
            currentValue ^= 2;                               // XOR keeps flipping the bit on / off alternately each time this runs.
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, currentValue);
            xQueueSend(xOLEDQueue, &value, 0);
            //if(currentValue == 0){value++;}
            if(on){on = 0; value++;}else{on = 1;}

            xSemaphoreGive(xAltMutex);
        }
        vTaskDelay(led_blink_rate / portTICK_RATE_MS);              // Suspend this task (so others may run) for BLINK_RATE (or as close as we can get with the current RTOS tick setting).
    }// No way to kill this blinky task unless another task has an xTaskHandle reference to it and can use vTaskDelete() to purge it.
}


static void
OLEDDisplay (void *pvParameters)
{
    char cMessage[17];
    int  num_flashes;
    while(1)
    {
        xQueueReceive(xOLEDQueue, &num_flashes, 10);

        usnprintf(cMessage, sizeof(cMessage), "%d flashes", num_flashes);
        OLEDStringDraw(cMessage, 0, 0);
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}



void
initClk(void)
{
    // Set the clock rate to 80 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);                       // Set clock frequency
}

void
initGPIO(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                // Activate internal bus clocking for GPIO port F
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));        // Busy-wait until GPIOF's bus clock is ready
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);         // PF_1 as output
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);    // Doesn't need too much drive strength as the RGB LEDs on the TM4C123 launchpad are switched via N-type transistors
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);               // Off by default
}

void
init(void)
{
    initClk();
    initGPIO();
    OLEDInitialise();
    initBtns();
    initialiseUSB_UART();
    //SwitchTaskInit();
}

void
createTasks(void)
{
    // LED pin number - static preserves the value while the task is running
    static uint8_t led = LED_PIN_RED;

    xTaskCreate(BlinkLED,       "Blinker",  BLINK_STACK_DEPTH,  (void *) &led,      LED_TASK_PRIORITY,      NULL);
    xTaskCreate(OLEDDisplay,    "Screen",   OLED_STACK_DEPTH,   NULL,               OLED_TASK_PRIORITY,     NULL);
    xTaskCreate(ButtonsCheck,   "Switch",   SWITCH_STACK_DEPTH, NULL,               SWITCH_TASK_PRIORITY,   NULL);
    //xTaskCreate(SwitchTask,     "Switch",   SWITCH_STACK_DEPTH, NULL,               SWITCH_TASK_PRIORITY,   NULL);
}


void
createQueues(void)
{
    xOLEDQueue = xQueueCreate(5, sizeof( uint32_t ) );

    xAltBtnQueue = xQueueCreate(5, sizeof( uint32_t ) );
    xYawBtnQueue = xQueueCreate(5, sizeof( uint32_t ) );
}

void
createSemaphores(void)
{
    xTokenMutex = xSemaphoreCreateMutex();
    xAltMutex = xSemaphoreCreateMutex();
    xYawMutex = xSemaphoreCreateMutex();
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
