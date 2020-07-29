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

#include "utils/ustdlib.h"
#include "OrbitOLED/OrbitOLEDInterface.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*
 * DEFINITIONS
 */
#define LED_BLINK_RATE      1000                                // (ms) Duration to suspend LED task
#define OLED_REFRESH_RATE   200
#define LED_PIN_RED         1                                   // RED Led pin

#define TASK_STACK_DEPTH    32
#define LED_TASK_PRIORITY   4
#define OLED_TASK_PRIORITY  5

xQueueHandle xOLEDQueue;
int value = 0;

/*
 * BLINKY FUNCTION
 */
void BlinkLED(void *pvParameters)
{
    uint8_t whichLed = *((uint8_t *)pvParameters);              // pvParameters is a pointer to an unsigned 8 bit integer - the LED pin number

    const uint8_t whichBit = 1 << whichLed;                     // TivaWare GPIO calls require the pin# as a binary bitmask, not a simple number.
                                                                // Alternately, we could have passed the bitmask into pvParameters instead of a simple number.
    uint8_t currentValue = 0;

    while (1)
    {
        currentValue ^= whichBit;                               // XOR keeps flipping the bit on / off alternately each time this runs.
        GPIOPinWrite(GPIO_PORTF_BASE, whichBit, currentValue);
        vTaskDelay(LED_BLINK_RATE / portTICK_RATE_MS);              // Suspend this task (so others may run) for BLINK_RATE (or as close as we can get with the current RTOS tick setting).
    }
    // No way to kill this blinky task unless another task has an xTaskHandle reference to it and can use vTaskDelete() to purge it.
}





static void
OLEDDisplay (void *pvParameters)
{
    char cMessage[17];
    xQueueReceive(xOLEDQueue, &cMessage, portMAX_DELAY);

    value++;
    usnprintf(cMessage, sizeof(cMessage), "Default %d", value, '%');
    OLEDStringDraw(cMessage, 0, 0);
}




void initClk(void)
{
    // Set the clock rate to 80 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
}

void initGPIO(void)
{
    // For LED blinky task - initialize GPIO port F and then pin #1 (Red) for output
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                // Activate internal bus clocking for GPIO port F
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));        // Busy-wait until GPIOF's bus clock is ready
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);         // PF_1 as output
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);    // Doesn't need too much drive strength as the RGB LEDs on the TM4C123 launchpad are switched via N-type transistors
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);               // Off by default
}

void init(void)
{
    initClk();
    initGPIO();
    OLEDInitialise();
}

void createTasks(void)
{
    // LED pin number - static preserves the value while the task is running
    static uint8_t led = LED_PIN_RED;

    xTaskCreate(BlinkLED,       "Blinker",  TASK_STACK_DEPTH, (void *) &led,        LED_TASK_PRIORITY,  NULL);
    xTaskCreate(OLEDDisplay,    "Screen",   TASK_STACK_DEPTH, (void *) &xOLEDQueue, OLED_TASK_PRIORITY, NULL);
}

void createQueues(void)
{
    xOLEDQueue = xQueueCreate(1, sizeof( uint32_t ) );
}

int main(void)
{
    init();
    createTasks();


    vTaskStartScheduler();
    while(1);
}
//End
