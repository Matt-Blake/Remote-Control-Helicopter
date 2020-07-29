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

#include "buttons.h"
#include "inc/hw_gpio.h"

/*
 * DEFINITIONS
 */
                            // (ms) Duration to suspend LED task
#define OLED_REFRESH_RATE   200
#define LED_PIN_RED         1                                   // RED Led pin

#define TASK_STACK_DEPTH    32
#define LED_TASK_PRIORITY   4
#define OLED_TASK_PRIORITY  5

#define SWITCHTASKSTACKSIZE   128         // Stack size in words
#define PRIORITY_SWITCH_TASK  3

xQueueHandle xOLEDQueue;
xQueueHandle xButtonQueue;
int value = 1;


/*
 * BLINKY FUNCTION
 */
void BlinkLED(void *pvParameters)
{
    uint8_t whichLed = *((uint8_t *)pvParameters);              // pvParameters is a pointer to an unsigned 8 bit integer - the LED pin number

    const uint8_t whichBit = 1 << whichLed;                     // TivaWare GPIO calls require the pin# as a binary bitmask, not a simple number.
                                                                // Alternately, we could have passed the bitmask into pvParameters instead of a simple number.
    uint8_t currentValue = 0;
    uint16_t led_blink_rate = 500;

    for (;;)
    {
        xQueueReceive(xButtonQueue, &led_blink_rate, 10);

        currentValue ^= whichBit;                               // XOR keeps flipping the bit on / off alternately each time this runs.
        //GPIOPinWrite(GPIO_PORTF_BASE, 14, currentValue);
        GPIOPinWrite(GPIO_PORTF_BASE, whichBit, currentValue);
        xQueueSend(xOLEDQueue, &value, 0);
        if(currentValue == 0){value++;}
        vTaskDelay(led_blink_rate / portTICK_RATE_MS);              // Suspend this task (so others may run) for BLINK_RATE (or as close as we can get with the current RTOS tick setting).
    }

    // No way to kill this blinky task unless another task has an xTaskHandle reference to it and can use vTaskDelete() to purge it.
}

static void
SwitchTask(void *pvParameters)
{
    portTickType ui16LastTime;
    uint32_t ui32SwitchDelay = 25;
    uint8_t ui8CurButtonState, ui8PrevButtonState;
    uint16_t led_blink_rate = 500;

    ui8CurButtonState = ui8PrevButtonState = 0;

    //
    // Get the current tick count.
    //
    ui16LastTime = xTaskGetTickCount();

    //
    // Loop forever.
    //
    while(1)
    {
        //
        // Poll the debounced state of the buttons.
        //
        ui8CurButtonState = ButtonsPoll(0, 0);

        //
        // Check if previous debounced state is equal to the current state.
        //
        if(ui8CurButtonState != ui8PrevButtonState)
        {
            ui8PrevButtonState = ui8CurButtonState;

            //
            // Check to make sure the change in state is due to button press
            // and not due to button release.
            //
            if((ui8CurButtonState & ALL_BUTTONS) != 0)
            {
                if((ui8CurButtonState & ALL_BUTTONS) == LEFT_BUTTON)
                {
                    if (led_blink_rate < 1000) {
                        led_blink_rate += 100;
                    }

                }
                else if((ui8CurButtonState & ALL_BUTTONS) == RIGHT_BUTTON)
                {

                    if (led_blink_rate > 0) {
                        led_blink_rate -= 100;
                    }

                }

                //
                // Pass the value of the button pressed to LEDTask.
                //
                if(xQueueSend(xButtonQueue, &led_blink_rate, portMAX_DELAY) !=
                   pdPASS)
                {
                    //
                    // Error. The queue should never be full. If so print the
                    // error message on UART and wait for ever.
                    //
                    while(1)
                    {
                    }
                }
            }
        }

        //
        // Wait for the required amount of time to check back.
        //
        vTaskDelayUntil(&ui16LastTime, ui32SwitchDelay / portTICK_RATE_MS);
    }
}

uint32_t
SwitchTaskInit(void)
{
    //
    // Unlock the GPIO LOCK register for Right button to work.
    //
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0xFF;

    //
    // Initialize the buttons
    //
    ButtonsInit();
    //
    // Create the switch task.
    //
    /*
    if(xTaskCreate(SwitchTask, (const portCHAR *)"Switch",
                   TASK_STACK_DEPTH, NULL, tskIDLE_PRIORITY +
                   PRIORITY_SWITCH_TASK, NULL) != pdTRUE)
    {
        return(1);
    }
    */
    //
    // Success.
    //
    return(1);
}

static void
OLEDDisplay (void *pvParameters)
{
    char cMessage[17];
    int  num_flashes;
    for (;;)
    {
        xQueueReceive(xOLEDQueue, &num_flashes, 10);

        usnprintf(cMessage, sizeof(cMessage), "%d flashes", num_flashes);
        OLEDStringDraw(cMessage, 0, 0);
    }
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
    SwitchTaskInit();
}

void createTasks(void)
{
    // LED pin number - static preserves the value while the task is running
    static uint8_t led = LED_PIN_RED;

    xTaskCreate(BlinkLED,       "Blinker",  TASK_STACK_DEPTH, (void *) &led,    LED_TASK_PRIORITY,  NULL);
    xTaskCreate(OLEDDisplay,    "Screen",   TASK_STACK_DEPTH, NULL,             OLED_TASK_PRIORITY, NULL);
    xTaskCreate(SwitchTask,     "Switch",   SWITCHTASKSTACKSIZE, NULL,             PRIORITY_SWITCH_TASK, NULL);
}


void createQueues(void)
{
    xOLEDQueue = xQueueCreate(1, sizeof( uint32_t ) );
    xButtonQueue = xQueueCreate(5, sizeof( uint32_t ) );
}


int main(void)
{
    init();
    createTasks();
    createQueues();


    vTaskStartScheduler();
    while(1);
}
//End
