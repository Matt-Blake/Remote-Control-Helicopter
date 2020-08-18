/*
 * BUTTONS.c
 *
 * Module for the D-Pad Buttons (U/D/L/R) and Switches
 * Supports buttons on the Tiva/Orbit.
 * Comprises of initialisers and button checks
 *
 * Author: P.J. Bones UCECE
 * Edited: Derrick Edward, Grayson Mynott, Ryan Earwaker
 * Thu AM Group 18
 * Last modified:  29.05.2019
 *
 * buttons.c - Code updated for use with FreeRTOS
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 05/08/2020
 */

#include "buttons.h"


#define U_BTN_PERIPH        SYSCTL_PERIPH_GPIOE         // Up Peripheral
#define U_BTN_PORT_BASE     GPIO_PORTE_BASE             // Up Port Base
#define U_BTN_PIN           GPIO_PIN_0                  // Up Pin
#define U_BTN_NORMAL        false                       // Up Inactive State (Active HIGH)
#define D_BTN_PERIPH        SYSCTL_PERIPH_GPIOD         // Down Peripheral
#define D_BTN_PORT_BASE     GPIO_PORTD_BASE             // Down Port Base
#define D_BTN_PIN           GPIO_PIN_2                  // Down Pin
#define D_BTN_NORMAL        false                       // Down Inactive State (Active HIGH)
#define L_BTN_PERIPH        SYSCTL_PERIPH_GPIOF         // Left Peripheral
#define L_BTN_PORT_BASE     GPIO_PORTF_BASE             // Left Port Base
#define L_BTN_PIN           GPIO_PIN_4                  // Left Pin
#define L_BTN_NORMAL        true                        // Left Inactive State (Active LOW)
#define R_BTN_PERIPH        SYSCTL_PERIPH_GPIOF         // Right Peripheral
#define R_BTN_PORT_BASE     GPIO_PORTF_BASE             // Right Port Base
#define R_BTN_PIN           GPIO_PIN_0                  // Right Pin
#define R_BTN_NORMAL        true                        // Right Inactive State (Active LOW)
#define NUM_BTN_POLLS       3                           // Number Of Times To Poll The Buttons (For Debouncing)
#define SW_PERIPH           SYSCTL_PERIPH_GPIOA         // Switch Peripheral
#define SW_PORT_BASE        GPIO_PORTA_BASE             // Switch Port Base
#define L_SW_PIN            GPIO_PIN_6                  // Left Switch Pin
#define R_SW_PIN            GPIO_PIN_7                  // Right Switch Pin
#define ALT_CHANGE          10                          // The altitude change on button press (percentage)
#define MAX_ALT             100                         // The maximum altitude (percentage)
#define MIN_ALT             0                           // The minimum altitude (percentage)
#define YAW_CHANGE          15                          // The yaw change on button press (degrees)
#define MAX_YAW             164                         // The maximum yaw (degrees) before increment
#define MIN_YAW             -165                        // The minimum yaw (degrees) before increment
#define DEGREES_CIRCLE      360                         // The number of degrees in a circle

typedef enum HELI_STATE {LANDED = 0, TAKEOFF = 1, FLYING = 2, LANDING = 3} HELI_STATE;

// *****************************************************
// Globals
// *****************************************************
static bool btn_state[NUM_BTNS];	// Corresponds to the electrical state
static uint8_t btn_count[NUM_BTNS];
static bool btn_flag[NUM_BTNS];
static bool btn_normal[NUM_BTNS];   // Corresponds to the electrical state

TimerHandle_t xUpBtnTimer;
TimerHandle_t xYawFlipTimer;

TaskHandle_t BtnCheck;
TaskHandle_t SwitchCheck;

QueueHandle_t xAltBtnQueue;
QueueHandle_t xYawBtnQueue;


SemaphoreHandle_t xUpBtnSemaphore;
SemaphoreHandle_t xYawFlipSemaphore;


/*
 * Function:    vBtnTimerCallback
 * -------------------------------
 * Handler for the button timer.
 * When the button timer expires, this function
 * resets the timer ID.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void vDblBtnTimerCallback( TimerHandle_t xTimer )
{
    uint32_t    ulCount;
    uint8_t     reset = 0;

    /* The number of times this timer has expired is saved as the
    timer's ID.  Obtain the count. */
    ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );

    /* Increment the count, then test to see if the timer has expired
    ulMaxExpiryCountBeforeStopping yet. */
    //ulCount++;

    /* If the timer has expired stop it from running. */
    if( ulCount >= 1 )
    {
        /* Do not use a block time if calling a timer API function
        from a timer callback function, as doing so could cause a
        deadlock! */
        vTimerSetTimerID( xTimer, ( void * ) reset ); //( void * ) ulCount
        xTimerStop( xTimer, reset );
    }
    UARTSend("Btn Timer Callback\n\r");
}



// *****************************************************
// initButtons: Initialise the variables associated with the set of buttons
// defined by the constants in the buttons2.h header file.
void
initBtns(void)
{
    uint8_t i;

    // UP button (active HIGH)
    SysCtlPeripheralEnable(U_BTN_PERIPH);
    GPIOPinTypeGPIOInput(U_BTN_PORT_BASE, U_BTN_PIN);
    GPIOPadConfigSet(U_BTN_PORT_BASE, U_BTN_PIN, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPD);
    btn_normal[UP] = U_BTN_NORMAL;

    // DOWN button (active HIGH)
    SysCtlPeripheralEnable(D_BTN_PERIPH);
    GPIOPinTypeGPIOInput(D_BTN_PORT_BASE, D_BTN_PIN);
    GPIOPadConfigSet(D_BTN_PORT_BASE, D_BTN_PIN, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPD);
    btn_normal[DOWN] = D_BTN_NORMAL;

    // LEFT button (active LOW)
    SysCtlPeripheralEnable(L_BTN_PERIPH);
    GPIOPinTypeGPIOInput(L_BTN_PORT_BASE, L_BTN_PIN);
    GPIOPadConfigSet(L_BTN_PORT_BASE, L_BTN_PIN, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);
    btn_normal[LEFT] = L_BTN_NORMAL;

    // RIGHT button (active LOW)
    // Note that PF0 is one of a handful of GPIO pins that need to be
    // "unlocked" before they can be reconfigured.  This also requires
    //      #include "inc/tm4c123gh6pm.h"
    SysCtlPeripheralEnable(R_BTN_PERIPH);
    //---Unlock PF0 for the right button:
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTF_CR_R |= GPIO_PIN_0; //PF0 unlocked
    GPIO_PORTF_LOCK_R = GPIO_LOCK_M;
    GPIOPinTypeGPIOInput(R_BTN_PORT_BASE, R_BTN_PIN);
    GPIOPadConfigSet(R_BTN_PORT_BASE, R_BTN_PIN, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);
    btn_normal[RIGHT] = R_BTN_NORMAL;

    for (i = 0; i < NUM_BTNS; i++)
    {
        btn_state[i] = btn_normal[i];
        btn_count[i] = 0;
        btn_flag[i] = false;
    }

    // Switches
    SysCtlPeripheralEnable(SW_PERIPH);
    GPIOPinTypeGPIOInput(SW_PORT_BASE, R_SW_PIN | L_SW_PIN);
    GPIOPadConfigSet(SW_PORT_BASE, R_SW_PIN | L_SW_PIN, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPD);
}

// *******************************************************
// updateButtons: Function designed to be called regularly. It polls all
// buttons once and updates variables associated with the buttons if
// necessary.  It is efficient enough to be part of an ISR, e.g. from
// a SysTick interrupt.
// Debounce algorithm: A state machine is associated with each button.
// A state change occurs only after NUM_BTN_POLLS consecutive polls have
// read the pin in the opposite condition, before the state changes and
// a flag is set.  Set NUM_BTN_POLLS according to the polling rate.
void
updateButtons(void)
{
    bool btn_value[NUM_BTNS];
    int i;

    // Read the pins; true means HIGH, false means LOW
    btn_value[UP] =     (GPIOPinRead(U_BTN_PORT_BASE, U_BTN_PIN) == U_BTN_PIN);
    btn_value[DOWN] =   (GPIOPinRead(D_BTN_PORT_BASE, D_BTN_PIN) == D_BTN_PIN);
    btn_value[LEFT] =   (GPIOPinRead(L_BTN_PORT_BASE, L_BTN_PIN) == L_BTN_PIN);
    btn_value[RIGHT] =  (GPIOPinRead(R_BTN_PORT_BASE, R_BTN_PIN) == R_BTN_PIN);
    // Iterate through the buttons, updating button variables as required
    for (i = 0; i < NUM_BTNS; i++)
    {
        if (btn_value[i] != btn_state[i])
        {
            btn_count[i]++;
            if (btn_count[i] >= NUM_BTN_POLLS)
            {
                btn_state[i] = btn_value[i];
                btn_flag[i] = true;	   // Reset by call to checkbutton()
                btn_count[i] = 0;
            }
        }
        else
            btn_count[i] = 0;
    }
}

// *******************************************************
// checkButton: Function returns the new button logical state if the button
// logical state (PUSHED or RELEASED) has changed since the last call,
// otherwise returns NO_CHANGE.
uint8_t
checkButton(uint8_t btnName)
{
    if (btn_flag[btnName])
    {
        btn_flag[btnName] = false;
        if (btn_state[btnName] == btn_normal[btnName])
            return RELEASED;
        else
            return PUSHED;
    }
    return NO_CHANGE;
}

/*
 * Increment the altitude by 10% if the up button has been pushed
 */
void
upButtonPush(void)
{
    uint8_t state;
    int16_t alt_desired = 0;

    UARTSend ("Up\n");
    if(xSemaphoreTake(xAltMutex, 0/portTICK_RATE_MS) == pdPASS){ // If the altitude mutex is free, increment the altitude
        xQueuePeek(xAltDesQueue, &alt_desired, 10); // Retrieve desired altitude data from the RTOS queue

        alt_desired += ALT_CHANGE;

        // Check upper limits of the altitude when left button is pressed
        if (alt_desired > MAX_ALT)
        {
            alt_desired = MAX_ALT;
        }
        xQueueOverwrite(xAltDesQueue, &alt_desired); // Update the RTOS altitude reference queue
        xSemaphoreGive(xAltMutex); // Give altitude mutex so other mutually exclusive altitude tasks can run
    }

    if(xQueueOverwrite(xAltBtnQueue, &state) != pdPASS) { // Error. The queue should never be full. If so print the error message on UART and wait for ever.
        UARTSend("AltBtnQueue failed.\n");
        while(1){}
    }
}

/*
 * Decrement the desired altitude by 10% if the down button has been pushed
 */
void
downButtonPush(void)
{
    uint8_t state;
    int16_t alt_desired = 0;

    UARTSend ("Down\n");
    if(xSemaphoreTake(xAltMutex, 0/portTICK_RATE_MS) == pdPASS){ // If the altitude mutex is free, increment the altitude
        xQueuePeek(xAltDesQueue, &alt_desired, 10); // Retrieve desired altitude data from the RTOS queue
        alt_desired -= ALT_CHANGE;

        // Check lower limits of the altitude when left button is pressed
        if (alt_desired < MIN_ALT)
        {
            alt_desired = MIN_ALT;
        }

        xQueueOverwrite(xAltDesQueue, &alt_desired); // Update the RTOS altitude reference queue
        xSemaphoreGive(xAltMutex); // Give altitude mutex so other mutually exclusive altitude tasks can run
    }

    if(xQueueOverwrite(xAltBtnQueue, &state) != pdPASS) { // Error. The queue should never be full. If so print the error message on UART and wait for ever.
        UARTSend("AltBtnQueue failed.\n");
        while(1){}
    }
}

/*
 * Decrement the desired yaw by 15 degrees if the left button has been pushed
 */
void
rightButtonPush(void)
{
    uint8_t state;
    static int32_t yaw_desired = 0;

    UARTSend ("Right\n");

    if(xSemaphoreTake(xYawMutex, 0/portTICK_RATE_MS) == pdPASS){ // If the yaw mutex is free, decrement the yaw
        xQueuePeek(xYawDesQueue, &yaw_desired, 10); // Retrieve desired yaw data from the RTOS queue

        // Check upper limits of the yaw when left button is pressed
        if (yaw_desired >= MIN_YAW) {
        yaw_desired = yaw_desired - YAW_CHANGE;
        } else {
            yaw_desired = DEGREES_CIRCLE - YAW_CHANGE + yaw_desired;
        }

        xQueueOverwrite(xYawDesQueue, &yaw_desired); // Update the RTOS yaw reference queue
        xSemaphoreGive(xYawMutex); // Give yaw mutex so other mutually exclusive yaw tasks can run
    }


    if(xQueueOverwrite(xYawBtnQueue, &state) != pdPASS) {  // Error. The queue should never be full. If so print the error message on UART and wait for ever.
        UARTSend("YawBtnQueue failed.\n");
        while(1){}
    }
}

/*
 * Increment the desired yaw by 15 degrees if the left button has been pushed
 */
void
leftButtonPush(void)
{
    uint8_t state;
    static int32_t yaw_desired = 0;

    UARTSend ("Left\n");

    if(xSemaphoreTake(xYawMutex, 0/portTICK_RATE_MS) == pdPASS){ // If the yaw mutex is free, increment the yaw
        xQueuePeek(xYawDesQueue, &yaw_desired, 10); // Retrieve desired yaw data from the RTOS queue

        // Check upper limits of the yaw if right button is pressed
        if (yaw_desired <= MAX_YAW) {
            yaw_desired = yaw_desired + YAW_CHANGE;
        } else {
            yaw_desired = -DEGREES_CIRCLE + YAW_CHANGE + yaw_desired;
        }

        xQueueOverwrite(xYawDesQueue, &yaw_desired); // Update the RTOS yaw reference queue
        xSemaphoreGive(xYawMutex); // Give yaw mutex so other mutually exclusive yaw tasks can run
    }

    if(xQueueOverwrite(xYawBtnQueue, &state) != pdPASS) {
        // Error. The queue should never be full. If so print the error message on UART and wait for ever.
        UARTSend("YawBtnQueue failed.\n");
        while(1){}
    }
}

/*
 * FreeRTOS task which polls the buttons to check for button presses
 */
void
ButtonsCheck(void *pvParameters)
{

    /*
     * For each button the following procedure is run:
     *
     * If Button State is PUSHED:
     *      Update Target Altitude/Yaw accordingly
     *      If Target Alt/Yaw is now beyond the limits:
     *          Update targets to be at limit (0->100 for Alt, -180->179 for Yaw).
     */

    portTickType ui16LastTaskTime;
    uint32_t ui32ButtonsDelay = 25;
    uint32_t inUpTimeLoop;
    uint32_t inYawTimeLoop;
    int32_t desired_alt;
    int32_t desired_yaw;

    ui16LastTaskTime = xTaskGetTickCount(); // Get the current tick count.

    // Loop forever.
    while(1)
    {
        inUpTimeLoop = ( uint32_t ) pvTimerGetTimerID( xUpBtnTimer );
        inYawTimeLoop = ( uint32_t ) pvTimerGetTimerID( xYawFlipTimer );

        xQueuePeek(xYawDesQueue, &desired_yaw, 10);
        xQueuePeek(xAltDesQueue, &desired_alt, 10);
        /*
         * Check if any buttons have been pressed. Update button state.
         */
        updateButtons();

        if(checkButton(UP) == PUSHED)
        {
            if(inUpTimeLoop == 0) { // check to see if the timer has ran out
                vTimerSetTimerID(xUpBtnTimer, (void *) 1);
                xTimerStart(xUpBtnTimer, 10); // Restarts timer
            } else {
                xSemaphoreGive(xUpBtnSemaphore);
            }

            if (uxSemaphoreGetCount(xUpBtnSemaphore) == 1) {
                xSemaphoreTake(xUpBtnSemaphore, 10);
                desired_alt = 50;
                xQueueOverwrite(xAltDesQueue, &desired_alt);
            }else {
                /*
                 * Call the up button handler to increase target altitude by 10%
                 */
                upButtonPush();
            }
        }

        if(checkButton(DOWN) == PUSHED)
        {
            if(inYawTimeLoop == 0) { // check to see if the timer has ran out
                vTimerSetTimerID(xYawFlipTimer, (void *) 1);
                xTimerStart(xYawFlipTimer, 10); // Restarts timer
            } else {
                xSemaphoreGive(xYawFlipSemaphore);
            }

            if (uxSemaphoreGetCount(xYawFlipSemaphore) == 1) {
                xSemaphoreTake(xYawFlipSemaphore, 10);
                //desired_yaw += 180;
                if (desired_yaw >= 0) {
                    desired_yaw = desired_yaw - 180;
                } else {
                    desired_yaw = DEGREES_CIRCLE - 180 + desired_yaw;
                }
                desired_alt += 10;
                xQueueOverwrite(xYawDesQueue, &desired_yaw);
                xQueueOverwrite(xAltDesQueue, &desired_alt);
            }else {
                /*
                 * Call the down button handler to decrease target altitude by 10%
                 */
                downButtonPush();
            }
        }
        if(checkButton(LEFT) == PUSHED)
        {
            leftButtonPush();
        }
        if(checkButton(RIGHT) == PUSHED)
        {
            rightButtonPush();
        }

        vTaskDelayUntil(&ui16LastTaskTime, ui32ButtonsDelay/portTICK_RATE_MS); // Wait for the required amount of time to check back.
    }
}

/*
 * FreeRTOS task which polls the switches
 */
void
SwitchesCheck(void *pvParameters)
{
    portTickType ui16LastTaskTime;
    uint32_t ui32SwitchDelay = 25;
    uint32_t state;
    uint16_t R_PREV = GPIOPinRead(SW_PORT_BASE, R_SW_PIN);
    uint16_t L_PREV = GPIOPinRead(SW_PORT_BASE, L_SW_PIN);

    ui16LastTaskTime = xTaskGetTickCount(); // Get the current tick count.

    while(1) {
        xQueuePeek(xFSMQueue, &state, 10);
        if(GPIOPinRead(SW_PORT_BASE, R_SW_PIN) != R_PREV)
        {
            R_PREV = GPIOPinRead(SW_PORT_BASE, R_SW_PIN);
            if(R_PREV == R_SW_PIN){
                UARTSend ("R_SW High\n\r");
                state = TAKEOFF;

            }else{
                UARTSend ("R_SW Low\n\r");
                if(state == FLYING){
                    state = LANDING;
                }
            }

            xQueueOverwrite(xFSMQueue, &state);
        }
        if(GPIOPinRead(SW_PORT_BASE, L_SW_PIN) != L_PREV)
        {
            L_PREV = GPIOPinRead(SW_PORT_BASE, L_SW_PIN);
            if(L_PREV == L_SW_PIN){
                UARTSend ("L_SW High\n\r");
            }else{
                UARTSend ("L_SW Low\n\r");
            }
        }
        vTaskDelayUntil(&ui16LastTaskTime, ui32SwitchDelay/portTICK_RATE_MS);
    }
}
