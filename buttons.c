/* ****************************************************************
 * buttons.c
 *
 * Source file for the buttons module
 * Supports buttons on the Tiva/Orbit.
 * Comprises of initialisers and button checks
 *
 * Based on buttons4.c - P.J. Bones, UCECE
 *
 * Further based on BUTTONS.c
 * Thu AM Group 18
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Derrick Edward      18017758
 * Last modified: 29/05/2019
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 19/08/2020
 *
 * ***************************************************************/

#include "buttons.h"

static bool btn_state[NUM_BTNS];    // Corresponds to the electrical state
static bool btn_normal[NUM_BTNS];   // Corresponds to the electrical state
static bool btn_flag[NUM_BTNS];
static uint8_t btn_count[NUM_BTNS];


/*
 * Function:    vBtnTimerCallback
 * -------------------------------
 * Handler for the button timer.
 * When the button timer expires, this function
 * resets the timer ID.
 *
 * @params:
 *      - TimeHandle_t xTimer - the timer being handled
 * @return:
 *      - NULL
 * ---------------------
 */
void
vDblBtnTimerCallback( TimerHandle_t xTimer )
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
    if( ulCount >= TIMER_EXPIRY )
    {
        /* Do not use a block time if calling a timer API function
        from a timer callback function, as doing so could cause a
        deadlock! */
        vTimerSetTimerID( xTimer, ( void * ) reset ); //( void * ) ulCount
        xTimerStop( xTimer, reset );
    }
    UARTSend("Btn Timer Callback\n\r");
}



/*
 * Function:    initBtns
 * ---------------------
 * Initialises the Tiva board and Orbit Boosterback's
 * buttons and switches for user input.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
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
    SysCtlPeripheralEnable(R_BTN_PERIPH);
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // Unlock PF0 for the right button:
    GPIO_PORTF_CR_R |= GPIO_PIN_0; // PF0 unlocked
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

/*
 * Function:    updateButtons
 * ---------------------
 * Polls the buttons to check for button presses.
 * This includes a FSM where a state change occurs
 * only after NUM_BTN_POLLS consecutive polls have
 * read the pin in the opposite condition, before the state changes and
 * a flag is set.  Set NUM_BTN_POLLS according to the polling rate.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
static void
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
                btn_flag[i] = true;	   // Reset by call to checkButton()
                btn_count[i] = 0;
            }
        }
        else
            btn_count[i] = 0;
    }
}

/*
 * Function:    checkButton
 * ---------------------
 * Function that returns if the logic state of a button has
 * changed since last called.
 *
 * @params:
 *      - uint8_t btnName - the button being checked
 * @return:
 *      - uint8_t btnState - The chance in the button's state
 * ---------------------
 */
static uint8_t
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
 * Function:    upButtonPush
 * ---------------------
 * Handler for the up button.
 * Increments the helicopter's desired altitude by 10%
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
static void
upButtonPush(void)
{
    int32_t alt_desired = 0;

    UARTSend ("Up\n");
    xQueuePeek(xAltDesQueue, &alt_desired, TICKS_TO_WAIT); // Retrieve desired altitude data from the RTOS queue

    alt_desired += ALT_CHANGE;

    // Check upper limits of the altitude when left button is pressed
    if (alt_desired > MAX_ALT)
    {
        alt_desired = MAX_ALT;
    }
    xQueueOverwrite(xAltDesQueue, &alt_desired); // Update the RTOS altitude reference queue
}

/*
 * Function:    downButtonPush
 * ---------------------
 * Handler for the down button.
 * Decrements the helicopter's desired altitude by 10%
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
static void
downButtonPush(void)
{
    int32_t alt_desired = 0;

    UARTSend ("Down\n");
    xQueuePeek(xAltDesQueue, &alt_desired, TICKS_TO_WAIT); // Retrieve desired altitude data from the RTOS queue
    alt_desired -= ALT_CHANGE;

    // Check lower limits of the altitude when left button is pressed
    if (alt_desired < MIN_ALT)
    {
        alt_desired = MIN_ALT;
    }
    xQueueOverwrite(xAltDesQueue, &alt_desired); // Update the RTOS altitude reference queue
}

/*
 * Function:    rightButtonPush
 * ---------------------
 * Handler for the right button.
 * Increments the helicopter's desired altitude by 10%
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
static void
rightButtonPush(void)
{
    int32_t yaw_desired = 0;

    UARTSend ("Right\n");
    xQueuePeek(xYawDesQueue, &yaw_desired, TICKS_TO_WAIT); // Retrieve desired yaw data from the RTOS queue

    // Check upper limits of the yaw when left button is pressed
    if (yaw_desired <= (MAX_YAW - YAW_CHANGE)) {
    yaw_desired = yaw_desired + YAW_CHANGE;
    } else {
        yaw_desired = -DEGREES_CIRCLE + YAW_CHANGE + yaw_desired;
    }
    xQueueOverwrite(xYawDesQueue, &yaw_desired); // Update the RTOS yaw reference queue
}

/*
 * Function:    leftButtonPush
 * ---------------------
 * Handler for the left button.
 * decrements the helicopter's desired altitude by 10%
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
static void
leftButtonPush(void)
{
    int32_t yaw_desired = 0;

    UARTSend ("Left\n");
    xQueuePeek(xYawDesQueue, &yaw_desired, TICKS_TO_WAIT); // Retrieve desired yaw data from the RTOS queue

    // Check upper limits of the yaw if right button is pressed
    if (yaw_desired >= (MIN_YAW + YAW_CHANGE)) {
        yaw_desired = yaw_desired - YAW_CHANGE;
    } else {
        yaw_desired = DEGREES_CIRCLE - YAW_CHANGE + yaw_desired;
    }
    xQueueOverwrite(xYawDesQueue, &yaw_desired); // Update the RTOS yaw reference queue
}

/*
 * Function:    ButtonsCheck
 * ---------------------
 * FreeRTOS task which polls the buttons to check for button presses
 * For each button the following procedure is run:
 *
 * If Button State is PUSHED:
 *      Update Target Altitude/Yaw accordingly
 *      If Target Alt/Yaw is now beyond the limits:
 *          Update targets to be at limit (0->100 for Alt, -180->179 for Yaw).
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
ButtonsCheck(void *pvParameters)
{
    portTickType ui16LastTaskTime;
    uint32_t inUpTimeLoop;
    uint32_t inYawTimeLoop;
    int32_t desired_alt;
    int32_t desired_yaw;

    ui16LastTaskTime = xTaskGetTickCount(); // Get the current tick count.

    // Loop forever.
    while(1)
    {
        inUpTimeLoop = ( uint32_t ) pvTimerGetTimerID( xUpBtnTimer );
        inYawTimeLoop = ( uint32_t ) pvTimerGetTimerID( xDownBtnTimer );

        xQueuePeek(xYawDesQueue, &desired_yaw, TICKS_TO_WAIT);
        xQueuePeek(xAltDesQueue, &desired_alt, TICKS_TO_WAIT);
        /*
         * Check if any buttons have been pressed. Update button state.
         */
        updateButtons();

        if(checkButton(UP) == PUSHED)
        {
            if(inUpTimeLoop == 0) { // check to see if the timer has ran out
                vTimerSetTimerID(xUpBtnTimer, (void *) 1);
                xTimerStart(xUpBtnTimer, TICKS_TO_WAIT); // Restarts timer
            } else {
                xSemaphoreGive(xUpBtnSemaphore);
            }

            if (uxSemaphoreGetCount(xUpBtnSemaphore) == 1) {
                xSemaphoreTake(xUpBtnSemaphore, TICKS_TO_WAIT);
                desired_alt = MODE_1_ALT;
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
                vTimerSetTimerID(xDownBtnTimer, (void *) 1);
                xTimerStart(xDownBtnTimer, TICKS_TO_WAIT); // Restarts timer
            } else {
                xSemaphoreGive(xYawFlipSemaphore);
            }

            if (uxSemaphoreGetCount(xYawFlipSemaphore) == 1) {
                xSemaphoreTake(xYawFlipSemaphore, TICKS_TO_WAIT);
                //desired_yaw += 180;
                if (desired_yaw >= 0) {
                    desired_yaw = desired_yaw - MODE_2_YAW_CHANGE;
                } else {
                    desired_yaw = DEGREES_CIRCLE - MODE_2_YAW_CHANGE + desired_yaw;
                }
                desired_alt += ALT_CHANGE;
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

        vTaskDelayUntil(&ui16LastTaskTime, INPUT_PERIOD/portTICK_RATE_MS); // Wait for the required amount of time to check back.
    }
}

/*
 * Function:    SwitchesCheck
 * ---------------------
 * FreeRTOS task which polls the switches to check for switch pushes
 * For each button the following procedure is run:
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
SwitchesCheck(void *pvParameters)
{
    portTickType ui16LastTaskTime;
    uint32_t state;
    uint16_t R_PREV = GPIOPinRead(SW_PORT_BASE, R_SW_PIN);
    uint16_t L_PREV = GPIOPinRead(SW_PORT_BASE, L_SW_PIN);

    ui16LastTaskTime = xTaskGetTickCount(); // Get the current tick count.

    while(1) {
        xQueuePeek(xFSMQueue, &state, TICKS_TO_WAIT);
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
        vTaskDelayUntil(&ui16LastTaskTime, INPUT_PERIOD/portTICK_RATE_MS);
    }
}
