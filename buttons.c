/* ****************************************************************
 * BUTTONS.c
 *
 * Module for the D-Pad Buttons (U/D/L/R)
 * Supports buttons on the Tiva/Orbit.
 * Comprises of initialisers and button checks
 *
 * Author: P.J. Bones UCECE
 * Edited: Derrick Edward, Grayson Mynott, Ryan Earwaker
 * Thu AM Group 18
 * Last modified:  29.05.2019
 *
 * ***************************************************************/

#include "buttons.h"

// *******************************************************
// Globals to module
// *******************************************************
static bool btn_state[NUM_BTNS];	// Corresponds to the electrical state
static uint8_t btn_count[NUM_BTNS];
static bool btn_flag[NUM_BTNS];
static bool btn_normal[NUM_BTNS];   // Corresponds to the electrical state

// *******************************************************
// initButtons: Initialise the variables associated with the set of buttons
// defined by the constants in the buttons2.h header file.
void initBtns(void)
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
    GPIOPinTypeGPIOInput(SW_PORT_BASE, L_SW_PIN | R_SW_PIN);
    GPIOPadConfigSet(SW_PORT_BASE, L_SW_PIN | R_SW_PIN, GPIO_STRENGTH_2MA,
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
void updateButtons(void)
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
uint8_t checkButton(uint8_t btnName)
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


void
ButtonsCheck(void *pvParameters)
{

    /*
     * For each button the following procedure is run:
     *
     * If Button State is PUSHED:
     *      Update Target Altitude/Yaw accordingly
     *      If Target Alt/Yaw is now beyond the limits:
     *          Update targets to be at limit (0-100 for Alt, -180-180 for Yaw).
     */

    portTickType ui16LastTime;
    uint32_t ui32SwitchDelay = 25;
    uint8_t state = 0;
    uint16_t L_PREV = GPIOPinRead(SW_PORT_BASE, L_SW_PIN);
    uint16_t R_PREV = GPIOPinRead(SW_PORT_BASE, R_SW_PIN);
    uint16_t TARGET_ALT;

    // Get the current tick count.
    ui16LastTime = xTaskGetTickCount();

    // Loop forever.
    while(1)
    {
        updateButtons();
        if(xSemaphoreTake(xAltMutex, 0/portTICK_RATE_MS) == pdPASS){

            if(checkButton(UP) == PUSHED)               // INCREASE ALTITUDE
            {
                state = 1;
                UARTSend ("Up\n");
                TARGET_ALT += 10;

                if (TARGET_ALT > 100)
                {
                    TARGET_ALT = 100;
                }
                char cMessage[5];
                usnprintf(cMessage, sizeof(cMessage), "%d\n", TARGET_ALT);
                UARTSend(cMessage);

                if(xQueueOverwrite(xAltBtnQueue, &state) != pdPASS) {
                    UARTSend("AltBtnQueue failed.\n");
                    while(1){}
                }
            }

            if(checkButton(DOWN) == PUSHED)               // DECREASE ALTITUDE
            {
                state = 0;
                UARTSend ("Down\n");

                //TARGET_ALT -= 10;
                //if (TARGET_ALT <= 0)
                //{
                //    TARGET_ALT = 0;
                //}

                if(xQueueOverwrite(xAltBtnQueue, &state) != pdPASS) {
                    // Error. The queue should never be full. If so print the error message on UART and wait for ever.
                    UARTSend("AltBtnQueue failed.\n");
                    while(1){}
                }
            }
            while(xSemaphoreGive(xAltMutex) != pdPASS){
                UARTSend("Couldn't give Alt Mutex\n");
            }
        }


        if(xSemaphoreTake(xYawMutex, 0/portTICK_RATE_MS) == pdPASS){
            if(checkButton(LEFT) == PUSHED)
            {
                // ROTATE ANTI-CLOCKWISE
                UARTSend ("Left\n");
                //TARGET_YAW += 15;
                //if (TARGET_YAW >= 180)
                //{
                //    TARGET_YAW = -180;
                //}

                if(xQueueOverwrite(xYawBtnQueue, &state) != pdPASS) {
                    // Error. The queue should never be full. If so print the error message on UART and wait for ever.
                    UARTSend("YawBtnQueue failed.\n");
                    while(1){}
                }
            }
            if(checkButton(RIGHT) == PUSHED)
            {
                // ROTATE CLOCKWISE
                UARTSend ("Right\n");
                //TARGET_YAW -= 15;
                //if (TARGET_YAW <= -180)
                //{
                //    TARGET_YAW = 180;
                //}

                if(xQueueOverwrite(xYawBtnQueue, &state) != pdPASS) {
                    // Error. The queue should never be full. If so print the error message on UART and wait for ever.
                    UARTSend("YawBtnQueue failed.\n");
                    while(1){}
                }
            }

            while(xSemaphoreGive(xYawMutex) != pdPASS){
                UARTSend("Couldn't give Yaw Mutex\n");
            }
        }

        if(GPIOPinRead(SW_PORT_BASE, L_SW_PIN) != L_PREV)
        {
            L_PREV = GPIOPinRead(SW_PORT_BASE, L_SW_PIN);
            UARTSend ("Left Switch\n");
        }
        if(GPIOPinRead(SW_PORT_BASE, R_SW_PIN) != R_PREV)
        {
            R_PREV = GPIOPinRead(SW_PORT_BASE, R_SW_PIN);
            UARTSend ("Right Switch\n");
        }

        // Wait for the required amount of time to check back.
        vTaskDelayUntil(&ui16LastTime, ui32SwitchDelay / portTICK_RATE_MS);
    }
}
