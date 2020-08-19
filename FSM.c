/* ****************************************************************
 * FSM.c
 *
 * Source file for the finite state machine (FSM) module
 * Control the tasks operated based on if the helicopter is trying
 * to take off, hover, land or is landed.
 *
 * Based on FSM.c
 * Tue AM Group 1
 * Creators: Brendain Hennessy   57190084
 *           Sarah Kennelly      76389950
 *           Matt Blake          58979250
 * Last modified: 9/05/2019
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 19/08/2020
 *
 * ***************************************************************/

#include "FSM.h"


/*
 * Function:    GetStackUsage
 * ---------------------------
 * Function that calculates and transmits over UART
 * statistics about the stack usage.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
GetStackUsage(void)
{
    char cMessage[17];

    uint32_t StatusLED_stack;
    uint32_t OLEDDisp_stack;
    uint32_t UARTDisp_stack;
    uint32_t BtnCheck_stack;
    uint32_t SwitchCheck_stack;
    uint32_t ADCTrig_stack;
    uint32_t ADCMean_stack;
    uint32_t MainPWM_stack;
    uint32_t TailPWM_stack;
    uint32_t FSMTask_stack;

    // Retrieve stack usage information from each task
    StatusLED_stack   = uxTaskGetStackHighWaterMark(StatLED);
    OLEDDisp_stack    = uxTaskGetStackHighWaterMark(OLEDDisp);
    UARTDisp_stack    = uxTaskGetStackHighWaterMark(UARTDisp);
    BtnCheck_stack    = uxTaskGetStackHighWaterMark(BtnCheck);
    SwitchCheck_stack = uxTaskGetStackHighWaterMark(SwiCheck);
    ADCTrig_stack     = uxTaskGetStackHighWaterMark(ADCTrig);
    ADCMean_stack     = uxTaskGetStackHighWaterMark(ADCMean);
    MainPWM_stack     = uxTaskGetStackHighWaterMark(MainPWM);
    TailPWM_stack     = uxTaskGetStackHighWaterMark(TailPWM);
    FSMTask_stack     = uxTaskGetStackHighWaterMark(FSMTask);

    // Send stack information via UART
    usnprintf(cMessage, sizeof(cMessage), "StatusLED unused: %d words\n",   StatusLED_stack);
    UARTSend(cMessage);
    usnprintf(cMessage, sizeof(cMessage), "OLEDDisp unused: %d words\n",    OLEDDisp_stack);
    UARTSend(cMessage);
    usnprintf(cMessage, sizeof(cMessage), "OLEDDisp unused: %d words\n",    UARTDisp_stack);
    UARTSend(cMessage);
    usnprintf(cMessage, sizeof(cMessage), "BtnCheck Unused: %d words\n",    BtnCheck_stack);
    UARTSend(cMessage);
    usnprintf(cMessage, sizeof(cMessage), "SwiCheck Unused: %d words\n",    SwitchCheck_stack);
    UARTSend(cMessage);
    usnprintf(cMessage, sizeof(cMessage), "ADCTrig Unused: %d words\n",     ADCTrig_stack);
    UARTSend(cMessage);
    usnprintf(cMessage, sizeof(cMessage), "ADCMean Unused: %d words\n",     ADCMean_stack);
    UARTSend(cMessage);
    usnprintf(cMessage, sizeof(cMessage), "MainPWM Unused: %d words\n",     MainPWM_stack);
    UARTSend(cMessage);
    usnprintf(cMessage, sizeof(cMessage), "TailPWM Unused: %d words\n",     TailPWM_stack);
    UARTSend(cMessage);
    usnprintf(cMessage, sizeof(cMessage), "FSMTask Unused: %d words\n",     FSMTask_stack);
    UARTSend(cMessage);
}


/*
 * Function:    vLandTimerCallback
 * --------------------------------
 * Callback function for the timer started during the
 * landing sequence.
 * Increases timer ID each time function is called.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void vLandTimerCallback( TimerHandle_t xTimer )
{
    uint32_t ulCount;
    UARTSend("Landing Timer Callback\n\r");

    ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );
    ulCount++;
    vTimerSetTimerID( xTimer, (void *) ulCount );
}


/*
 * Function:    findYawRef
 * ------------------------
 * Disables the PWM control, buttons, and switches.
 * Sets the main PWM to be 50% duty cycle in order for the
 * helicopter to spin.
 * Once the yaw reference flag has been set by an interrupt,
 * all tasks resume.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
findYawRef(void)
{
    UARTSend("Finding Ref\n\r");
    setRotorPWM(FIND_REF_PWM_MAIN, 1);      // Set the main rotor to on, the torque from the main rotor should work better than using the tail, have to test and actually see whats best
    setRotorPWM(FIND_REF_PWM_TAIL, 0);
}


/*
 * Function:    takeoff
 * ---------------------
 * If the reference has not be found, the findYawRef function is
 * called.
 * If the reference flag has been set, the helicopter ascends to
 * 20% height, and rotates to 0 degrees yaw.
 * Once this position has been reached, the state changes to FLYING.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
takeoff(void)
{
    int32_t yaw;
    int32_t alt;
    int32_t desired_yaw = 0;
    int32_t desired_alt = 20;
    int32_t found_yaw;
    int32_t state;

    found_yaw = xEventGroupGetBits(xFoundYawReference);

    if(!found_yaw) {                // If the reference yaw has not been found
        vTaskSuspend(MainPWM);      // Suspend the PWM control systems until ref is found
        vTaskSuspend(TailPWM);
        vTaskSuspend(BtnCheck);     // Disable user input while the ref is being found
        vTaskSuspend(SwiCheck);     // Stop checking the switches until takeoff is complete
        findYawRef();               // Find the reference yaw
    } else {
        xQueueOverwrite(xYawDesQueue, &desired_yaw); // Rotate to reference yaw
        xQueueOverwrite(xAltDesQueue, &desired_alt); // Ascend to 20% altitude
        vTaskResume(MainPWM);       // Re-enable the control system
        vTaskResume(TailPWM);
        vTaskResume(BtnCheck);      // Re-enable user input
        vTaskResume(SwiCheck);
        xQueuePeek(xAltMeasQueue, &alt, 10); // Retrieve the current altitude value
        xQueuePeek(xYawMeasQueue, &yaw, 10); // Retrieve the current yaw value

        if ((yaw > (-YAW_TOLERANCE)) && (yaw < YAW_TOLERANCE)) { // If reached desired yaw

            if (alt > (desired_alt - ALT_TOLERANCE) && (alt < (desired_alt + ALT_TOLERANCE))) { // If reached desired altitude

                state = FLYING;
                xQueueOverwrite(xFSMQueue, &state); // Set state to hover mode
            }
        }
    }
}


/*
 * Function:    hover
 * -------------------
 * Basic flying mode. All tasks are functional.
 * Movement is controlled by the GPIO buttons and the PID
 * controller.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
hover(void)
{
    // Resume suspended tasks
    vTaskResume(MainPWM);
    vTaskResume(TailPWM);
    vTaskResume(BtnCheck);
    vTaskResume(SwiCheck);
}


/*
 * Function:    land
 * ------------------
 * Function that sets the desired altitude to 10%, and then
 * decreases this value by 2% every second.
 * Once the desired position is reached, the state is changed to
 * LANDED.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
land(void)
{
    int32_t ref_yaw = 0;
    int32_t yaw;
    int32_t meas;
    int32_t state = LANDING;
    static int32_t descent = 30;
    static int32_t prev_timerID = 0;

    int32_t timerID = ( uint32_t ) pvTimerGetTimerID( xLandingTimer );

    vTaskSuspend(BtnCheck); // Disable changes to yaw and altitude while landing
    vTaskSuspend(SwiCheck); // Disable changes to the helicopter state while landing

    xQueueOverwrite(xYawDesQueue, &ref_yaw);
    xQueuePeek(xAltMeasQueue, &meas, 10);
    xQueuePeek(xYawMeasQueue, &yaw, 10);

    if (timerID == 0){
        xTimerStart(xLandingTimer, 10); // Starts timer
        vTimerSetTimerID( xLandingTimer, (void *) 1 );
        descent = meas;
    }else if ((timerID != prev_timerID) && (meas <= descent)){
        descent -= 10;
        if (descent <= 0){ // when landing the heli gives up sometimes and jsut cuts power before reaching the ground
            descent = 0;
        }
    }
    prev_timerID = timerID;

    if (descent < 2 && meas <= 1 && (yaw <= 2) && (yaw >= -2)) {
        UARTSend("LANDING_SEQ_FIN\n\r");
        state = LANDED;
        vTimerSetTimerID( xLandingTimer, (void *) 0 );
        prev_timerID = 0;
        xTimerStop( xLandingTimer, 0 );
        vTaskResume(SwiCheck);
    }
    xQueueOverwrite(xAltDesQueue, &descent);
    xQueueOverwrite(xFSMQueue, &state);
}


/*
 * Function:    landed
 * --------------------
 * Disables all input with the exception of the switches.
 * Helicopter is in an idle state.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void landed(void)
{
    // Suspend unwanted tasks
    vTaskSuspend(MainPWM); // Suspend the control system while landed
    vTaskSuspend(TailPWM);
    vTaskSuspend(BtnCheck); // Disable changes to yaw and altitude while landed
    vTaskResume(SwiCheck); // Resume checking the switches

    // Set motor duty cycles to minimum
    setRotorPWM(MIN_DUTY, 1);
    setRotorPWM(MIN_DUTY, 0);

    // Reset error on controllers
    g_alt_controller.previousError   = 0;
    g_yaw_controller.previousError   = 0;
    g_alt_controller.integratedError = 0;
    g_yaw_controller.integratedError = 0;

    // Get max stack usage
    GetStackUsage();
}

/*
 * Function:    FSM
 * -----------------
 * FreeRTOS task that periodically checks the current state of the
 * helicopter and runs the appropriate function.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
FSM(void *pvParameters) {

    uint32_t state = 0;

    while(1)
    {
        xQueuePeek(xFSMQueue, &state, 10);      // Read the current flight mode/state.
        switch(state) {
            case LANDED:
                landed();
                break;
            case TAKEOFF:
                takeoff();
                break;
            case FLYING:
                hover();
                break;
            case LANDING:
                land();
                break;

            default:
                UARTSend("FSM Error\n");
        }


        vTaskDelay(FSM_PERIOD / portTICK_RATE_MS);

    }

}
