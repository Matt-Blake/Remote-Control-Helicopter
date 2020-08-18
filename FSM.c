/*
 * FSM.c - Helicopter finite state machine
 *
 * Tue am Group 1
 * Creators: Brendain Hennessy   57190084
 *           Sarah Kennelly      76389950
 *           Matt Blake          58979250
 * Last modified: 9/05/2019
 *
 * pidController.c - This code was based off the FSM.c code from ENEL361.
 * It has been edited to include FreeRTOS functionality and has two extra
 * modes allowing the helicopter to reach the mid-point altitude and turn
 * 180 degrees.
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 10/08/2020
 */

#include "FSM.h"

#define FSM_PERIOD              200

typedef enum HELI_STATE {LANDED = 0, TAKEOFF = 1, FLYING = 2, LANDING = 3} HELI_STATE;

TimerHandle_t xLandingTimer;
TaskHandle_t FSMTask;

QueueHandle_t xFSMQueue;


// Functions
/*
 * Calculate the maximum stack usage all of the RTOS tasks.
 */
void
GetStackUsage(void)
{
    uint32_t Blinky_stack;
    uint32_t OLEDDisp_stack;
    uint32_t BtnCheck_stack;
    uint32_t SwitchCheck_stack;
    uint32_t ADCTrig_stack;
    uint32_t ADCMean_stack;
    uint32_t MainPWM_stack;
    uint32_t TailPWM_stack;
    uint32_t FSMTask_stack;

    // Retrieve stack usage information from each task
    OLEDDisp_stack    = uxTaskGetStackHighWaterMark(OLEDDisp);
    BtnCheck_stack    = uxTaskGetStackHighWaterMark(BtnCheck);
    SwitchCheck_stack = uxTaskGetStackHighWaterMark(SwitchCheck);
    ADCTrig_stack     = uxTaskGetStackHighWaterMark(ADCTrig);
    ADCMean_stack     = uxTaskGetStackHighWaterMark(ADCMean);
    MainPWM_stack     = uxTaskGetStackHighWaterMark(MainPWM);
    TailPWM_stack     = uxTaskGetStackHighWaterMark(TailPWM);
    FSMTask_stack     = uxTaskGetStackHighWaterMark(FSMTask);
    Blinky_stack      = uxTaskGetStackHighWaterMark(Blinky);
}

/*
 * WRITE DESCRIPTION
 */
void vLandTimerCallback( TimerHandle_t xTimer )
{
    uint32_t ulCount;
    //UARTSend("Landing Timer Callback\n");

    ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );
    ulCount++;
    vTimerSetTimerID( xTimer, (void *) ulCount );
}

//****************************************************************************
//Check if found the reference yaw, if it has then set found reference to 1 and
//reset the integrator error and update yaw reference
//****************************************************************************
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
    int32_t PWM_Main = 15; // place holder for now
    int32_t PWM_Tail = 00; // place holder for now


    vTaskSuspend(MainPWM); // suspend the control system until ref is found
    vTaskSuspend(TailPWM);
    vTaskSuspend(BtnCheck);
    vTaskSuspend(SwitchCheck);

    UARTSend("Finding Ref\n");
    setRotorPWM(PWM_Main, 1); // Set the main rotor to on, the torque from the main rotor should work better than using the tail, have to test and actually see whats best
    setRotorPWM(PWM_Tail, 0);   // Set the tail rotor to on
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

    UARTSend("Take-Off\r\n");

    if(!found_yaw) { // If the reference yaw has been found
        vTaskSuspend(MainPWM); // suspend the control system until ref is found
        vTaskSuspend(TailPWM);
        vTaskSuspend(BtnCheck);
        vTaskSuspend(SwitchCheck);
        findYawRef(); // Find the reference yaw
    } else {
        vTaskResume(MainPWM); // Re-enable the control system
        vTaskResume(TailPWM);
        vTaskResume(BtnCheck);
        vTaskResume(SwitchCheck);
        xQueueOverwrite(xYawDesQueue, &desired_yaw); // Rotate to reference yaw
        xQueueOverwrite(xAltDesQueue, &desired_alt); // Ascend to 20 % altitude
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
    vTaskResume(SwitchCheck);
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
    vTaskSuspend(SwitchCheck);

    xQueueOverwrite(xYawDesQueue, &ref_yaw);
    xQueuePeek(xAltMeasQueue, &meas, 10);
    xQueuePeek(xYawMeasQueue, &yaw, 10);

    if (timerID == 0){
        xTimerStart(xLandingTimer, 10); // Starts timer
        vTimerSetTimerID( xLandingTimer, (void *) 1 );
        descent = meas;
    }else if ((timerID != prev_timerID) && (meas <= descent)){
        descent -= 10;
        if (descent <= 0){ // when landing the heli gives up sometimes and jsut cuts power before reaching the ground                   Alt goes from 8-0
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
        vTaskResume(SwitchCheck);
    }
    xQueueOverwrite(xAltDesQueue, &descent);
    xQueueOverwrite(xFSMQueue, &state);
}


/*
 * Function:    landed
 * ------------------------
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

    //vTaskResume(SwitchCheck);
}

/*
 * Function:    FSM
 * ------------------------
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
