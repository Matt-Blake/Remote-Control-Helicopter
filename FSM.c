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

typedef enum HELI_STATE {LANDED = 0, TAKEOFF = 1, FLYING = 2, LANDING = 3} HELI_STATE;

#define FSM_PERIOD              200

//****************************************************************************
//Check if found the reference yaw, if it has then set found reference to 1 and
//reset the integrator error and update yaw reference
//****************************************************************************
void
findYawRef(void)
{
    int32_t PWM_main = 50; // place holder for now
    int32_t found_yaw;
    int32_t ref_yaw = 0;

    vTaskSuspend(MainPWM); // suspend the control system until ref is found
    vTaskSuspend(TailPWM);
    vTaskSuspend(BtnCheck);
    vTaskSuspend(SwitchCheck);

    found_yaw = xEventGroupGetBits(xFoundYawReference);
    UARTSend("Finding Ref\n");

    if(found_yaw) {
        vTaskResume(MainPWM); // Re-enable the control system
        vTaskResume(TailPWM);
        vTaskResume(BtnCheck);
        vTaskResume(SwitchCheck);
        xEventGroupClearBitsFromISR(xFoundYawReference, YAW_REFERENCE_FLAG);
        xQueueOverwrite(xYawDesQueue, &ref_yaw);

    } else { // finding ref mode
        setRotorPWM(PWM_main, 1); // set the main rotor to on, the torque from the main rotor should work better than using the tail, have to test and actually see whats best
    }
}

//****************************************************************************
// Helicopter rotates to reference yaw and ascends to 20% altitude
//****************************************************************************
void
takeoff(void)
{
    int32_t yaw;
    int32_t alt;
    int32_t desired_yaw = 0;
    int32_t desired_alt = 20;
    int32_t found_yaw;
    int32_t state;

    vTaskSuspend(MainPWM); // suspend the control system until ref is found
    vTaskSuspend(TailPWM);
    vTaskSuspend(BtnCheck);
    vTaskSuspend(SwitchCheck);

    found_yaw = xEventGroupGetBits(xFoundYawReference);

    if(!found_yaw) { // If the reference yaw has been found
        findYawRef(); // Find the reference yaw
    } else {
        vTaskResume(MainPWM); // Re-enable the control system
        vTaskResume(TailPWM);
        xQueueOverwrite(xYawDesQueue, &desired_alt); // Ascend to 20 % altitude
        xQueueOverwrite(xYawDesQueue, &desired_yaw); // Rotate to reference yaw
        xQueuePeek(xAltMeasQueue, &alt, 10); // Retrieve the current altitude value
        xQueuePeek(xYawMeasQueue, &yaw, 10); // Retrieve the current yaw value
        if ((yaw > (-YAW_TOLERANCE)) || (yaw < YAW_TOLERANCE)) { // If reached desired yaw
            if ((alt > (desired_alt - ALT_TOLERANCE)) || (alt < (desired_alt + ALT_TOLERANCE))) { // If reached desired altitude
                state = FLYING;
                xQueueOverwrite(xFSMQueue, &state); // Set state to hover mode
            }
        }
    }
}

//****************************************************************************
// Helicopter tracks reference altitude and yaw
//****************************************************************************
void hover(void)
{
    // Resume suspended tasks
    vTaskResume(MainPWM);
    vTaskResume(TailPWM);
    vTaskResume(BtnCheck);
    vTaskResume(SwitchCheck);
}

//****************************************************************************
// Helicopter rotates to reference yaw then incrementally descends to 0 altitude
//****************************************************************************
void
land(void)
{
    int32_t yaw = 0;
    int32_t meas;
    int32_t state = LANDING;
    static int32_t descent = 10;
    static int32_t prev_timerID = 0;

    int32_t timerID = ( uint32_t ) pvTimerGetTimerID( xLandingTimer );

    vTaskSuspend(BtnCheck); // Disable changes to yaw and altitude while landing
    vTaskSuspend(SwitchCheck);

    xQueueOverwrite(xYawDesQueue, &yaw);
    xQueuePeek(xAltMeasQueue, &meas, 10);

    if (timerID == 0){
        xTimerStart(xLandingTimer, 10); // Starts timer
        vTimerSetTimerID( xLandingTimer, (void *) 1 );
        descent = 10;
    }else if ((timerID != prev_timerID) && (meas <= 10)){
        descent = descent - 2;
    }
    prev_timerID = timerID;

    if (descent == 0 && meas <= 1) {
        state = LANDED;
        vTimerSetTimerID( xLandingTimer, (void *) 0 );
        prev_timerID = 0;
        xTimerStop( xLandingTimer, 0 );
        vTaskResume(SwitchCheck);
    }
    xQueueOverwrite(xAltDesQueue, &descent);
    xQueueOverwrite(xFSMQueue, &state);
}

//****************************************************************************
//Checks the switch and updates the display and UART after the helicoptor is
//landed
//****************************************************************************
void landed(void)
{
    vTaskSuspend(MainPWM); // Suspend the control system while landed
    vTaskSuspend(TailPWM);
    vTaskSuspend(BtnCheck); // Disable changes to yaw and altitude while landed
    //vTaskResume(SwitchCheck);
}

//****************************************************************************
//Calls the appropriate function for the current state
// the slider will switch between take_off and landing. The transition will be dependent on current state
//****************************************************************************
void
FSM(void *pvParameters) {

    uint32_t state = 0;

    while(1)
    {
        xQueuePeek(xFSMQueue, &state, 10);
        switch(state) {
            case LANDED:
//                UARTSend("Landed\n");
                landed();
                break;
            case TAKEOFF:
//                UARTSend("Taking off\n");
                takeoff();
                break;
            case FLYING:
//                UARTSend("Flying\n");
                hover();
                break;

            case LANDING:
//                UARTSend("Landing\n");
                land();
                break;

            default:
                UARTSend("FSM Error\n");
        }

        vTaskDelay(FSM_PERIOD / portTICK_RATE_MS);

    }

}
