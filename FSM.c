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

typedef enum HELI_STATE {FIND_REF = 0, LANDED = 1, FLYING = 2, LANDING = 3} HELI_STATE;

#define FSM_PERIOD              100

//****************************************************************************
//Check if found the reference yaw, if it has then set found reference to 1 and
//reset the integrator error and update yaw reference
//****************************************************************************
void
findYawRef(void)
{
    int32_t PWM_main = 50; // place holder for now
    int32_t state = 2;
    int32_t found_yaw;

    vTaskSuspend(MainPWM); // suspend the control system until ref is found
    vTaskSuspend(TailPWM);
    vTaskSuspend(BtnCheck);
    //vTaskResume(SwitchCheck);

    found_yaw = xEventGroupGetBits(xFoundYawReference);

    if(found_yaw) {
        vTaskResume(MainPWM); // re enable the control system
        vTaskResume(TailPWM);
        vTaskResume(BtnCheck);
        xQueueOverwrite(xFSMQueue, &state);
        xEventGroupClearBitsFromISR(xFoundYawReference, YAW_REFERENCE_FLAG);

    } else { // finding ref mode
        setRotorPWM(PWM_main, 1); // set the main rotor to on, the torque from the main rotor should work better than using the tail, have to test and actually see whats best
    }
}

void
land(void)
{
    int32_t yaw = 0;
    int32_t mes;
    int32_t state = 3;
    static int32_t descent = 10;

    int32_t timerID = ( uint32_t ) pvTimerGetTimerID( xTimerLand );

    vTaskSuspend(MainPWM); // Suspend the control system while landed
    vTaskSuspend(TailPWM);
    vTaskSuspend(BtnCheck); // Disable changes to yaw and altitude while landing
    //vTaskResume(SwitchCheck);

    xQueueOverwrite(xYawDesQueue, &yaw);
    xQueuePeek(xAltMeasQueue, &mes, 10);

    if (timerID == 0){
        xTimerStart(xTimerLand, 10); // Starts timer
        vTimerSetTimerID( xTimerLand, (void *) 1 );
    }else{
        descent = descent - 2;
    }

    if (descent == 0 && mes <= 1) {
        xQueueOverwrite(xFSMQueue, &state);
        xTimerStop( xTimerLand, 0 );
    }else{
        xQueueOverwrite(xAltDesQueue, &descent);
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
    //vTaskResume(SwitchCheck);
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

    uint32_t state;

    while(1)
    {
        xQueuePeek(xFSMQueue, &state, 10);
        switch(state) {
            case FIND_REF:
                UARTSend("State 1: Finding Ref\n");
                findYawRef();
                break;

            case LANDED:
                UARTSend("State 2: Hover\n");
                //hover_loop();
                break;

            case FLYING:
                UARTSend("State 3: Flying\n");
                break;

            case LANDING:
                UARTSend("State 4: Landing\n");
                land();
                break;

            default:
                UARTSend("FSM Error\n");
        }

        vTaskDelay(FSM_PERIOD / portTICK_RATE_MS);

    }

}
