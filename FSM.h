/* ****************************************************************
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
 * ***************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "uart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"
#include "timers.h"

#include "pwm.h"

extern TaskHandle_t MainPWM;
extern TaskHandle_t TailPWM;
extern TaskHandle_t BtnCheck;
extern TaskHandle_t SwitchCheck;

extern QueueHandle_t xFSMQueue;
extern QueueHandle_t xAltMesQueue;
extern QueueHandle_t xAltDesQueue;

extern TimerHandle_t xTimerLand;

//****************************************************************************
//Check if found the reference yaw, if it has then set found reference to 1 and
//reset the integrator error and update yaw reference
//****************************************************************************
void findYawRef(void);

//****************************************************************************
//If it has reached the appropriate height and yaw it will move to state 2
//Otherwise if it still needs to rotate to reference yaw set altitude to 25
//Finally if needed to reach height then sets altitude_PWM
//Altitude PWM is returned
//****************************************************************************
//int goToStartPoisition(yaw_degrees, adc_error_signal, heightVal, rotatedToReferenceYaw);

//****************************************************************************
//Gets the helicopter to find the reference yaw and get to altitude of 20%
//****************************************************************************
void take_off(void);

//****************************************************************************
// Helicopter tracks reference altitude and yaw
//****************************************************************************
void hover(void);

//****************************************************************************
//Lands the helicoptor by rotating to the yaw reference then decrementing the
//height and a defined frequency
//****************************************************************************
void land(void);

//****************************************************************************
//Checks the switch and updates the display and UART after the helicoptor is
//landed
//****************************************************************************
void landed(void);

//****************************************************************************
//Calls the appropriate function for the current state
// the slider will switch between take_off and landing. The transition will be dependent on current state
//****************************************************************************
void FSM(void *pvParameters);

