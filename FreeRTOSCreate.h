/* ****************************************************************
 * FreeRTOSCreate.h
 *
 * Header file for the OLED module
 * Create FreeRTOS variables and tasks, as part of the FreeRTOS
 * scheduler.
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 19/08/2020
 *
 * ***************************************************************/

#ifndef FREERTOSCREATE_H_
#define FREERTOSCREATE_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "timers.h"
#include "statusLED.h"
#include "OLED.h"
#include "buttons.h"
#include "ADC.h"
#include "altitude.h"
#include "pwm.h"
#include "FSM.h"

// Task stack sizes in words, calculated experimentally based on uxTaskGetStackHighWaterMark()
#define LED_STACK_DEPTH         32
#define OLED_STACK_DEPTH        128
#define UART_STACK_DEPTH        128
#define BTN_STACK_DEPTH         64
#define SWITCH_STACK_DEPTH      64
#define ADC_STACK_DEPTH         32
#define MEAN_STACK_DEPTH        64
#define MAIN_PWM_STACK_DEPTH    128
#define TAIL_PWM_STACK_DEPTH    128
#define FSM_STACK_DEPTH         128

// Task priorities. Max priority is 8
#define LED_TASK_PRIORITY       4
#define OLED_TASK_PRIORITY      4
#define UART_TASK_PRIORITY      4
#define BTN_TASK_PRIORITY       5
#define SWI_TASK_PRIORITY       5
#define ADC_TASK_PRIORITY       8
#define MEAN_TASK_PRIORITY      7
#define MAIN_PWM_TASK_PRIORITY  6
#define TAIL_PWM_TASK_PRIORITY  6
#define FSM_TASK_PRIORITY       5

// Task periods (in ms)
#define LED_PERIOD              200         // The period used for the statusLED FreeRTOS task
#define DISPLAY_PERIOD          200         // Period to refresh the OLED display
#define UART_PERIOD             1000        // The period used to send information over UART
#define INPUT_PERIOD            25          // The period used for the button and switch polling FreeRTOS tasks
#define SAMPLING_PERIOD         10          // Period of ADC trigger task used to sample the altitude
#define ALTITUDE_PERIOD         200         // Period used to average and calculate the altitude
#define CONTROL_PERIOD          20          // Period used in the control loops
#define FSM_PERIOD              200         // Period used to control state changes in the helicopter's FSM

// Timer periods
#define DBL_BTN_TMR_PERIOD      1000
#define YAW_FLIP_TMR_PERIOD     1000

// FreeRTOS constants
#define SEMAPHORE_SIZE          2           // The size used for counting semaphores
#define TICKS_TO_WAIT           10          // The number of ticks to wait to get a value from a FreeRTOS variable

extern TaskHandle_t FSMTask;
extern TaskHandle_t OLEDDisp;
extern TaskHandle_t UARTDisp;
extern TaskHandle_t StatLED;
extern TaskHandle_t BtnCheck;
extern TaskHandle_t SwiCheck;
extern TaskHandle_t ADCTrig;
extern TaskHandle_t ADCMean;
extern TaskHandle_t MainPWM;
extern TaskHandle_t TailPWM;

extern QueueHandle_t xAltMeasQueue;
extern QueueHandle_t xAltDesQueue;
extern QueueHandle_t xYawMeasQueue;
extern QueueHandle_t xYawDesQueue;
extern QueueHandle_t xYawSlotQueue;
extern QueueHandle_t xFSMQueue;

extern SemaphoreHandle_t xUARTMutex;
extern SemaphoreHandle_t xUpBtnSemaphore;
extern SemaphoreHandle_t xYawFlipSemaphore;

extern EventGroupHandle_t xFoundAltReference;
extern EventGroupHandle_t xFoundYawReference;

extern TimerHandle_t xUpBtnTimer;
extern TimerHandle_t xDownBtnTimer;
extern TimerHandle_t xLandingTimer;


/*
 * Function:    initFreeRTOS
 * --------------------------
 * Call to all initialize functions for FreeRTOS components
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void initFreeRTOS(void);

#endif /* FREERTOSCREATE_H_ */
