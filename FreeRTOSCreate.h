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
#include "adc.h"
#include "altitude.h"
#include "pwm.h"
#include "fsm.h"

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

// Timer periods
#define DBL_BTN_TMR_PERIOD      1000
#define YAW_FLIP_TMR_PERIOD     1000

TaskHandle_t FSMTask;
TaskHandle_t OLEDDisp;
TaskHandle_t UARTDisp;
TaskHandle_t StatLED;
TaskHandle_t BtnCheck;
TaskHandle_t SwiCheck;
TaskHandle_t ADCTrig;
TaskHandle_t ADCMean;
TaskHandle_t MainPWM;
TaskHandle_t TailPWM;

QueueHandle_t xAltMeasQueue;
QueueHandle_t xAltDesQueue;
QueueHandle_t xYawMeasQueue;
QueueHandle_t xYawDesQueue;
QueueHandle_t xYawSlotQueue;
QueueHandle_t xFSMQueue;

SemaphoreHandle_t xUARTMutex;
SemaphoreHandle_t xUpBtnSemaphore;
SemaphoreHandle_t xYawFlipSemaphore;

EventGroupHandle_t xFoundAltReference;
EventGroupHandle_t xFoundYawReference;

TimerHandle_t xUpBtnTimer;
TimerHandle_t xDownBtnTimer;
TimerHandle_t xLandingTimer;


/*
 * Function:    createTasks
 * -------------------------
 * Creates all FreeRTOS tasks used in the system.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
createTasks(void);


/*
 * Function:    createQueues
 * -------------------------
 * Creates all FreeRTOS queues used in the system and
 * initialises the values to zero.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
createQueues(void);


/*
 * Function:    createSemaphores
 * -------------------------
 * Creates all FreeRTOS semaphores, mutexes and event groups
 * used in the system.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void createSemaphores(void);


/*
 * Function:    createSemaphores
 * -------------------------
 * Creates all FreeRTOS EventGroups used as flags in the system.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void createEventGroups(void);


/*
 * Function:    createTimers
 * --------------------------
 * Creates all FreeRTOS timers used in the system.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
createTimers(void);

#endif /* FREERTOSCREATE_H_ */
