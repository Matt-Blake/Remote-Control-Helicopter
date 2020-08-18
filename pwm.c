/*
 * pwm.c - Example code which generates a single PWM
 *     output on J4-05 (M0PWM7) with duty cycle fixed and
 *     the frequency controlled by UP and DOWN buttons in
 *     the range 50 Hz to 400 Hz.
 * 2017: Modified for Tiva and using straightforward, polled
 *     button debouncing implemented in 'buttons4' module.
 *
 * P.J. Bones   UCECE
 * Last modified:  07/02/2018
 *
 * pwmMainGen.c - This code was based off the pwmGen.c example
 *      code. We have changed the code only generate a single PWM signal on
 *      Tiva board pin J4-05 = PC5 (M0PWM7). This is the same PWM output as
 *      the helicopter main rotor.
 *
 * ENCE Tue am Group 1
 * Creators: Brendain Hennessy   57190084
 *           Sarah Kennelly      76389950
 *           Matt Blake          58979250
 * Last modified: 09/05/2019
 *
 * pwm.c - This code was based off the pwmMainGen.c code from ENCE361.
 *      This code has been changed to incorporate PWM of the tail rotor.
 *      This is part of ENCE464 Assignment 1.
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 31/07/2020
 */

#include "pwm.h"

// PWM configuration
//  PWM Hardware Details M0PWM7 (gen 3)
#define PWM_START_RATE_HZ       200
#define PWM_FIXED_DUTY          0
#define PWM_DIVIDER_CODE        SYSCTL_PWMDIV_16
#define PWM_DIVIDER             16

//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_PERIPH_PWM     SYSCTL_PERIPH_PWM0
#define PWM_MAIN_BASE           PWM0_BASE
#define PWM_MAIN_GEN            PWM_GEN_3
#define PWM_MAIN_OUTNUM         PWM_OUT_7
#define PWM_MAIN_OUTBIT         PWM_OUT_7_BIT

#define PWM_MAIN_PERIPH_GPIO    SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE      GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG    GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN       GPIO_PIN_5

//  ---Tail Rotor PWM: PF1, J4-05
#define PWM_TAIL_PERIPH_PWM     SYSCTL_PERIPH_PWM1
#define PWM_TAIL_BASE           PWM1_BASE
#define PWM_TAIL_GEN            PWM_GEN_2
#define PWM_TAIL_OUTNUM         PWM_OUT_5
#define PWM_TAIL_OUTBIT         PWM_OUT_5_BIT

#define PWM_TAIL_PERIPH_GPIO    SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE      GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG    GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN       GPIO_PIN_1



QueueHandle_t xMainPWMQueue;
QueueHandle_t xTailPWMQueue;

TaskHandle_t MainPWM;
TaskHandle_t TailPWM;

controller_t g_alt_controller;
controller_t g_yaw_controller;

/*
 * Function:    initPWM
 * ---------------------
 * Initializes the PWM output used to drive the helicopter motors.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
initPWM (void)
{

    SysCtlPWMClockSet(PWM_DIVIDER_CODE);

    // Enable PWM peripherals
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    while(!SysCtlPeripheralReady(PWM_MAIN_PERIPH_PWM));

    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);
    while(!SysCtlPeripheralReady(PWM_TAIL_PERIPH_PWM));

    // Enable GPIO peripherals
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);
    while(!SysCtlPeripheralReady(PWM_MAIN_PERIPH_GPIO));

    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);
    while(!SysCtlPeripheralReady(PWM_TAIL_PERIPH_GPIO));

    // Set PWM pin types
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);

    // Set PWM GPIO
    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);

    // Configure PWM generators
    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set initial duty cycles
    setRotorPWM(PWM_FIXED_DUTY, 1);         // Set the initial main PWM parameters
    setRotorPWM(PWM_FIXED_DUTY, 0);         // Set the initial tail PWM parameters

    // Enable PWM generators
    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);
    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    // Disable the output.  Repeat this call with 'true' to turn output on.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);

}

/*
 * Function:    initControllers
 * -----------------------------
 * Initializes the structs used to hold the PWM PID controller
 * gains and errors.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
initControllers(void)
{
    // Create empty controllers
    //controller_t alt_controller;
    //controller_t yaw_controller;
    //controller_t* alt_controller_address;
    //controller_t* yaw_controller_address;

    // Assign addresses to controllers
    //alt_controller_address = &alt_controller;
    //yaw_controller_address = &yaw_controller;

    // Inialise controllers with predefined gains
    //initController(alt_controller_address, false);
    //initController(yaw_controller_address, true);

    initController(&g_alt_controller, false);
    initController(&g_yaw_controller, true);

    // Store pointers to controllers in queues so they can be accessed by other functions
    //xQueueOverwrite(xAltControllerQueue, &alt_controller);
    //xQueueOverwrite(xYawControllerQueue, &yaw_controller);
}

/*
 * Function:    setRotorPWM
 * -------------------------
 * Sets the PWM duty cycle and period of each motor.
 *
 * @params:
 *      - uint32_t ui32Duty: The duty cycle to be set for the PWM.
 *      - bool SET_MAIN: If True, sets the main rotor PWM params.
 *      If False, sets the tail rotor PWM params
 * @return:
 *      - NULL
 * ---------------------
 */
void
setRotorPWM (uint32_t ui32Duty, bool SET_MAIN)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period = SysCtlClockGet() / PWM_DIVIDER / PWM_START_RATE_HZ;

    if (SET_MAIN == true) // Set PWM freq/duty of the main rotor
    {
        PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
        PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM, ui32Period * ui32Duty / 100);
    }
    else // Set PWM freq/duty of the tail rotor
    {
        PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
        PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM, ui32Period * ui32Duty / 100);
    }
}


/*
 * Function:    turnOnMainPWM
 * ---------------------------
 * Turns on the main motor PWM output.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
turnOnMainPWM(void)
{
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
}


/*
 * Function:    turnOnTailPWM
 * ---------------------------
 * Turns on the tail motor PWM output.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
turnOnTailPWM(void)
{
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
}


/*
 * Function:    SetMainDuty
 * -------------------------
 * FreeRTOS task that periodically calls functions to set the PWM
 * duty cycle of the main rotor required to reach the desired
 * altitude.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
SetMainDuty(void *pvParameters)
{
    int32_t alt_PWM = 0;
    int32_t alt_meas = 0;
    int32_t alt_desired = 0;
    //controller_t alt_controller;

    while (1)
    {
        // Retrieve altitude information
        xQueuePeek(xAltMeasQueue, &alt_meas,    10); // Retrieve measured altitude data from the RTOS queue
        xQueuePeek(xAltDesQueue,  &alt_desired, 10); // Retrieve desired altitude data from the RTOS queue

        // Set PWM duty cycle of main rotor in order to hover to the desired altitude
        //xQueuePeek(xAltControllerQueue, &alt_controller, 10); // Get altitude controller information
        alt_PWM = getControlSignal(&g_alt_controller, alt_desired, alt_meas, false); // Use the error to calculate a PWM duty cycle for the main rotor
        if(alt_PWM > MAX_DUTY){
            alt_PWM = MAX_DUTY;
        }else if(alt_PWM < MIN_DUTY){
            alt_PWM = MIN_DUTY;
        }
        setRotorPWM(alt_PWM, 1); // Set main rotor to calculated PWM
        xQueueOverwrite(xMainPWMQueue, &alt_PWM); // Store the main PWM duty cycle in a queue

        vTaskDelay(CONTROL_PERIOD / portTICK_RATE_MS); // Block task so lower priority tasks can run
    }
}


/*
 * Function:    SetTailDuty
 * -------------------------
 * FreeRTOS task that periodically calls functions to set the PWM
 * duty cycle of the tail rotor required to reach the desired yaw.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
SetTailDuty(void *pvParameters)
{
    int32_t yaw_PWM = 0;
    int32_t yaw_meas = 0;
    int32_t yaw_desired = 0;
    int32_t alt_PWM = 0;
    //controller_t yaw_controller;

//    char cMessage[20];

    while (1)
    {
        // Retrieve yaw information
        xQueuePeek(xYawMeasQueue, &yaw_meas,   10); // Retrieve measured yaw data from the RTOS queue
        xQueuePeek(xYawDesQueue, &yaw_desired, 10); // Retrieve desired yaw data from the RTOS queue
        xQueuePeek(xMainPWMQueue, &alt_PWM, 10); // Retrieve the main rotor's duty cycle

        // Set PWM duty cycle of tail rotor in order to spin to target yaw
        //xQueuePeek(xYawControllerQueue, &yaw_controller, 10); // Get yaw controller information
        yaw_PWM = getControlSignal(&g_yaw_controller, yaw_desired, yaw_meas, true); // Use the error to calculate a PWM duty cycle for the tail rotor
        yaw_PWM = yaw_PWM + (alt_PWM * MAIN_ROTOR_FACTOR); // Compensate tail PWM due to effect of main rotor duty cycle

        //yaw_PWM = alt_PWM * MAIN_ROTOR_FACTOR;

        if (yaw_PWM > MAX_DUTY){
            yaw_PWM = MAX_DUTY;
        }else if(yaw_PWM < MIN_DUTY){
            yaw_PWM = MIN_DUTY;
        }
        setRotorPWM(yaw_PWM, 0); // Set tail rotor to calculated PWM
        xQueueOverwrite(xTailPWMQueue, &yaw_PWM); // Store the tail PWM duty cycle in a queue

        vTaskDelay(CONTROL_PERIOD / portTICK_RATE_MS); // Block task so lower priority tasks can run
    }
}
