//*****************************************************
//
// yaw.c - Initializes quadrature decoder to deal with yaw.
// Controls the increase and decrease of yaw using button
// based interrupts. Finds the yaw zero reference. Converts
// quadrature values into degrees.
//
// Tue am Group 1
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          58979250
// Last modified: 9/05/2019
//******************************************************

#include "yaw.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "clock.h"

//********************************************************
// Constants
//********************************************************
#define MOUNTSLOTCOUNT 112
#define DEGREES 180
#define NEG_DEGREES_CIRCLE -360

#define YAW_GPIO_BASE       GPIO_PORTB_BASE //Sets the base for pins J1-03 (PB0, channel A) and J1-04 (PB1, channel B)
#define YAW_PIN0_GPIO_PIN   GPIO_INT_PIN_0
#define YAW_PIN1_GPIO_PIN   GPIO_INT_PIN_1

#define YAW_REFERENCE_BASE GPIO_PORTC_BASE
#define YAW_REFERENCE_PIN GPIO_INT_PIN_4

//********************************************************
// Globals
//********************************************************
static int32_t yaw;
static int32_t currentChannelReading;
static int32_t reference_yaw;
static int16_t g_flagFoundZeroReference;

enum STATE_QUADRATURE {STATE_00 = 0, STATE_01 = 1, STATE_10 = 2, STATE_11 = 3};

//********************************************************
// Converts reference yaw to degrees
//********************************************************
int32_t getReferenceYaw(void)
{
    int32_t reference_yaw_degrees;

    reference_yaw_degrees = reference_yaw * MOUNTSLOTCOUNT / DEGREES;

    return reference_yaw_degrees;
}

//********************************************************
// Interrupt for to check if the helicopter has found the zero yaw reference
//********************************************************
void referenceInterrupt(void)
{
    reference_yaw = yaw;
    g_flagFoundZeroReference = 1;
    GPIOIntClear(YAW_REFERENCE_BASE, YAW_REFERENCE_PIN);
}

//********************************************************
// Flag for having found the zero reference
//********************************************************
int16_t haveFoundZeroReferenceYaw(void)
{
    return g_flagFoundZeroReference;
}

//*****************************************************************************
// This function checks whether the yaw has reached the positive or negative
// thresholds and resets it to the opposite threshold if necessary
//*****************************************************************************
void checkYawThresholds(void)
{
    //Set yaw to -180 degrees if the current reading is 179 degrees
    if (yaw * DEGREES / MOUNTSLOTCOUNT >= (DEGREES - 1)) {
        yaw = -1 * MOUNTSLOTCOUNT;
    }

    //Set yaw to 179 degrees if the current reading is -180 degrees
    else if (yaw * DEGREES / MOUNTSLOTCOUNT <= (-1 * DEGREES)) {
        yaw = (DEGREES - 1) * MOUNTSLOTCOUNT / DEGREES;
    }
}

//*****************************************************************************
// Pin change interrupt handler for the quadrature decoder
// Contains a Finite State Machine which increments the yaw (rotation) according to the
// values obtained by the quadrature decoder.
//*****************************************************************************
void quadratureFSMInterrupt(void)
{
    // for readings 0 means 00 where MSB is pin 1 and LSB is pin 0 , 1 means 01, 2 means 10, 3 means 11
    int32_t newChannelReading = GPIOPinRead(GPIO_PORTB_BASE, YAW_PIN0_GPIO_PIN | YAW_PIN1_GPIO_PIN);

    //inside each we would set old channel to current channel
    // current channel is 00
    if (currentChannelReading == STATE_00 && newChannelReading == STATE_01) {
        yaw--;
    }

    if (currentChannelReading == STATE_00 && newChannelReading == STATE_10) {
        yaw++;
    }

    // current channel is 01
    if (currentChannelReading == STATE_01 && newChannelReading == STATE_00) {
        yaw--;
    }

    if (currentChannelReading == STATE_01 && newChannelReading == STATE_11) {
        yaw++;
    }

    // current channel is 11
    if (currentChannelReading == STATE_11 && newChannelReading == STATE_01) {
        yaw--;
    }

    if (currentChannelReading == STATE_11 && newChannelReading == STATE_10) {
        yaw++;
    }

    // current channel is 10
    if (currentChannelReading == STATE_10 && newChannelReading == STATE_11) {
        yaw--;
    }

    if (currentChannelReading == STATE_10 && newChannelReading == STATE_00) {
        yaw++;
    }

    currentChannelReading = newChannelReading;

    GPIOIntClear(YAW_GPIO_BASE, YAW_PIN0_GPIO_PIN | YAW_PIN1_GPIO_PIN);

    //Check if yaw has reached its threshold values
    checkYawThresholds();
}

//********************************************************
// Initializes the quadrature decoders used to calculate the yaw
//********************************************************
void initReferenceYaw(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    // YAW_GPIO_BASE holds the value for Port B base
    GPIOIntRegister(YAW_REFERENCE_BASE, referenceInterrupt);

    GPIOPinTypeGPIOInput(YAW_REFERENCE_BASE, YAW_REFERENCE_PIN);

    GPIOIntTypeSet(YAW_REFERENCE_BASE, YAW_REFERENCE_PIN,
    GPIO_FALLING_EDGE);

    GPIOIntEnable(YAW_REFERENCE_BASE, YAW_REFERENCE_PIN);

    reference_yaw = 0;
}

//********************************************************
// Initialization functions for the clock (incl. SysTick), ADC, display, quadrature.
//********************************************************
void initQuadratureGPIO(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // YAW_GPIO_BASE holds the value for Port B base
    GPIOIntRegister(YAW_GPIO_BASE, quadratureFSMInterrupt);

    // YAW_PIN0_GPIO_PIN, YAW_PIN0_GPIO_PIN have the value for pin 0 and pin 1
    GPIOPinTypeGPIOInput(YAW_GPIO_BASE, YAW_PIN0_GPIO_PIN | YAW_PIN1_GPIO_PIN);

    GPIOIntTypeSet(YAW_GPIO_BASE, YAW_PIN0_GPIO_PIN | YAW_PIN1_GPIO_PIN,
    GPIO_BOTH_EDGES);

    GPIOIntEnable(YAW_GPIO_BASE, YAW_PIN0_GPIO_PIN | YAW_PIN1_GPIO_PIN);

    // set initial quadrature conditions
    currentChannelReading = GPIOPinRead(YAW_GPIO_BASE,
    YAW_PIN0_GPIO_PIN | YAW_PIN1_GPIO_PIN);
}

//********************************************************
// Converts yaw into degrees and returns yaw in degrees.
//********************************************************
int32_t getYawDegrees(void)
{
    return yaw * DEGREES / MOUNTSLOTCOUNT;
}
