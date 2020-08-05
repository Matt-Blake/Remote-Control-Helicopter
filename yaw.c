/********************************************************
 *
 * yaw.c - Initializes quadrature decoder to deal with yaw.
 * Controls the increase and decrease of yaw using button
 * based interrupts. Finds the yaw zero reference. Converts
 * quadrature values into degrees.
 *
 * Tue am Group 1
 * Creators: Brendain Hennessy   57190084
 *           Sarah Kennelly      76389950
 *           Matt Blake          58979250
 * Last modified: 9/05/2019
 *
 * ENCE464 Assignment 1 Group 2
 * Creators: Grayson Mynott      56353855
 *           Ryan Earwaker       12832870
 *           Matt Blake          58979250
 * Last modified: 31/07/2020
********************************************************/

#include "yaw.h"

/********************************************************
 * Constants
********************************************************/
#define MOUNTSLOTCOUNT      112
#define DEGREES             180
//#define NEG_DEGREES_CIRCLE  -360

#define YAW_GPIO_BASE       GPIO_PORTB_BASE //Sets the base for pins J1-03 (PB0, channel A) and J1-04 (PB1, channel B)
#define YAW_PIN0_GPIO_PIN   GPIO_INT_PIN_0
#define YAW_PIN1_GPIO_PIN   GPIO_INT_PIN_1

#define YAW_REFERENCE_BASE  GPIO_PORTC_BASE
#define YAW_REFERENCE_PIN   GPIO_INT_PIN_4

/********************************************************
 * Globals
********************************************************/
static int32_t yaw;
static int32_t currentChannelReading;
static int32_t reference_yaw;
static int16_t g_flagFoundZeroReference;

enum STATE_QUADRATURE {STATE_00 = 0, STATE_01 = 1, STATE_10 = 2, STATE_11 = 3};

/********************************************************
 * Converts reference yaw to degrees.
********************************************************/
int32_t getReferenceYaw(void)
{
    int32_t reference_yaw_degrees;

    reference_yaw_degrees = reference_yaw * MOUNTSLOTCOUNT / DEGREES;

    return reference_yaw_degrees;
}

/********************************************************
 * Interrupt for to check if the helicopter has found the
 * zero yaw reference.
********************************************************/
void referenceInterrupt(void)
{
    reference_yaw = yaw;
    g_flagFoundZeroReference = 1;
    GPIOIntClear(YAW_REFERENCE_BASE, YAW_REFERENCE_PIN);
}

/********************************************************
 * Flag for having found the zero reference
********************************************************/
int16_t haveFoundZeroReferenceYaw(void)
{
    return g_flagFoundZeroReference;
}

/********************************************************
 * This function checks whether the yaw has reached the
 * positive or negative thresholds and resets it to the
 * opposite threshold if necessary.
********************************************************/
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

/********************************************************
 * Pin change interrupt handler for the quadrature decoder
 * Contains a Finite State Machine which increments the
 * yaw (rotation) according to the values obtained by the
 * quadrature decoder.
********************************************************/
void quadratureFSMInterrupt(void)
{
    int32_t newChannelReading = GPIOPinRead(GPIO_PORTB_BASE, YAW_PIN0_GPIO_PIN | YAW_PIN1_GPIO_PIN);
    GPIOIntClear(YAW_GPIO_BASE, YAW_PIN0_GPIO_PIN | YAW_PIN1_GPIO_PIN);

    // Bit shift the old reading and combine with new reading. Creates a 4-bit code unique to each state.
    uint8_t state_code = currentChannelReading << 2 | newChannelReading;

    switch (state_code){
        case (0b0010):
                UARTSend("CCW\n");
                yaw--;
                break;
        case (0b0001):
                UARTSend("CW\n");
                yaw++;
                break;
        case (0b0100):
                UARTSend("CCW\n");
                yaw--;
                break;
        case (0b0111):
                UARTSend("CW\n");
                yaw++;
                break;
        case (0b1101):
                UARTSend("CCW\n");
                yaw--;
                break;
        case (0b1110):
                UARTSend("CW\n");
                yaw++;
                break;
        case (0b1011):
                UARTSend("CCW\n");
                yaw--;
                break;
        case (0b1000):
                UARTSend("CW\n");
                yaw++;
                break;
        // Goes into default when a state is skipped. Usually happens when you turn too fast.
        default:
                UARTSend("QD Error\n");
    }

    currentChannelReading = newChannelReading;

    //Check if yaw has reached its threshold values
    //checkYawThresholds();
}

/********************************************************
 * Initializes the quadrature decoders used to calculate the yaw
********************************************************/
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

/********************************************************
 * Initialize the GPIO ports/pins used for quadrature
 * decoding.
********************************************************/
void initQuadrature(void)
{
    /*
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
    */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        GPIOPinTypeQEI(YAW_GPIO_BASE, YAW_PIN0_GPIO_PIN | YAW_PIN1_GPIO_PIN);      // Sets pin types to be Quad Decoding pins (Just makes Phase B HIGH = 2 instead of 1)

        GPIOIntRegister(YAW_GPIO_BASE, quadratureFSMInterrupt);                  // Sets QDIntHandler to be function to handle interrupt
        GPIOIntTypeSet(YAW_GPIO_BASE, YAW_PIN0_GPIO_PIN,                 // Sets Phase A interrupt on both rising and falling edges
                       GPIO_BOTH_EDGES);
        GPIOIntTypeSet(YAW_GPIO_BASE, YAW_PIN1_GPIO_PIN,                 // Sets Phase B interrupt on both rising and falling edges
                       GPIO_BOTH_EDGES);
        GPIOIntEnable(YAW_GPIO_BASE, YAW_PIN0_GPIO_PIN                   // Enables interrupts
                      | YAW_PIN1_GPIO_PIN);

}

/********************************************************
 * Converts yaw into degrees and returns yaw in degrees.
********************************************************/
int32_t getYawDegrees(void)
{
    return yaw * DEGREES / MOUNTSLOTCOUNT;
}
