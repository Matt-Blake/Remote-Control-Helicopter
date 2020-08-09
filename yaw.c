/*
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
 */

#include "yaw.h"

/********************************************************
 * Constants
********************************************************/


/********************************************************
 * Globals
********************************************************/

static int32_t currentChannelReading;
static int16_t g_flagFoundZeroReference;

enum STATE_QUADRATURE {STATE_00 = 0, STATE_01 = 1, STATE_10 = 2, STATE_11 = 3};


/********************************************************
 * Interrupt for to check if the helicopter has found the
 * zero yaw reference.
********************************************************/
void
referenceInterrupt(void)
{
    int32_t yaw;

    xQueuePeek(xYawMeasQueue, &yaw, 10);// Read the current yaw value
    xQueueOverwrite(xYawRefQueue, &yaw);// Store the resulting yaw measurement in the RTOS queue
    g_flagFoundZeroReference = 1;
    GPIOIntClear(YAW_REFERENCE_BASE, YAW_REFERENCE_PIN);
}

/********************************************************
 * Flag for having found the zero reference
********************************************************/
int16_t
haveFoundZeroReferenceYaw(void)
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
    int32_t yaw;
    int32_t yaw_slot;

    xQueuePeek(xYawMeasQueue, &yaw, 10);// Read the current yaw value

    //Set yaw to -180 degrees if the current reading is 179 degrees
    if (yaw >= MAX_YAW_LIMIT) {
        yaw = MIN_YAW_LIMIT;
        yaw_slot = MIN_YAW_LIMIT * DEGREES_HALF_CIRCLE/MOUNT_SLOT_COUNT;
        xQueueOverwrite(xYawMeasQueue, &yaw);// Store the resulting yaw measurement in the RTOS queue
        xQueueOverwrite(xYawSlotQueue, &yaw_slot);
    }

    //Set yaw to 179 degrees if the current reading is -180 degrees
    else if (yaw <= MIN_YAW_LIMIT) {
        yaw = MAX_YAW_LIMIT;
        yaw_slot = MAX_YAW_LIMIT * DEGREES_HALF_CIRCLE/MOUNT_SLOT_COUNT;
        xQueueOverwrite(xYawMeasQueue, &yaw);// Store the resulting yaw measurement in the RTOS queue
        xQueueOverwrite(xYawSlotQueue, &yaw_slot);
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
    int32_t yaw;
    int32_t yaw_slot;
    int32_t newChannelReading = GPIOPinRead(GPIO_PORTB_BASE, YAW_PIN0_GPIO_PIN | YAW_PIN1_GPIO_PIN);
    GPIOIntClear(YAW_GPIO_BASE, YAW_PIN0_GPIO_PIN | YAW_PIN1_GPIO_PIN);

    // Bit shift the old reading and combine with new reading. Creates a 4-bit code unique to each state.
    uint8_t state_code = currentChannelReading << 2 | newChannelReading;

    xQueuePeek(xYawSlotQueue, &yaw_slot, 10);; // Get the current number of slots traveled

    switch (state_code){
        case (0b0010):
                UARTSend("CCW\n");
                yaw_slot--;
                break;
        case (0b0001):
                UARTSend("CW\n");
                yaw_slot++;
                break;
        case (0b0100):
                UARTSend("CCW\n");
                yaw_slot--;
                break;
        case (0b0111):
                UARTSend("CW\n");
                yaw_slot++;
                break;
        case (0b1101):
                UARTSend("CCW\n");
                yaw_slot--;
                break;
        case (0b1110):
                UARTSend("CW\n");
                yaw_slot++;
                break;
        case (0b1011):
                UARTSend("CCW\n");
                yaw_slot--;
                break;
        case (0b1000):
                UARTSend("CW\n");
                yaw_slot++;
                break;
        // Goes into default when a state is skipped. Usually happens when you turn too fast.
        default:
                UARTSend("QD Error\n");
    }

    currentChannelReading = newChannelReading;

    // Calculate yaw in degrees and store the results
    yaw = yaw_slot * MOUNT_SLOT_COUNT/DEGREES_HALF_CIRCLE; // Convert to degrees
    xQueueOverwrite(xYawSlotQueue, &yaw_slot); // Store the current number of slots traveled in the RTOS queue
    xQueueOverwrite(xYawMeasQueue, &yaw); // Store the resulting yaw measurement in the RTOS queue
    checkYawThresholds(); //Check if yaw has reached its threshold values

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
    GPIOPinTypeQEI(YAW_GPIO_BASE, YAW_PIN0_GPIO_PIN | YAW_PIN1_GPIO_PIN);     // Sets pin types to be Quad Decoding pins (Just makes Phase B HIGH = 2 instead of 1)

    GPIOIntRegister(YAW_GPIO_BASE, quadratureFSMInterrupt);                  // Sets QDIntHandler to be function to handle interrupt
    GPIOIntTypeSet(YAW_GPIO_BASE, YAW_PIN0_GPIO_PIN, GPIO_BOTH_EDGES);       // Sets Phase A interrupt on both rising and falling edges
    GPIOIntTypeSet(YAW_GPIO_BASE, YAW_PIN1_GPIO_PIN, GPIO_BOTH_EDGES);       // Sets Phase B interrupt on both rising and falling edges
    GPIOIntEnable(YAW_GPIO_BASE, YAW_PIN0_GPIO_PIN|YAW_PIN1_GPIO_PIN);       // Enables interrupts

}
