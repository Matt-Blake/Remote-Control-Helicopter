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


#define MOUNT_SLOT_COUNT    224//112                         // The number of slots in the helirig mount
#define DEGREES_HALF_CIRCLE 180                         // The number of degrees in a half circle
#define DEGREES_CIRCLE      360                         // The number of degrees in a circle
#define MAX_YAW_LIMIT       180                         // The maximum yaw (degrees)
#define MIN_YAW_LIMIT       -180                        // The minimum yaw (degrees)
#define YAW_GPIO_BASE       GPIO_PORTB_BASE             //Sets the base for pins J1-03 (PB0, channel A) and J1-04 (PB1, channel B)
#define QEI_PIN0            GPIO_INT_PIN_0
#define QEI_PIN1            GPIO_INT_PIN_1
#define YAW_REFERENCE_BASE  GPIO_PORTC_BASE
#define YAW_REFERENCE_PIN   GPIO_INT_PIN_4


QueueHandle_t xYawSlotQueue;
QueueHandle_t xYawMeasQueue;
QueueHandle_t xYawDesQueue;

/*
 * WRITE DESCRIPTION
 */
void vYawRefCallback( TimerHandle_t xTimer )
{
    int8_t  reset = 0;
    UARTSend("Yaw Ref Callback\n");
    GPIOIntEnable(YAW_REFERENCE_BASE, YAW_REFERENCE_PIN);
    vTimerSetTimerID( xTimer, (void *) reset );
}


/*
 * Function:    referenceInterrupt
 * --------------------------------
 * Handler for the interrupt which occurs when the helicopter
 * reaches the reference yaw position.
 * Resets the current yaw position and the number of quadrature
 * slots from reference to be 0.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
referenceInterrupt(void)
{
    int32_t reset = 0;

    UARTSend("REF_INTERRUPT\n\r");

    xQueueOverwriteFromISR(xYawMeasQueue, &reset, pdFALSE);         // Reset the current yaw to 0 (reference position)
    xQueueOverwriteFromISR(xYawSlotQueue, &reset, pdFALSE);         // Reset the curreny yaw_slow position to 0
    xEventGroupSetBitsFromISR(xFoundYawReference, YAW_REFERENCE_FLAG, pdFALSE); // Set reference flag
    GPIOIntClear(YAW_REFERENCE_BASE, YAW_REFERENCE_PIN);            // Clear the interrupt
}


/*
 * Function:    checkYawThresholds
 * --------------------------------
 * This function checks whether the yaw has reached the
 * positive or negative thresholds and resets it to the
 * opposite threshold if necessary.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
checkYawThresholds(void)
{
    int32_t yaw;
    //int32_t yaw_slot;

    xQueuePeek(xYawMeasQueue, &yaw, 10);// Read the current yaw value

    if (yaw > MAX_YAW_LIMIT) { //Set yaw to -180 degrees if the current reading is >180 degrees
        //yaw = MIN_YAW_LIMIT;
        yaw -= DEGREES_CIRCLE;
        //yaw_slot = MIN_YAW_LIMIT * DEGREES_HALF_CIRCLE/MOUNT_SLOT_COUNT;
        xQueueOverwrite(xYawMeasQueue, &yaw);// Store the resulting yaw measurement in the RTOS queue
        //xQueueOverwrite(xYawSlotQueue, &yaw_slot);
    }
    else if (yaw <= MIN_YAW_LIMIT) { //Set yaw to 180 degrees if the current reading is -180 degrees
        //yaw = MAX_YAW_LIMIT;
        yaw += DEGREES_CIRCLE;
        //yaw_slot = MAX_YAW_LIMIT * DEGREES_HALF_CIRCLE/MOUNT_SLOT_COUNT;
        xQueueOverwrite(xYawMeasQueue, &yaw);// Store the resulting yaw measurement in the RTOS queue
        //xQueueOverwrite(xYawSlotQueue, &yaw_slot);
    }
}


/*
 * Function:    quadratureFSMInterrupt
 * ------------------------------------
 * Pin change interrupt handler for the quadrature decoder
 * Contains a Finite State Machine which increments the
 * yaw (rotation) according to the values obtained by the
 * quadrature decoder.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
quadratureFSMInterrupt(void)
{
    int32_t yaw;
    int32_t yaw_slot;
    int32_t newChannelReading = GPIOPinRead(GPIO_PORTB_BASE, QEI_PIN0|QEI_PIN1);
    static int32_t currentChannelReading = 0;

    // Bit shift the old reading and combine with new reading. Creates a 4-bit code unique to each state.
    uint8_t state_code = currentChannelReading << 2 | newChannelReading;

    xQueuePeekFromISR(xYawSlotQueue, &yaw_slot); // Get the current number of slots traveled

    switch (state_code){
        case (0b0010):
                yaw_slot--;
                break;
        case (0b0001):
                yaw_slot++;
                break;
        case (0b0100):
                yaw_slot--;
                break;
        case (0b0111):
                yaw_slot++;
                break;
        case (0b1101):
                yaw_slot--;
                break;
        case (0b1110):
                yaw_slot++;
                break;
        case (0b1011):
                yaw_slot--;
                break;
        case (0b1000):
                yaw_slot++;
                break;
        default:                            // Goes into default when a state is skipped. Occurs when you turn too fast.
                UARTSend("QD Error\n");
    }

    currentChannelReading = newChannelReading;

    // Calculate yaw in degrees and store the results
    yaw = yaw_slot * MOUNT_SLOT_COUNT/DEGREES_HALF_CIRCLE;      // Convert the number of yaw slots to degrees
    xQueueOverwriteFromISR(xYawSlotQueue, &yaw_slot, pdFALSE);  // Store the current number of slots traveled in the RTOS queue
    xQueueOverwriteFromISR(xYawMeasQueue, &yaw, pdFALSE);       // Store the resulting yaw measurement in the RTOS queue
    GPIOIntClear(YAW_GPIO_BASE, QEI_PIN0|QEI_PIN1);     // Clears the interrupt on either of the pins
    checkYawThresholds();                                       //Check if yaw has reached its threshold values
}


/*
 * Function:    initReferenceYaw
 * ------------------------------
 * Initialises the pins and interrupt for the yaw reference.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
initReferenceYaw(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOInput(YAW_REFERENCE_BASE, YAW_REFERENCE_PIN);
    GPIOIntRegister(YAW_REFERENCE_BASE, referenceInterrupt);
    GPIOIntTypeSet(YAW_REFERENCE_BASE, YAW_REFERENCE_PIN, GPIO_FALLING_EDGE);
    GPIOIntEnable(YAW_REFERENCE_BASE, YAW_REFERENCE_PIN);
}


/*
 * Function:    initQuadrature
 * ----------------------------
 * Initialises the pins and interrupts for quadrature decoding.
 *
 * @params:
 *      - NULL
 * @return:
 *      - NULL
 * ---------------------
 */
void
initQuadrature(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeQEI(YAW_GPIO_BASE, QEI_PIN0|QEI_PIN1);    // Sets pin types to be Quad Decoding pins (Just makes Phase B HIGH = 2 instead of 1)
    GPIOIntRegister(YAW_GPIO_BASE, quadratureFSMInterrupt);                  // Sets QDIntHandler to be function to handle interrupt
    GPIOIntTypeSet(YAW_GPIO_BASE, QEI_PIN0|QEI_PIN1, GPIO_BOTH_EDGES);       // Sets Phase A interrupt on both rising and falling edges
    //GPIOIntTypeSet(YAW_GPIO_BASE, YAW_PIN0_GPIO_PIN, GPIO_BOTH_EDGES);       // Sets Phase A interrupt on both rising and falling edges
    //GPIOIntTypeSet(YAW_GPIO_BASE, YAW_PIN1_GPIO_PIN, GPIO_BOTH_EDGES);       // Sets Phase B interrupt on both rising and falling edges
    GPIOIntEnable(YAW_GPIO_BASE, QEI_PIN0|QEI_PIN1);       // Enables interruptss
}
