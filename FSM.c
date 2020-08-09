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



//****************************************************************************
//Check if found the reference yaw, if it has then set found reference to 1 and
//reset the integrator error and update yaw reference
//****************************************************************************
void
findYawRef(void)
{
    int32_t PWM_main = 30; // place holder for now
    int32_t hover = 10; // hover alt

    //
    vTaskSuspend(MainPWM); // suspend the control system until ref is found
    vTaskSuspend(TailPWM);
    //vTaskSuspend(BtnCheck);

    if(xEventGroupGetBits(xFoundYawReference)) {
        setRotorPWM(0, 1);
        vTaskResume(MainPWM); // re enable the control system
        vTaskResume(TailPWM);
        //vTaskResume(BtnCheck);

    } else { // finding ref mode
        setRotorPWM(PWM_main, 1); // set the main rotor to on, the torque from the main rotor should work better than using the tail, have to test and actually see whats best
    }
}

int findZeroReferenceYaw(void)
{
    int8_t foundReferenceYaw = 0;
    //Checks if the yaw zero reference has been found
    if (g_flagFoundZeroReference == 0) {
        g_flagFoundZeroReference = haveFoundZeroReferenceYaw();
    }
    else
    {
        foundReferenceYaw = 1;
        g_yawReference = getReferenceYaw();
        g_yawZeroReference = g_yawReference;
        g_yaw_controller.intergratedError = 0;
    }
    return foundReferenceYaw;
}

//****************************************************************************
//If it has reached the appropriate height and yaw it will move to state 2
//Otherwise if it still needs to rotate to reference yaw set altitude to 25
//Finally if needed to reach height then sets altitude_PWM
//Altitude PWM is returned
//****************************************************************************
int ToStartPoisition(yaw_degrees, adc_error_signal, heightVal, rotatedToReferenceYaw)
{
    uint32_t altitude_PWM;
    //Checks if the heli has reached the altitude reference for flying and is stable
    //at the set yaw reference then changes to flying state
    if (heightVal >= g_altitudeReference && ((yaw_degrees < (g_yawZeroReference - 1)) || (yaw_degrees > (g_yawZeroReference + 1)))) {
        changeState(2);
    }
    //If the heli is not at yaw reference then keep the main rotor PWM at 25, this is so there is less friction on the heli.
    else if (rotatedToReferenceYaw == 0)
    {
        altitude_PWM = 25;
    }
    //If heli is at the yaw reference but not at altitude reference, use a control signal to set PWM for main rotor
    else
    {
        altitude_PWM = getControlSignal(&g_altitude_controller, adc_error_signal, false);
    }
    return altitude_PWM;
}

//****************************************************************************
//Gets the helicopter to find the reference yaw and get to altitude of 20%
//****************************************************************************
void take_off(void)
{
    //Setting up variables
    uint32_t  g_count;
    int32_t  yaw_degrees;
    int32_t  heightVal;
    int32_t  sample_mean_adc;
    int32_t  yaw_error_signal;
    int32_t  adc_error_signal;
    uint32_t altitude_PWM;
    uint32_t yaw_PWM;
    uint32_t prev_uart_count;
    uint32_t prev_dis_count;
    uint32_t prev_pid_count;

    int8_t foundReferenceYaw;
    uint8_t rotatedToReferenceYaw;

    g_altitudeReference = 0; //Set the reference height to 0%
    rotatedToReferenceYaw = 0; //Set the variable to 0 showing it hasn't rotated to reference Yaw
    foundReferenceYaw = 0; //Set the found reference yaw varible to 0

    //Take off while loop
    while(g_heliState == TAKEOFF) {
        //Get current height, yaw, g_count, yaw error singal and adc error signal
        sample_mean_adc = getSampleMeanADC();
        heightVal = getAltitudePercent(sample_mean_adc);
        yaw_degrees = getYawDegrees();
        g_count = getGCount();

        //Updating UART and display at defined frequencies
        if ((g_count - prev_uart_count) >= (SYSTICK_RATE_HZ / UART_RATE_HZ)){
            UARTDisplay(yaw_degrees, g_yawReference, heightVal, g_altitudeReference, altitude_PWM, yaw_PWM, g_heliState);
            prev_uart_count = g_count;
        }

        //Updating the OLED Display
        if ((g_count - prev_dis_count) >= (SYSTICK_RATE_HZ / DISPLAY_RATE_HZ))
        {
            display_screen(heightVal, yaw_degrees, altitude_PWM, yaw_PWM);
            prev_dis_count = g_count;
        }

        // Updating the PID controller
        if ((g_count - prev_pid_count) >= (SYSTICK_RATE_HZ / PI_RATE_HZ)){

            //Get error signals
            yaw_error_signal = getErrorSignal(g_yawZeroReference, yaw_degrees);
            adc_error_signal = g_altitudeReference - heightVal;

            //Move around until the zero reference is found
            if (foundReferenceYaw == 0) {
                yaw_PWM = YAW_START_HZ;
                altitude_PWM = 25;
                foundReferenceYaw = findZeroReferenceYaw();

            } else {

                //Move to the reference yaw and then up to 20% height
                yaw_PWM = getControlSignal(&g_yaw_controller, yaw_error_signal, true);
                if ((yaw_degrees < (g_yawZeroReference - 2)) || (yaw_degrees > (g_yawZeroReference + 2))) {
                    rotatedToReferenceYaw = 1;
                    g_altitudeReference = 20;
                }
                altitude_PWM = goToStartPoisition(yaw_degrees, adc_error_signal, heightVal, rotatedToReferenceYaw);
            }
            //Set the PWM for each rotor
            setTailPWM(yaw_PWM);
            setMainRotorPWM(altitude_PWM);

            prev_pid_count = g_count;
        }
    }
}

//****************************************************************************
//Polls the buttons to change the altitude and yaw references
//Keeps the helicoptor at the desired references
//Checks the switch if it needs to change states
//****************************************************************************
void hover_loop(void)
{
    //initializes variables
    uint32_t g_count;
    uint32_t prev_dis_count;
    uint32_t prev_uart_count;
    int32_t yaw_degrees;
    int32_t  heightVal;
    int32_t  sample_mean_adc;
    int32_t yaw_error_signal;
    int32_t adc_error_signal;
    uint32_t altitude_PWM;
    uint32_t yaw_PWM;
    uint32_t prev_pid_count;

    g_altitudeReference = 20;

    while (g_heliState == HOVER) {
        checkSwitch(); //Checks swtich and then changes the state if necessary

        //Gets the current height and yaw position
        yaw_degrees = getYawDegrees();
        sample_mean_adc = getSampleMeanADC();
        heightVal = getAltitudePercent(sample_mean_adc);

        updateCheckButtons(yaw_degrees, heightVal); //Checks the buttons and updates the references appropriately

        g_count = getGCount();

        //Updates the OLED Display
        if ((g_count - prev_dis_count) >= (SYSTICK_RATE_HZ / DISPLAY_RATE_HZ))
        {
            display_screen(heightVal, yaw_degrees, altitude_PWM, yaw_PWM);
            prev_dis_count = g_count;
        }

        //Updates the UART
        if ((g_count - prev_uart_count) >= (SYSTICK_RATE_HZ / UART_RATE_HZ)){
            UARTDisplay(yaw_degrees, g_yawReference, heightVal, g_altitudeReference, altitude_PWM, yaw_PWM, g_heliState);
            prev_uart_count = g_count;
        }

        //Updates the PID Controller
        if ((g_count - prev_pid_count) >= (SYSTICK_RATE_HZ / PI_RATE_HZ)){

            adc_error_signal = g_altitudeReference - heightVal;
            yaw_error_signal = getErrorSignal(g_yawReference, yaw_degrees);

            altitude_PWM = getControlSignal(&g_altitude_controller, adc_error_signal, false);
            yaw_PWM = getControlSignal(&g_yaw_controller, yaw_error_signal, true);

            setMainRotorPWM(altitude_PWM);
            setTailPWM(yaw_PWM);

            prev_pid_count = g_count;
        }
    }
}

//****************************************************************************
//Lands the helicoptor by rotating to the yaw reference then decrementing the
//height and a defined frequency
//****************************************************************************
void land(void)
{
    //Initializes the variables
    uint32_t prev_uart_count;
    int32_t yaw_degrees;
    int32_t  heightVal;
    int32_t  sample_mean_adc;
    uint32_t yaw_error_signal;
    int32_t adc_error_signal;
    uint32_t altitude_PWM;
    uint32_t yaw_PWM;
    uint32_t prev_dis_count;
    uint32_t prev_pid_count;
    uint32_t g_count;
    uint32_t rotatedToReferenceYaw;

    //Sets up the landed references for yaw and altitude
    g_altitudeReference = 20;
    g_yawReference = g_yawZeroReference;
    rotatedToReferenceYaw = 0;

    while (g_heliState == LANDING)
    {
        //Get the current g_count, yaw degrees and height
        yaw_degrees = getYawDegrees();
        sample_mean_adc = getSampleMeanADC();
        heightVal = getAltitudePercent(sample_mean_adc);
        g_count = getGCount();

        //If height is zero and within a 2 degree range of yaw reference then change state
        if ((heightVal == 0) && ((yaw_degrees > (g_yawZeroReference - 2)) && (yaw_degrees < (g_yawZeroReference + 2)))) {
            g_altitudeReference = 0;
            changeState(0);
        }
        else
        {
            //Updates the control, alititude reference, uart and display at appropriate rates
            if ((g_count - prev_uart_count) >= (SYSTICK_RATE_HZ / UART_RATE_HZ)){
                UARTDisplay(yaw_degrees, g_yawReference, heightVal, g_altitudeReference, altitude_PWM, yaw_PWM, g_heliState);
                prev_uart_count = g_count;
            }

            //Updates OLED display
            if ((g_count - prev_dis_count) >= (SYSTICK_RATE_HZ / DISPLAY_RATE_HZ))
            {
                display_screen(heightVal, yaw_degrees, altitude_PWM, yaw_PWM);
                prev_dis_count = g_count;
            }

            //Used for stable landing by decrementing the reference altitude at a set rate
            if ((g_count - prev_pid_count) >= (SYSTICK_RATE_HZ / ALTITUDE_DECREMENT_RATE_HZ)){
                //If gone to the reference yaw and the altitude reference is abocve zero decrement it
                if (rotatedToReferenceYaw == 1 || g_altitudeReference > 0){
                    g_altitudeReference -= 1;
                }
            }

            //Updates PID controller
            if ((g_count - prev_pid_count) >= (SYSTICK_RATE_HZ / PI_RATE_HZ)){
                //If within certain range of yaw reference change rotated to reference yaw to 1
                if (rotatedToReferenceYaw == 0 || (yaw_degrees < (g_yawZeroReference - 1)) || (yaw_degrees > (g_yawZeroReference + 1))) {
                    rotatedToReferenceYaw = 1;
                }
                //Get error signals and set the rotor pwms
                adc_error_signal = g_altitudeReference - heightVal;
                altitude_PWM = getControlSignal(&g_altitude_controller, adc_error_signal, false);
                yaw_error_signal = getErrorSignal(g_yawZeroReference, yaw_degrees);
                yaw_PWM = getControlSignal(&g_yaw_controller, yaw_error_signal, true);
                setMainRotorPWM(altitude_PWM);
                setTailPWM(yaw_PWM);

                prev_pid_count = g_count;
            }
        }

    }
}

//****************************************************************************
//Checks the switch and updates the display and UART after the helicoptor is
//landed
//****************************************************************************
void landed(void)
{
    //Initialize appropriate variables
    uint32_t g_count;
    uint32_t prev_uart_count;
    int32_t yaw_degrees;
    int32_t  heightVal;
    int32_t  sample_mean_adc;
    int32_t altitude_PWM;
    int32_t yaw_PWM;
    uint32_t prev_dis_count;

    //Set the PWM back to zero
    altitude_PWM = 0;
    yaw_PWM = 0;
    setMainRotorPWM(altitude_PWM);
    setTailPWM(yaw_PWM);

    //Landed while loop
    while (g_heliState == LANDED) {
        checkSwitch(); //Checks switch and changes state if necessary
        yaw_degrees = getYawDegrees();
        sample_mean_adc = getSampleMeanADC();
        heightVal = getAltitudePercent(sample_mean_adc);

        //Updates the display and UART at the appropriate frequencies
        g_count = getGCount();

        //Update UART
        if ((g_count - prev_uart_count) >= (SYSTICK_RATE_HZ / UART_RATE_HZ)){
            UARTDisplay(yaw_degrees, g_yawReference, heightVal, g_altitudeReference, altitude_PWM, yaw_PWM, g_heliState);
            prev_uart_count = g_count;
        }

        //Update OLED Display
        if ((g_count - prev_dis_count) >= (SYSTICK_RATE_HZ / DISPLAY_RATE_HZ))
        {
            display_screen(heightVal, yaw_degrees, altitude_PWM, yaw_PWM);
            prev_dis_count = g_count;
        }
    }
}

//****************************************************************************
//Calls the appropriate function for the current state
//****************************************************************************
static void
FSM(void *pvParameters) {

    uint8_t = state;

    while(1)
    {
        xQueuePeek(xFSMQueue, &state, 10);
        switch(state) {
            case TAKEOFF:
                UARTSend("State 1: Takeoff\n");
                take_off();
                break;

            case HOVER:
                UARTSend("State 2: Hover\n");
                hover();
                break;

            case LANDING:
                UARTSend("State 3: Landing\n");
                land()
                break;

            case LANDED:
                UARTSend("State 4: Landed\n");
                landed();
                break;

            default:
                UARTSend("QD Error\n");
        }

        vTaskDelay(FSM_PERIOD / portTICK_RATE_MS);

    }

}
