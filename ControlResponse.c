/*
 * ControlResponse.c
 *
 *  Created on: 29/07/2020
 *      Author:
 */
/*
int kp;
int kd;
int ki;

int prev_error;
int int_error;

int control;


void flightController(int target, int input, int output) {

    int time_step = SysCtlClockGet() / 1000;   //get time step //xTaskGetTickCount()/80000 //gives 1ms?

    double error = target - input; //calculate the error


    control = kp*error + kd*(prev_error - error)/time_step + int_error;

    prev_error = error;

    int_error+= (ki * error); //sum for int control

    output = control;
}
*/
