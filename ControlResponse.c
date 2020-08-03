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

#define OUT_MAX; //need to find out max and min values
#define OUT_MIN;

void flightController(int target, int input, int output) {

    int time_step = SysCtlClockGet() / 1000;   //get time step //xTaskGetTickCount()/80000 //gives 1ms?

    double error = target - input; //calculate the error, critical section so disable ints etc

    control = kp*error + kd*(prev_error - error)/time_step + int_error;

    prev_error = error;

    int_error+= (ki * error); //sum for int control

    if(control > OUT_MAX) {control = OUT_MAX;}

    if(control < OUT_MIN) {control = OUT_MIN;}

    output = control;

    vTaskDelay(time_step); for 1ms
}
*/
