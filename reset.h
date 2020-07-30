//*****************************************************
//
// reset.h - Initialises the helicopter to have a hard reset interrupt
//
// Tue am Group 1
// Creators: Brendain Hennessy   57190084
//           Sarah Kennelly      76389950
//           Matt Blake          58979250
// Last modified: 9/05/2019
//******************************************************

#ifndef RESET_H_
#define RESET_H_

#include <stdint.h>
#include <stdbool.h>

//******************************************************
// Initilize reset GPIO, sets up reset interrupt.
//******************************************************
void initResetGPIO(void);

#endif /* RESET_H_ */
