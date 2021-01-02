# Remote Control Helicopter 

## Description
A C program which facilitates the stable flight of a remote control helicopter attached to a HeliRig. The user has control of the helicopters altitude and yaw through PID based motor control. This program was originally designed as part of ENCE464 Assignment 1, based of code used in the ENCE361 Assignment.

## Author
+ Matt Blake
+ Ryan Earwaker
+ Grayson Mynott 

## License
Added an [MIT License](LICENSE)

## Equipment Required
+ EK-TM4C123GXL – Tiva C Series TM4C123G LaunchPad with Orbit BoosterPack
+ Helicopter with PWM controllable motors


## Inputs
The target altitude is set using the Tiva board's up and down buttons. These buttons respectivly increase and decrease the altitude by 10% of the maximum altitude value. Double tapping the up button causes the helicopter to fly to the midpoint altitude (50% of maximum). This maxium altitude is set by changing the following ADC.h constant:
```c
#define VOLTAGE_DROP_ADC        1200 
```


switches

## Outputs
UART
OLED

## Known Issues
There are currently no known issues

### Contact
If you encounter any issues or questions with the preprocessing, please contact 
Matt Blake by email at matt.blake.mb@gmail.com
