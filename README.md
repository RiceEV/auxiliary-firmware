# Auxiliary Controller Code

Initial Author: Boyuan Deng

### Contributors
Ethan Peck (edp2@rice.edu, etdpeck@gmail.com)
Leo Marek (lnm7@rice.edu)
Oli MacGregor

## Overview

This library is built for Rice Electric Vehicle Electric Team's auxiliary board (aux board). The main functions of the auxiliary board include the following:

1. Take in and respond to information from pedal and break
2. Communicate with and respond to motor controller
3. Enable lighting control through switches installed on board
4. (Planned) Display the car's status on screen

All the functions mentioned above requires the microcontroller on aux board. We've chosen a Teensy 3.6 as the microcontroller, and make use of the Arduino libraries for C++. This board uses Controller Area Network (CAN) as the bus for communication between the aux board and the motor controller. The following sections will detail the libraries and files contained in the library.

## External Libraries

Besides main.cpp, there are two external libraries under the /lib directory. You can add additional directories to /lib. Refer to the README under /lib to find out how to do that.

### Arduino-PID-Library-master

The Proportioal-Integral-Derivative (PID) control is a control loop mechanism used widely in industry. If you are unfamiliar with this, be sure to do some online research to understand it. The basic idea is to read a sensor, then compute the desired actuator output by calculating proportional, integral, and derivative responses and summing those three components to compute the output.

This library is an external libarary provided by Arduino. It is currently unused as the PID functionality has been taken care of by the motor controller. However, it still comes handy if you ever want PID in aux board. The constructor of the class is as below:
```
PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki,
         double Kd, int POn, int ControllerDirection)
```
The PID input will be stored in first pointer, and the output second pointer, the setpoint third pointer. Three constants Kp, Ki and Kd are also required. You can specify the direction by last variable. A typical initialization of a PID object would look like below:
```
double pid_inA, pid_outA, pid_setA;
PID pidA(&pid_inA, &pid_outA, &pid_setA, 1, 1, 0, REVERSE);
```
Then in the main loop, you call
```
pidA.Compute();
```
And then `pid_outA` will contain desired output.

### OLED

This is a library that I personally write to drive the OLED screen. **Please install U8g2 in PlatformIO/your preferred IDE before it will work!** It is dependent on U8g2 libarary. A typical initialization would look like this:
```
OLED OLED_screen;
```
Internally a U8g2 screen object of the required type will be initialized. Then in your setup() routine, initializa the object by passing in a pointer to a location where the speed/power/whatever you want to show is stored:
```
OLED_screen.init(&speed);
```
After that, you can call it's functions like `display_braking`, `display_speed` etc. Some functions inside it is outdated, and you can add or replace some of them if you want.

## Main Code

Under /src directory are main control algorithms for the car.

### pins.h

Here we define necessary pins for main.cpp to use. In this way, we no longer need to memorize the pins for each functionality after we settled the design. You can find a pinout for Teensy 3.5 here:

https://www.pjrc.com/teensy/card8a_rev2.pdf

### main.cpp

This is the main control algorithm for aux board. I will explain some important variables I declared before I move to the algorithm itself.

##### Data

In SPI communication, we are transmitting 16 bits of information in forms of an integer. Manipulating this integer is a little trouble some, and `Data` is more easy to play with. `create_data()`, `decode_data()`, `encode_data()` and `print_data()` are self-explanatory helper functions to aid transition between struct and int.

##### Message ints

```
volatile unsigned int message_A_out;
volatile unsigned int message_A_in = 0;
volatile unsigned int message_B_out;
volatile unsigned int message_B_in = 0;
```
These are the variables we use to receive and send information over SPI. Rules for their encoding is explained in next section.

##### brk, direction and error

These are global variables keeping track of current state of the car.

##### The IntervalTimer objects

```
IntervalTimer clock_timer;
IntervalTimer SPI_timer;
```
These two IntervalTimer objects utilizes Teensy 3.5's hardware timed interrupts. They send out interrupt signals every certain amount of time, so that the designated function will execute every fixed amount of time. Meanwhile, main loop will pause.

#### The setup() routine

Here a few things need to be set up:

1. Set pinMode for necessary pins
2. Serial monitor, enabling debugging
3. Interrupt for break, as well we the IntervalTimer objects
4. SPI communication
5. OLED screen object

Refer to the code comments for implementation details.

#### The loop() routine

Currently useless. Important routines are wrapped inside IntervalTimer objects so they can be steadily and predicably executed every certain amount of time. However, potentially it can be used to monitor motor controller's state and react to it accordingly.

#### The blink() routine

This function send out signals to toggle headlight every half a second, so that it blinks.

#### The brake_isr() routine

Whenever there is a change in the `BRAKE_SNS` pin, this function will be executed. If it is activated when the current state is breaking, that means the brake is released. Otherwise, it means that the brake is been activated, then the current state is changed and we communicate with motor controller this change.

#### The get_and_send() routine

The core routine of main.cpp. First we read in values indicating position of the pedal. We then send this information along with current state over to motor controller. Note that you need to pull the pins low before you transmit the information, and pull the pins high after you finished.

## SPI specification

The information we send over in SPI is an 16-bit integer.

### 4 Most Significant Bits

4MSBs indicat the current state of the car. 1st bit indicate brake, 2nd bit direction, 3rd bit error, 4th bit currently left empty and for future use. For brake and error, 1 indicate "have" while 0 indicate "do not have" (e.g. 1 in brake indicate braking). For direction, 1 is forward while 0 is backward.

### 12 lower bits

12 lower bits are for set points. This allows for a resolution of 1/4096 for set points. The actual representation of the set points are by no means fixed, and feel free to change if you see fit.

### To conclude:

Brake|Direction|Error|Blank|12 bits of set point

#### Exersise:

What does 1100000000001111 mean?

Answer: Braking, forward, no errors and a setpoint of 31.

## Compile Instructions

The whole folder (aux_mcu-code) serves as a valid PlatformIO project. You should install the following PlatformIO plugins in your IDE:

1. platformio-ide
2. platformio-debugger
3. platformio-terminal

Currently, the tested IDE are VS Code and Atom. If in your IDE, the