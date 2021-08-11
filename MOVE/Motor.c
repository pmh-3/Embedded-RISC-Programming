// Motor.c
// Runs on MSP432
// Provide mid-level functions that initialize ports and
// set motor speeds to move the robot. Lab 13 solution
// Daniel Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

#include <stdint.h>
#include "msp.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"

// *******Lab 13 solution*******

// ------------Motor_Init------------
// Initialize GPIO pins for output, which will be
// used to control the direction of the motors and
// to enable or disable the drivers.
// The motors are initially stopped, the drivers
// are initially powered down, and the PWM speed
// control is uninitialized.
// Input: none
// Output: none
void Motor_Init(void){
    P3->SEL0 &= ~(0xC0);      // set to 0
    P3->SEL1 &= ~(0xC0);
    P3->DIR |= 0xC0;          //set high
    P3->OUT &= ~0xC0;        // 3.6, 3.7 low, motor driver inactive, asleep  11000000b = 0xC0
    P2->SEL0 |= (0xC0);
    P2->SEL1 &= ~(0xC0);     //set low
    P2->DIR  |= 0xC0;       //set high
    P5->SEL0 &= ~(0x30);
    P5->SEL1 &= ~(0x30);
    P5->DIR  |= 0x30;
    P5->OUT &= ~(0x30);     // 5.4, 5.5 low, motor direction forward
    return;
}

// ------------Motor_Stop------------
// Stop the motors, power down the drivers, and
// set the PWM speed control to 0% duty cycle.
// Input: none
// Output: none
void Motor_Stop(void){
  P3->OUT &= ~0xC0;      //motor drivers asleep
  TIMER_A0->CTL &= ~0x0030;  // MC=00 stop the timer
  TIMER_A0->CCTL[3] &= ~0X00E0; // OUTMOD (bits 7-5) set to 000
  TIMER_A0->CCTL[3] &= ~0X0002; // OUT (bit 2) set to 0
  TIMER_A0->CCTL[4] &= ~0X00E0; // OUTMOD (bits 7-5) set to 000
  TIMER_A0->CCTL[4] &= ~0X0002; // OUT (bit 2) set to 0
}

// ------------Motor_Forward------------
// Drive the robot forward by running left and
// right wheels forward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called

void Motor_Forward(uint16_t leftDuty, uint16_t rightDuty){ 
  // write this similar to Motor_Right below
       TIMER_A0->CCR[3] = rightDuty;
       TIMER_A0->CCR[4] = leftDuty;
       P5->OUT &= ~0x10;       // left motor forward
       P5->OUT &= ~0x20;        // right motor forward
       TIMER_A0->CTL |= 0x0030;  // MC=11 up/down mode                 NEW
       TIMER_A0->CCTL[3] &= ~0X00E0; // OUTMOD (bits 7-5) set to 000   NEW
       TIMER_A0->CCTL[3] |= 0X0040; // OUTMOD (bits 7-5) set to 010    NEW
       TIMER_A0->CCTL[3] &= ~0X0002; // OUT (bit 2) set to 0           NEW
       TIMER_A0->CCTL[4] &= ~0X00E0; // OUTMOD (bits 7-5) set to 000   NEW
       TIMER_A0->CCTL[4] |=  0X0040; // OUTMOD (bits 7-5) set to 000   NEW
       TIMER_A0->CCTL[4] &= ~0X0002; // OUT (bit 2) set to 0           NEW
       P3->OUT |= 0xC0;      //motor drivers on
}

// ------------Motor_Right------------
// Turn the robot to the right by running the
// left wheel forward and the right wheel
// backward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Right(uint16_t leftDuty, uint16_t rightDuty){ 
    TIMER_A0->CCR[3] = rightDuty;
    TIMER_A0->CCR[4] = leftDuty;
    P5->OUT &= ~0x10;       // left motor forward
    P5->OUT |= 0x20;        // right motor backward
    TIMER_A0->CTL |= 0x0030;  // MC=11 up/down mode                 NEW
    TIMER_A0->CCTL[3] &= ~0X00E0; // OUTMOD (bits 7-5) set to 000   NEW
    TIMER_A0->CCTL[3] |= 0X0040; // OUTMOD (bits 7-5) set to 010    NEW
    TIMER_A0->CCTL[3] &= ~0X0002; // OUT (bit 2) set to 0           NEW
    TIMER_A0->CCTL[4] &= ~0X00E0; // OUTMOD (bits 7-5) set to 000   NEW
    TIMER_A0->CCTL[4] |=  0X0040; // OUTMOD (bits 7-5) set to 000   NEW
    TIMER_A0->CCTL[4] &= ~0X0002; // OUT (bit 2) set to 0           NEW
    P3->OUT |= 0xC0;      //motor drivers on
}

// ------------Motor_Left------------
// Turn the robot to the left by running the
// left wheel backward and the right wheel
// forward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Left(uint16_t leftDuty, uint16_t rightDuty){ 
  // write this similar to Motor_Right above
      TIMER_A0->CCR[3] = rightDuty;
      TIMER_A0->CCR[4] = leftDuty;
      P5->OUT |= 0x10;       // left motor backward
      P5->OUT &= ~0x20;        // right motor forward
      TIMER_A0->CTL |= 0x0030;  // MC=11 up/down mode                 NEW
      TIMER_A0->CCTL[3] &= ~0X00E0; // OUTMOD (bits 7-5) set to 000   NEW
      TIMER_A0->CCTL[3] |= 0X0040; // OUTMOD (bits 7-5) set to 010    NEW
      TIMER_A0->CCTL[3] &= ~0X0002; // OUT (bit 2) set to 0           NEW
      TIMER_A0->CCTL[4] &= ~0X00E0; // OUTMOD (bits 7-5) set to 000   NEW
      TIMER_A0->CCTL[4] |=  0X0040; // OUTMOD (bits 7-5) set to 000   NEW
      TIMER_A0->CCTL[4] &= ~0X0002; // OUT (bit 2) set to 0           NEW
      P3->OUT |= 0xC0;      //motor drivers on
}

// ------------Motor_Backward------------
// Drive the robot backward by running left and
// right wheels backward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Backward(uint16_t leftDuty, uint16_t rightDuty){ 
  // write this similar to Motor_Right above
         TIMER_A0->CCR[3] = rightDuty;
         TIMER_A0->CCR[4] = leftDuty;
         P5->OUT |= 0x10;       // left motor backward
         P5->OUT |= 0x20;        // right motor backward
         TIMER_A0->CTL |= 0x0030;  // MC=11 up/down mode                 NEW
         TIMER_A0->CCTL[3] &= ~0X00E0; // OUTMOD (bits 7-5) set to 000   NEW
         TIMER_A0->CCTL[3] |= 0X0040; // OUTMOD (bits 7-5) set to 010    NEW
         TIMER_A0->CCTL[3] &= ~0X0002; // OUT (bit 2) set to 0           NEW
         TIMER_A0->CCTL[4] &= ~0X00E0; // OUTMOD (bits 7-5) set to 000   NEW
         TIMER_A0->CCTL[4] |=  0X0040; // OUTMOD (bits 7-5) set to 000   NEW
         TIMER_A0->CCTL[4] &= ~0X0002; // OUT (bit 2) set to 0           NEW
         P3->OUT |= 0xC0;      //motor drivers on
}
