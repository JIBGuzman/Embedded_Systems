// TableTrafficLight.c
// Runs on LM4F120 or TM4C123
// Index implementation of a Moore finite state machine to operate
// a traffic light.
// Daniel Valvano, Jonathan Valvano
// July 20, 2013

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   Volume 1 Program 6.8, Example 6.4
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
   Volume 2 Program 3.1, Example 3.1

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// east facing red light connected to PB5
// east facing yellow light connected to PB4
// east facing green light connected to PB3
// north facing red light connected to PB2
// north facing yellow light connected to PB1
// north facing green light connected to PB0
// north facing car detector connected to PE1 (1=car present)
// east facing car detector connected to PE0 (1=car present)
#include "tm4c123gh6pm.h"
#include <stdint.h>
#include "SysTick.h"

//Defines for LED Ports and Button Sensors
#define LIGHT_PORTB (*((volatile unsigned long *)0x400053FC))
#define LIGHT_PORTF (*((volatile unsigned long *)0x40025038))
#define SENSOR      (*((volatile unsigned long *)0x40024022))

// Port E - Sensor Switches/Buttons
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400)) 
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_DATA_R 			(*((volatile unsigned long *)0x400243FC))
#define GPIO_PORTE_CR_R					(*((volatile unsigned long *)0x40024524))

// Port B - light LEDs
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400)) 
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_DATA_R 			(*((volatile unsigned long *)0x400053FC))
#define GPIO_PORTB_CR_R					(*((volatile unsigned long *)0x40005524))

// port F - right turn LED
#define GPIO_PORTF_LOCK_R				(*((volatile unsigned long *)0x40025520))
#define GPIO_LOCK_KEY						0x4C4F434B 
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400)) 
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R 				(*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_DATA_R 			(*((volatile unsigned long *)0x400253FC))
	
// Linked data structure
struct State{
	uint32_t Turn;
	uint32_t Out;
	uint32_t Time;
	uint32_t Next[8];
};

//Moore State Labels
#define goS 		0
#define waitS 	1
#define goW 		2
#define waitW 	3
#define rTurn 	4
#define rWaitW 	5

typedef const struct State STyp;

STyp FSM[6]={
	{0x02, 0x21, 600, {goS, waitS, goS, waitS, rTurn, waitS, rTurn, waitS}},
	{0x02, 0x22, 200, {goW, goW, goW, goW, goW, goW, goW, goW}},
	{0x02, 0x0C, 800, {goW, goW, waitW, waitW, rWaitW, rWaitW, rWaitW, rWaitW}},
	{0x02, 0x14, 200, {goS, goS, goS, goS, goS, goS, goS, goS}},
	{0x08, 0x21, 600, {goS, waitS, goS, waitS, rTurn, waitS, rTurn, waitS}},
	{0x02, 0x14, 200, {rTurn, rTurn, rTurn, rTurn, rTurn, rTurn, rTurn, rTurn}}
};

void PortB_Init(void);
void PortE_Init(void);
void PortF_Init(void);
void SysTick_Init(void);
void SysTick_Wait10ms(unsigned long delay);

unsigned long S;  // index to the current state 
unsigned long Input; 
int main(void){ 
	uint8_t CS;
	uint8_t Input;

	volatile unsigned long delay;
	
	PortB_Init();
	PortE_Init();
	PortF_Init();
	SysTick_Init();
	
	CS = goW; //Enables Interrupt
  
  while(1){
		LIGHT_PORTF = FSM[CS].Turn; //Right Turn LEDs
		LIGHT_PORTB = FSM[CS].Out; //6 LEDs for N/S and W/E
		SysTick_Wait10ms(FSM[CS].Time); //SysTick
		Input = SENSOR; //Reads input from sensors
		CS = FSM[CS].Next[Input]; //Takes next input
  }
}


// PB5, PB4, PB3, PB2, PB1, PB0 for LEDs red(PB2 PB5), yellow(PB1 PB4), green(PB0 PB3) 

void PortB_Init(void){ volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000002; 					// 1) Clock B
	delay = SYSCTL_RCGC2_R;									// 2) allow delay
	GPIO_PORTB_CR_R = 0x3F;									// 3) allow changes
	GPIO_PORTB_AMSEL_R = 0x00; 							// 4) disable analog function on PB5-0
	GPIO_PORTB_PCTL_R = 0x00000000; 				// 5) enable GPIO by clearing
	GPIO_PORTB_DIR_R = 0x3F; 								// 6) PB5-PB0 output
	GPIO_PORTB_AFSEL_R = 0x00; 							// 7) regular function on PB5-0
	GPIO_PORTB_DEN_R = 0x3F; 								// 8) enable digital on PB5-0
}

// PE2, PE1, PE0 for buttons west(PA2) south(PA3) 
void PortE_Init(void){ volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000010; 					// 1) Clock E
	delay = SYSCTL_RCGC2_R;									// 2) allow delay
	GPIO_PORTE_CR_R = 0x07;									// 3) allow changes
	GPIO_PORTE_AMSEL_R = 0x00; 							// 4) disable analog function on PE2-0
	GPIO_PORTE_PCTL_R = 0x00000000; 				// 5) enable GPIO by clearing
	GPIO_PORTE_DIR_R = 0xF8; 								// 6) PE2-0 output
	GPIO_PORTE_AFSEL_R = 0x00; 							// 7) regular function on PE2-0
	GPIO_PORTE_DEN_R = 0x07; 								// 8) enable digital on PE2-0
}

void PortF_Init(void){ volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000020; 					// 1) Clock F
	delay = SYSCTL_RCGC2_R;									// 2) allow delay
	GPIO_PORTF_LOCK_R = 0x4C4F434B;					// unlock PF0
	GPIO_PORTF_CR_R = 0x0E;									// 3) allow changes
	GPIO_PORTF_AMSEL_R = 0x00; 							// 4) disable analog function on PE2-0
	GPIO_PORTF_PCTL_R = 0x00000000; 				// 5) enable GPIO by clearing
	GPIO_PORTF_DIR_R = 0x0E; 								// 6) PE2-0 output
	GPIO_PORTF_AFSEL_R = 0x00; 							// 7) regular function on PE2-0
	GPIO_PORTF_DEN_R = 0x0E; 								// 8) enable digital on PE2-0
	GPIO_PORTF_PUR_R = 0x00; 								// enable pull up resistors
}


