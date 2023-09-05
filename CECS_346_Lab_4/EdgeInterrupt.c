// EdgeInterrupt.c
// Based on TExaSware\C12_EdgeInterrupt

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers"
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   Volume 1, Program 9.4
   
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
   Volume 2, Program 5.6, Section 5.5

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

#include <stdint.h> // C99 data types
#include "tm4c123gh6pm.h"

#define NVIC_ST_RELOAD_R			(*((volatile unsigned long *)0xE000E014))
#define NVIC_EN0_R            (*((volatile unsigned long *)0xE000E100)) //Set Enable Register
#define NVIC_PR17             (*((volatile unsigned long *)0xE000E41C)) //Priority Register
#define GPIO_PORTF_DATA_R     (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R      (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R    (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R      (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R      (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R     (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R       (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R    (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R     (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_R        (*((volatile unsigned long *)0x400FE108))
	
// Function Prototypes (from startup.s)
void DisableInterrupts(); // Disable interrupts
void EnableInterrupts();  // Enable interrupts
void WaitForInterrupt();  // Go to low power mode while waiting for the next interrupt

// Function Prototypes
void EdgeCounter_Init(void);  // Initialize edge trigger interrupt for PF0 (SW2) rising edge
void PortF_LEDInit(void);     // Initialize Port F LEDs
void SysTick_Init(unsigned long period);      // Initialize SysTick timer for 0.1s delay with interrupt enabled

void GPIOPortF_Handler(void); // Handle GPIO Port F interrupts
void SysTick_Handler(void);   // Handle SysTick generated interrupts

// global variable visible in Watch and Memory window of debugger
// increments at least once per button release
volatile uint32_t RisingEdges = 0;
volatile unsigned long counts = 0;

int main(void){
  PortF_LEDInit();
	EdgeCounter_Init();           // initialize GPIO Port F interrupt
	SysTick_Init(160000);
	
	// initialize LaunchPad LEDs to red
	GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~0x0E) + 0x02;
	
  while(1){
		WaitForInterrupt();
  }
}

// Color    LED(s) PortF
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08

// Initialize Port F LEDs
void PortF_LEDInit(void) { volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020; 			// activate port F
	delay = SYSCTL_RCGC2_R; 						// delay
	GPIO_PORTF_DIR_R |= 0x0E; 					// set direction as output(?) PF3-1
	GPIO_PORTF_AFSEL_R &= 0x0E; 				// disable alternate function on PF3-1
	GPIO_PORTF_DEN_R |= 0x0E; 					// digital enable PF3-1
	GPIO_PORTF_PCTL_R &= ~0x0000FFF0; 	// set PF3-1 as GPIO
	GPIO_PORTF_AMSEL_R &= ~0x0E; 				// disable analog PF3-1
}

// Initialize SysTick timer for 0.1s delay with interrupt enabled
void SysTick_Init(unsigned long period) {
	NVIC_ST_CTRL_R = 0; 																							// disable Systick duyring setup
	NVIC_ST_RELOAD_R = period-1; 																			// reload value
	NVIC_ST_CURRENT_R = 0; 																						// any write to current clears it
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x00FFFFFF) | 0x40000000; 		// priority 2
	NVIC_ST_CTRL_R = 0x07;
	EnableInterrupts();
}

// Initialize edge trigger interrupt for PF0 (SW2) rising edge
void EdgeCounter_Init() {                          
	SYSCTL_RCGC2_R |= 0x00000020;															// activate port F clock
	RisingEdges = 0;																					// initialize counter
	GPIO_PORTF_DIR_R &= ~0x01;																// make PF0 input onboard button
	GPIO_PORTF_AFSEL_R &= ~0x01;															// disable alternate function on PF0
	GPIO_PORTF_DEN_R |= 0x01;																	// enable digital I/O on PF0
	GPIO_PORTF_PCTL_R &= ~0x0000000F;													// configure PF0 as GPIO
	GPIO_PORTF_AMSEL_R = 0;																		// disable analog function on PF0
	GPIO_PORTF_PUR_R |= 0x01;																	// enable weak pullup on PF0
	
	GPIO_PORTF_IS_R &= ~0x01;																	// configure PF0 as edge sensitive
	GPIO_PORTF_IBE_R &= ~0x01;																// PF0 is not both edges
	GPIO_PORTF_IEV_R &= 0x01;																	// PF0 rising event
	GPIO_PORTF_ICR_R = 0x01;																	// clear flag 0
	GPIO_PORTF_IM_R |= 0x01;																	// arm interrupt on PF0
	NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF) | 0x00A00000; 		// priority 5
	NVIC_EN0_R = 0x40000000;																	// enable interrupt 30 in NVIC
	EnableInterrupts();																				// clears I bit
}

// Handle GPIO Port F interrupts. When Port F interrupt triggers, do what's necessary then increment global variable RisingEdges
void GPIOPortF_Handler() {
	GPIO_PORTF_ICR_R = 0x01;	// ack flag 0
	RisingEdges = RisingEdges + 1;
}

// Handle SysTick generated interrupts. When timer interrupt triggers, do what's necessary then toggle red and blue LEDs at the same time
void SysTick_Handler(void) {
	GPIO_PORTF_DATA_R ^= 0x02;
	GPIO_PORTF_DATA_R ^= 0x04;
	counts = counts + 1;
}
