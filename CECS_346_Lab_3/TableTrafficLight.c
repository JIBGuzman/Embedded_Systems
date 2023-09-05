// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PE5
// east/west yellow light connected to PE4
// east/west green light connected to PE3
// north/south facing red light connected to PE2
// north/south facing yellow light connected to PE1
// north/south facing green light connected to PE0

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"
#include "stdint.h"

// ***** 2. Global Declarations Section *****

// Declare Port A/E and "open" their specific functions
// Port A 2 Switches (PA2 West and PA3 South)
 
#define GPIO_PORTA_DIR_R        (*((volatile unsigned long *)0x40004400))  // 4000_4_400 
#define GPIO_PORTA_AFSEL_R      (*((volatile unsigned long *)0x40004420))	 // 4000 = port specific offset
#define GPIO_PORTA_DEN_R        (*((volatile unsigned long *)0x4000451C))	 // _4_ = port specific address
#define GPIO_PORTA_AMSEL_R      (*((volatile unsigned long *)0x40004528))	 // 400 = GPIO specific function to be init
#define GPIO_PORTA_PCTL_R       (*((volatile unsigned long *)0x4000452C))
#define SYSCTL_RCGC2_GPIOA      0x00000000  // port A Clock Gating Control
#define SYSCTL_RCGC2_R 					(*((volatile unsigned long *)0x400FE108)) 
#define SENSOR 									(*((volatile unsigned long *)0x40004400C)) // PA2, PA3 for switches 1100 = C

// Port E 3 Control LEDS (PE2 Red, PE1 Yellow, PE0 Green)
  
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400)) 
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define SYSCTL_RCGC2_GPIOE      0x00000010  // port E Clock Gating Control
#define SYSCTL_RCGC2_R 					(*((volatile unsigned long *)0x400FE108)) 
#define SIGNAL									(*((volatile unsigned long *)0x40024007)) 	 // PE2, PE1, PE0 for traffic LED's 0111 = 7

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// ***** 3. Subroutines Section *****
void portA_init(void); // Port A initialization
void portE_init(void); // Port E initialization

// Port A = 2 switches (west PA2, south PA3) aka sensors
// must be a delay after after GPIO module clock is enabled before GPIO module registers accessed
void PortA_Init(void) { volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000010;// 1) clock A
  delay = SYSCTL_RCGC2_R;      // 2) no need to unlock
  GPIO_PORTA_AMSEL_R &= ~0x1F; // 3) disable analog function on PA4-0 **could be improved on the ports**
  GPIO_PORTA_PCTL_R &= ~0x000000FF; // 4) enable regular GPIO
  GPIO_PORTA_DIR_R &= ~0x1F;   // 5) inputs on PA4-0 
  GPIO_PORTA_AFSEL_R &= ~0x1F; // 6) regular function on PA4-0
  GPIO_PORTA_DEN_R |= 0x1F;    // 7) enable digital on PA4-0 0001 1111 = 1F
}
// Port E = 3 LED's (red PE2, yellow PE1, green PE0)
void PortE_Init(void) { volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x11;      // 1) clock E
  delay = SYSCTL_RCGC2_R;      // 2) no need to unlock
  GPIO_PORTE_AMSEL_R &= ~0x3F; // 3) disable analog function on PE5-0
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // 4) enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x3F;   // 5) inputs on PE5-0 
  GPIO_PORTE_AFSEL_R &= ~0x3F; // 6) regular function on PE5-0
  GPIO_PORTE_DEN_R |= 0x3F;    // 7) enable digital on PE5-0 0011 1111 = 3F
}

#define goN 	0
#define waitN 1
#define goE 	2
#define waitE	3

typedef const struct State StateType;

struct State { 
    uint32_t Out; 
    int32_t Time; 
    uint32_t Next[4]; 
};
StateType  FSM[4] = {
    {0x21, 1000, {goN, waitN, goN, waitN}}, //row0
    {0x22, 2000,  {goE, goE, goE, goE}},       //row1
    {0x0C, 3000, {goE, goE, waitE, waitE}}, //row2
    {0x14, 4000,  {goN, goN, goN, goN}}     //row3
};

void delay10ms(unsigned long delay){
    unsigned long volatile time;

    time = (727240*200/91);//* delay;  // 0.1sec * delay

    while(time){
        time--;
		}
}
		
int main(void){ 
	uint8_t CS;
	uint8_t Input;
	
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	
	EnableInterrupts();

	// initialize ports and timer
	PortA_Init();
	PortE_Init();
	
	CS = goN; // initial state
	while(1) {
		SIGNAL = FSM[CS].Out; // set signals
		delay10ms(FSM[CS].Time);
		Input = SENSOR; // read sensors
		CS = FSM[CS].Next[Input];
	}
}
