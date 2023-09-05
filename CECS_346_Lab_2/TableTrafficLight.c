// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"
#include <stdint.h>

//Input/Outputs
#define SENSOR 		(*((volatile unsigned long *)0x40004030)) //Enable PA2-3 for buttons west(PA2) south(PA3) 
#define SIGNAL 		(*((volatile unsigned long *)0x400240FC)) //Enable PE5-0 for LEDs red(PE2 PE5), yellow(PE1 PE4), green(PE0 PE3) 

// ***** 2. Global Declarations Section *****
// Declare Port A/E and "open" their specific functions

// Port A (APB offset)
#define GPIO_PORTA_DIR_R        (*((volatile unsigned long *)0x40004400))  // 4000_4_400 
#define GPIO_PORTA_AFSEL_R      (*((volatile unsigned long *)0x40004420))	 // 4000 = port specific offset
#define GPIO_PORTA_DEN_R        (*((volatile unsigned long *)0x4000451C))	 // _4_ = port specific address
#define GPIO_PORTA_AMSEL_R      (*((volatile unsigned long *)0x40004528))	 // 400 = GPIO specific function to be init
#define GPIO_PORTA_PCTL_R       (*((volatile unsigned long *)0x4000452C))
#define SYSCTL_RCGC2_GPIOA      0x00000001  // port A Clock Gating Control
#define SYSCTL_RCGC2_R 					(*((volatile unsigned long *)0x400FE108)) 
	
// Port E (APB offset)
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400)) 
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define SYSCTL_RCGC2_GPIOE      0x00000010  // port E Clock Gating Control
#define SYSCTL_RCGC2_R 					(*((volatile unsigned long *)0x400FE108)) 
	
// FUNCTION PROTOTYPES: Each subroutine defined
void delay10ms(uint8_t delay); //Delay Function
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void PortA_Init(void);
void PortE_Init(void);

// ***** 3. Subroutines Section *****

#define goS 	0
#define waitS 1
#define goW 	2
#define waitW 3

void PortA_Init(void){ volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000001; 					// 1) Clock A
	delay = SYSCTL_RCGC2_R;									// 2) allow delay
	GPIO_PORTA_AMSEL_R &= ~0x0C;						// 3) disable analog function on PA3-2
	GPIO_PORTA_PCTL_R  &= ~0x0000FF00;			// 4) enable regular GPIO
	GPIO_PORTA_DIR_R   &= ~0x0C;						// 5) Inputs on PA3-2 (input = 0, output = 1)
	GPIO_PORTA_AFSEL_R &= ~0x0C;						// 6) regular function on PA3-2
	GPIO_PORTA_DEN_R   |= 0x0C;							// 7) enable digital on PA3-2
}

void PortE_Init(void){ volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000010; 					// 1) Clock E
	delay = SYSCTL_RCGC2_R;									// 2) allow delay
	GPIO_PORTE_AMSEL_R &= ~0x3F;						// 3) disable analog function on PE5-0
	GPIO_PORTE_PCTL_R  &= ~0x00FFFFFF;			// 4) enable reuglar GPIO
	GPIO_PORTE_DIR_R   |= 0x3F;							// 5) outputs on PE5-0
	GPIO_PORTE_AFSEL_R &= ~0x3F;						// 6) regular fuinction on PE3-2
	GPIO_PORTE_DEN_R   |= 0x3F;							// 7) enable digital on PE3-2
}

struct State{
	uint32_t Out;
	uint32_t Time;
	uint32_t Next[4];
};

typedef const struct State STyp;

STyp FSM[4] = {
	{0x21, 3000, {goS, waitS, goS, waitS}},
	{0x22, 500, {goW, goW, goW, goW}},
	{0x0C, 3000, {goW, goW, waitW, waitW}},
	{0x14, 500, {goS, goS, goS, goS}}
};


void delay10ms(uint8_t delay){
	volatile uint32_t time;
	time = delay*727240*100/91;		//0.5 second * delay, 2 * 0.5 second = 1 second delay
	while(time){
		time--;
	}
}

int main(void){ 
	uint8_t CS;
	uint8_t Input;

	volatile unsigned long delay;
	
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	EnableInterrupts();
	
	PortA_Init();
	PortE_Init();
	
	CS = goS; 
  
  while(1){
    SIGNAL = ~FSM[CS].Out; 
		SIGNAL = FSM[CS].Out; 
		delay10ms(FSM[CS].Time);
		Input = (~SENSOR&(0x0C)) >> 0x02;
		Input = SENSOR >> 0x02;
		CS = FSM[CS].Next[Input];
  }
}



