#include <stdint.h> 
#include "tm4c123gh6pm.h"

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // Go to low power mode while waiting for the next interrupt

// Function Prototypes
void EdgeCounter_Init(void);  // Initialize edge trigger interrupt for PF0 (SW2) rising edge
void PortF_Init();     // Initialize Port F
void PortB_Init();     // Initialize Port F
void GPIOPortF_Handler(void); // Handle GPIO Port F interrupts
void delay(long);   // Delay function type long

//0x03 is 0011, 0x06 is 0110, 0x0C is 1100, 0x09 is 1001
unsigned int CW_FStep[4] = {0x03, 0x06, 0x0C, 0x09}; //Clockwise
unsigned int CCW_FStep[4] = {0x09, 0x0C, 0x06, 0x03}; //Counter-Clockwise

#define StepperMotor		(*((volatile unsigned long *)0x4000503C)) //PB0-PB3
#define Switches				(*((volatile unsigned long *)0x40025004)) //PF0-PF4
	
int main(void) {        
	PortF_Init();
	PortB_Init();
	EdgeCounter_Init();
  while(1){
		WaitForInterrupt();
  }
}

void PortF_Init() {     
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // unlock PF0
	GPIO_PORTF_CR_R |= 0x01;					// allow changes to PF0
  GPIO_PORTF_DIR_R &= ~0x11;    		// make PF0-PF4 as input 
  GPIO_PORTF_AFSEL_R &= ~0x11;  		// disable alt function on PF0
  GPIO_PORTF_DEN_R |= 0x11;     		// enable digital I/O on PF0-PF4
  GPIO_PORTF_PCTL_R &= ~0x000F000F; // GPIO Clear PCTL of PF0-PF4
  GPIO_PORTF_AMSEL_R = ~0x11;       // disable analog functionality on PF0-PF4
  GPIO_PORTF_PUR_R |= 0x11;     		// enable weak pull-up on PF0-PF4
}

void PortB_Init() {
  SYSCTL_RCGC2_R |= 0x00000002;     	// 1) F clock
  GPIO_PORTB_AMSEL_R &= ~0x0F;        // 3) disable analog functions 
  GPIO_PORTB_PCTL_R &= ~0x0000FFFF;   // 4) GPIO Clear PCTL of PB0-PB3
  GPIO_PORTB_DIR_R |= 0x0F;          	// 5) make PB0-PB3 as output
  GPIO_PORTB_AFSEL_R &= ~0x0F;        // 6) regular function on PB0-PB3
  GPIO_PORTB_DEN_R |= 0x0F;          	// 7) enable digital I/O on PB0-PB3
}

void EdgeCounter_Init(void) {
	GPIO_PORTF_IS_R &= ~0x11;				// configure PF0-PF4 as edge sensitive
	GPIO_PORTF_IBE_R &= ~0x11;			// PF0-PF4 not both edges
	GPIO_PORTF_IEV_R &= ~0x11;			// PF0-PF4 for Falling edge
	GPIO_PORTF_ICR_R = 0x11;				// clear flag 4
	GPIO_PORTF_IM_R |= 0x11;				// arm interrupt on PF0
	NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF) | 0x00A00000; 		// priority 5
	NVIC_EN0_R = 0x40000000; 				// enables interrupts
}

void delay(long a){
	volatile uint32_t Time;
	Time = 727240*200/(91*200) + a;
	while(Time) {
		Time--;
	}
}
void GPIOPortF_Handler() {
	GPIO_PORTF_ICR_R = 0x11;				// acknowledge flag 4
	if ((Switches&0x01) == 0x01) {
		int i = 0;
		int j = 0;
		for (i = 0; i < 128; i++) {
			for(j = 0; j < 4; j++) {
				StepperMotor = CW_FStep[j];
				delay(0);
			}
		}
	}
	else if ((Switches&0x10) == 0x10) {
		int i = 0;
		int j = 0;
		for (i = 0; i < 257; i++) {
			for (j = 0; j < 4; j++) {
				StepperMotor = CCW_FStep[j];
				delay(0);
			}
		}
	}
	delay(1000000);
}

	

