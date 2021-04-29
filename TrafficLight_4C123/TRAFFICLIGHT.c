// TRAFFICLIGHT.c
// Runs on LM4F120 or TM4C123
// Request an interrupt on the falling edge of PF4 (when the user
// button is pressed) and increment a counter in the interrupt.  Note
// that button bouncing is not addressed.
// Daniel Valvano
// August 30, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers"
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2019
   Volume 1, Program 9.4
   
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2019
   Volume 2, Program 5.6, Section 5.5

 Copyright 2019 by Jonathan W. Valvano, valvano@mail.utexas.edu
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
//************************************************************************************************************
// user button connected to PF4 (increment counter on falling edge)

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"

//
// GLOBAL VARIABLE: visible in Watch window of debugger
// increments at least once per button press
//


volatile uint32_t FallingEdges = 0;

void  PortFInt_Init(void){                          
  SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
  FallingEdges = 0;             // (b) initialize counter
  GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4   
  GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}

void  PortBInt_Init(void){                          
  SYSCTL_RCGCGPIO_R |= 0x00000002; // (a) activate clock for port B
  FallingEdges = 0;             // (b) initialize counter
  GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4   
  GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}
void  PortEInt_Init(void){                          
  SYSCTL_RCGCGPIO_R |= 0x00000010; // (a) activate clock for port E
  FallingEdges = 0;             // (b) initialize counter
  GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4   
  GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}

//*****************************************************************************
//Traffic Light Code
//
// FUNCTION PROTOTYPES: Each subroutine defined
//
void PortFInt_Init(void);// Initializes the port F
void PortBInt_Init(void);// Initializes the port B
void PortEInt_Init(void);// Initializes the port E
//void PortsInit(void); 
void PLL_Init(void); // Initializes PLL to run clock Cycles at 80Mhz
void SysTick_Init(void);                         // SysTick initialization
void SysTick_Wait(unsigned long delay);          // SysTick wait function
void SysTick_Wait10ms(unsigned long wait_time_ms); // SysTick delay function

//
//FINTITE STATE MACHINE STRUCT
//
struct State{
	unsigned long TrafficLedsCars; //6 bits output to Six LEDs for Cars
	unsigned long TrafficLedsPed; //2 bits output to two Leds for Pedestrians
	unsigned long Time;    //delay in 10ms units
	unsigned long next[8]; //next states for inputs 0,1,2,3,4,5,6,7
};

typedef const struct State STGO;  //STGO - STOPGO
#define GoWest          0   //name of the states
#define WaitWest        1
#define GoSouth         2
#define WaitSouth       3
#define WalkPed         4
#define HurryOn_Ped1    5
#define HurryOff_Ped1   6
#define HurryOn_Ped2    7
#define HurryOff_Ped2   8

STGO FSM[9]={
		{0x0C,0x04,100,{0,0,1,1,1,1,1,1}},       //state 0
	  {0x14,0x04,50 ,{2,2,2,2,4,4,4,2}},       //state 1
		{0x21,0x04,100,{2,3,2,3,3,3,3,3}},       //state 2
		{0x22,0x04,50 ,{0,0,0,0,4,4,4,4}},       //state 3
		{0x24,0x08,100,{4,5,5,5,4,5,5,5}},       //state 4
		{0x24,0x04,25 ,{6,6,6,6,4,6,6,6}},       //state 5
		{0x24,0x00,10 ,{7,7,7,7,4,7,7,7}},       //state 6
		{0x24,0x04,25 ,{8,8,8,8,4,8,8,8}},       //state 7
		{0x24,0x00,10 ,{0,0,2,0,4,0,2,0}}        //state 8

};
unsigned long State;  //index to the current state
unsigned long Input_Switch; // input from switches

//
//main
//
int main(void){
	PortFInt_Init();
	PortBInt_Init();
	PortEInt_Init();
	//void PortsInit(); 
	SysTick_Init();
	
	while(1){
			GPIO_PORTF_DATA_R=FSM[State].TrafficLedsCars;    //set car signal lights
			GPIO_PORTB_DATA_R=FSM[State].TrafficLedsPed;    //set walk signal lights
			SysTick_Wait10ms(FSM[State].Time);
			Input_Switch=GPIO_PORTE_DATA_R;                      //read sensors
			State=FSM[State].next[Input_Switch];
			}
}

//
//INITIALIZE PORT F, B, AND E
//
void Ports_Init(void) {
    // clock
    volatile unsigned long delay;
 
    //  activation of clock for Port F, B, and E
    SYSCTL_RCGC2_R |= 0x00000032;
    delay           = SYSCTL_RCGC2_R; // allow time for clock to start

//Port F initialize
	
	GPIO_PORTF_LOCK_R  = 0x4C4F434B;  // 2) unlock GPIO Port F
    GPIO_PORTF_CR_R   |= 0x0A;        // allow changes to PF3, PF1
    GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
    GPIO_PORTF_PCTL_R  = 0x00;        // 4) PCTL GPIO on PF3, PF1
    GPIO_PORTF_DIR_R  |= 0x0A;        // 5) PF3, PF1 are outputs
    GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alternate function
    GPIO_PORTF_PUR_R   = 0x00;        // disable pull-up resistor
    GPIO_PORTF_DEN_R  |= 0x0A;        // 7) enable digital I/O on PF3, PF1
 // Port B
    GPIO_PORTB_LOCK_R  = 0x4C4F434B;  // 2) unlock GPIO Port B
    GPIO_PORTB_CR_R   |= 0x3F;        // allow changes to PB5-PB0
    GPIO_PORTB_AMSEL_R = 0x00;        // 3) disable analog function
    GPIO_PORTB_PCTL_R  = 0x00;        // 4) PCTL GPIO on PB5-PB0
    GPIO_PORTB_DIR_R  |= 0x3F;        // 5) PB5-PB0 are outputs
    GPIO_PORTB_AFSEL_R = 0x00;        // 6) disable alternate function
    GPIO_PORTB_PUR_R   = 0x00;        // disable pull-up resistor
    GPIO_PORTB_DEN_R  |= 0x3F;        // 7) enable digital I/O on PB5-PB0
 // Port E
    GPIO_PORTE_LOCK_R  = 0x4C4F434B;  // 2) unlock GPIO Port E
    GPIO_PORTE_CR_R   |= 0x07;        // allow changes to PE2-PE0
    GPIO_PORTE_AMSEL_R = 0x00;        // 3) disable analog function
    GPIO_PORTE_PCTL_R  = 0x00;        // 4) PCTL GPIO on PE2-PE0
    GPIO_PORTE_DIR_R   = 0x00;        // 5) PE2-PE0 are inputs
    GPIO_PORTE_AFSEL_R = 0x00;        // 6) disable alternate function
    GPIO_PORTE_PUR_R   = 0x00;        // disable pull-up resistor
    GPIO_PORTE_DEN_R  |= 0x07;        // 7) enable digital I/O on PE2-PE0
}
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
  NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock
}

void SysTick_Wait(unsigned long delay1){
  NVIC_ST_RELOAD_R = delay1-1;  // # of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to "CURRENT" clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}

// 800000*12.5ns equals 10ms
void SysTick_Wait10ms(unsigned long delay1){
  unsigned long i;
  for(i=0; i<delay1; i++){
    SysTick_Wait(800000);  // wait 10ms  -- 80000 is 1ms, 800000is 10ms
  }
}
	
	
	
//void  PortFInt_Init(void){                          
//  SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
//  FallingEdges = 0;             // (b) initialize counter
//  GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
//  GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
//  GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4   
//  GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO
//  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
//  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
//  GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
//  GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
//  GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
//  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
//  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
//  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
//  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
//}

//void  PortBInt_Init(void){                          
//  SYSCTL_RCGCGPIO_R |= 0x00000002; // (a) activate clock for port B
//  FallingEdges = 0;             // (b) initialize counter
//  GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
//  GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
//  GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4   
//  GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO
//  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
//  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
//  GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
//  GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
//  GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
//  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
//  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
//  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
//  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
//}
//void  PortEInt_Init(void){                          
//  SYSCTL_RCGCGPIO_R |= 0x00000010; // (a) activate clock for port E
//  FallingEdges = 0;             // (b) initialize counter
//  GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
//  GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
//  GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4   
//  GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO
//  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
//  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
//  GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
//  GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
//  GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
//  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
//  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
//  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
//  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
//}

//void GPIOPortF_Handler(void){
//  GPIO_PORTF_ICR_R = 0x10;      // acknowledge flag4
//  FallingEdges = FallingEdges + 1;
//}


