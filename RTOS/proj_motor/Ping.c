#include "Ping.h"
#include "Timer.h"
#include "ST7735.h"
#include "tm4c123gh6pm.h"
#include "OS.h"

#define PING_PRIORITY 2
#define PING_PERIOD 8000000
#define PULSE_LEN 5

#define PING0_TRIG (*((volatile unsigned long *) 0x40005200))
#define PING0_ECHO (*((volatile unsigned long *) 0x40005100))
#define PING1_TRIG (*((volatile unsigned long *) 0x40005080))
#define PING1_ECHO (*((volatile unsigned long *) 0x40005040))
#define PING2_TRIG (*((volatile unsigned long *) 0x40005020))
#define PING2_ECHO (*((volatile unsigned long *) 0x40005010))

unsigned long Ping0_Rise = 0;
unsigned long Ping0_Fall = 0; 
unsigned long Ping1_Rise = 0;
unsigned long Ping1_Fall = 0; 
unsigned long Ping2_Rise = 0;
unsigned long Ping2_Fall = 0; 

unsigned long Ping0_PulseWidth = 0;
unsigned long Ping1_PulseWidth = 0;
unsigned long Ping2_PulseWidth = 0;

unsigned long StartCritical(void);
void EndCritical(unsigned long primask);

void Ping0_Pin_Init() {
  SYSCTL_RCGCGPIO_R |= 0x02;            
  while((SYSCTL_PRGPIO_R&0x02) == 0){}; 
  
	GPIO_PORTB_DIR_R |= 0x80;             // make PB7 out
	GPIO_PORTB_AFSEL_R &= ~0xC0; // disable alt on PB6 and PB7
	GPIO_PORTB_DEN_R |= 0xC0;   // enable digital I/O on PB6 and PB7
	GPIO_PORTB_AMSEL_R = 0;  // disable analog
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0x00FFFFFF)+0x00000000; // PB6 and 7 as GPIO
	
	// set as both edge trigger
	GPIO_PORTB_IBE_R |= 0x40; 
	GPIO_PORTB_ICR_R = 0x40;
	GPIO_PORTB_IM_R |= 0x40;
	
	NVIC_PRI0_R = (NVIC_PRI0_R&0xFFFF00FF)|(PING_PRIORITY << 13);
	NVIC_EN0_R |= 0x02;
}

void Ping1_Pin_Init() {
  SYSCTL_RCGCGPIO_R |= 0x02;            
  while((SYSCTL_PRGPIO_R&0x02) == 0){}; 
  
	GPIO_PORTB_DIR_R |= 0x20;             // make PB5 out
	GPIO_PORTB_AFSEL_R &= ~0x30; // disable alt on PB4 and PB5
	GPIO_PORTB_DEN_R |= 0x30;   // enable digital I/O on PB4 and PB5
	GPIO_PORTB_AMSEL_R = 0;  // disable analog
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFF00FFFF)+0x00000000; // PB4 and 5 as GPIO
	
	// set as both edge trigger
	GPIO_PORTB_IBE_R |= 0x10; 
	GPIO_PORTB_ICR_R = 0x10;
	GPIO_PORTB_IM_R |= 0x10;
	
	NVIC_PRI0_R = (NVIC_PRI0_R&0xFFFF00FF)|(PING_PRIORITY << 13);
	NVIC_EN0_R |= 0x02;
}

void Ping2_Pin_Init() {
  SYSCTL_RCGCGPIO_R |= 0x02;            
  while((SYSCTL_PRGPIO_R&0x02) == 0){}; 
  
	GPIO_PORTB_DIR_R |= 0x08;             // make PB3 out
	GPIO_PORTB_DIR_R &= ~0x04;
	GPIO_PORTB_AFSEL_R &= ~0x0C; // disable alt on PB2 and PB3
	GPIO_PORTB_AMSEL_R = 0;  // disable analog
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00000000; // PB3 and 2 as GPIO
	GPIO_PORTB_DEN_R |= 0x0C;   // enable digital I/O on PB3 and PB2
	
	// set as both edge trigger
	GPIO_PORTB_IBE_R |= 0x04; 
	GPIO_PORTB_ICR_R = 0x04;
	GPIO_PORTB_IM_R |= 0x04;
	
	NVIC_PRI0_R = (NVIC_PRI0_R&0xFFFF00FF)|(PING_PRIORITY << 13);
	NVIC_EN0_R |= 0x02;
}

void Ping0_Pulse() {
	long sr;
	unsigned long start; 
	
	sr = StartCritical(); 
	
	PING0_TRIG = 1; 
	start = OS_Time();
	while(OS_TimeDifference(start, OS_Time()) < PULSE_LEN) {}
	PING0_TRIG = 0; 
	
	EndCritical(sr); 
}
	
void Ping1_Pulse() {
	long sr;
	unsigned long start; 
	
	sr = StartCritical(); 
	
	PING1_TRIG = 2; 
	start = OS_Time();
	while(OS_TimeDifference(start, OS_Time()) < PULSE_LEN) {}
	PING1_TRIG = 0; 
	
	EndCritical(sr); 
}

void Ping2_Pulse() {
	long sr;
	unsigned long start; 
	
	sr = StartCritical(); 
	
	PING2_TRIG ^= 0x08; 
	start = OS_Time();
	while(OS_TimeDifference(start, OS_Time()) < PULSE_LEN) {}
	PING2_TRIG = 0; 
	
	EndCritical(sr); 
}


void Ping0_Init(void) {
	Ping0_Pin_Init(); 
	Timer0A_Init(&Ping0_Pulse, PING_PERIOD);
}

void Ping1_Init(void) {
	Ping1_Pin_Init(); 
	Timer1A_Init(&Ping1_Pulse, PING_PERIOD);
}
void Ping2_Init(void) {
	Ping2_Pin_Init(); 
	Timer2A_Init(&Ping2_Pulse, PING_PERIOD);
}

void Ping0_Read() {
	unsigned long value = PING0_ECHO;
	if( value ) {
		Ping0_Rise = OS_Time();
	} else {
		Ping0_Fall = OS_Time();
	}
	
	if(Ping0_Fall > Ping0_Rise) {
		Ping0_PulseWidth = OS_TimeDifference(Ping0_Rise, Ping0_Fall); 
	}
}

void Ping1_Read() {
	unsigned long value = PING1_ECHO;
	if( value ) {
		Ping1_Rise = OS_Time();
	} else {
		Ping1_Fall = OS_Time();
	}
	
	if(Ping1_Fall > Ping1_Rise) {
		Ping1_PulseWidth = OS_TimeDifference(Ping1_Rise, Ping1_Fall); 
	}
}

void Ping2_Read() {
	unsigned long value = PING2_ECHO;
	if( value ) {
		Ping2_Rise = OS_Time();
	} else {
		Ping2_Fall = OS_Time();
	}
	
	if(Ping2_Fall > Ping2_Rise && Ping2_Rise != 0) {
		Ping2_PulseWidth = OS_TimeDifference(Ping2_Rise, Ping2_Fall);
	}
}


void GPIOPortB_Handler(void) {
	long status = GPIO_PORTB_MIS_R; 
	if( (status >> 6) & 1 ) {
		Ping0_Read();
	} else if( (status >> 4) & 1 ) {
		Ping1_Read();
	} else if( (status >> 2) & 1) {
		Ping2_Read(); 
	}
	GPIO_PORTB_ICR_R = 0x04;
}
