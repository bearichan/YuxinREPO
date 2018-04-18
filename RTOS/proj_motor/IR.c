#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "Timer.h"
#include "ADC.h"
#include "OS.h"
#include "ST7735.h"

#define PD0  (*((volatile unsigned long *)0x40007004))
#define PD1  (*((volatile unsigned long *)0x40007008))
#define PD2  (*((volatile unsigned long *)0x40007010))
#define PD3  (*((volatile unsigned long *)0x40007020))
#define IR_PERIOD 800000
#define NVIC_EN0_INT14          0x00004000  // Interrupt 14 enable
#define NVIC_EN0_INT17          0x00020000  // Interrupt 17 enable


void IR_Init(void) {
	ADC_Init();
	OS_MailBox_Init();
	Timer3A_Init(IR_PERIOD);
	NVIC_EN0_R |= 0x8000;
	NVIC_PRI3_R = (NVIC_PRI3_R&0x00FFFFFF)|0x40000000; //priority 2
}
