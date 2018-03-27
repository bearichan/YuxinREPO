#include "OS.h"
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"

volatile uint32_t counter;
void (*PeriodicTask)(void);

int OS_AddPeriodicThread(void(*task)(void),uint32_t period, uint32_t priority){
	
  SYSCTL_RCGCTIMER_R |= 0x10;   // 0) activate TIMER4
  PeriodicTask = task;          // user function
  TIMER4_CTL_R = 0x00000000;    // 1) disable TIMER4A during setup
  TIMER4_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER4_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER4_TAILR_R = period-1;    // 4) reload value
  TIMER4_TAPR_R = 0;            // 5) bus clock resolution
  TIMER4_ICR_R = 0x00000001;    // 6) clear TIMER1A timeout flag
  TIMER4_IMR_R = 0x00000001;    // 7) arm timeout interrupt

  NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFF00FF)|0x00008000; // 8) priority 4
// interrupts enabled in the main program after all devices initialized
// vector number 37, interrupt number 21
  NVIC_EN2_R = 1 << 6;           // 9) enable IRQ 70 in NVIC
  TIMER4_CTL_R = 0x00000001;    // 10) enable TIMER4A
  return 0;
}

void OS_ClearTimePeriod(void){
  counter = 0;
}

uint32_t OS_ReadPeriodTime(void){
  return counter;
}

void dummy(void){}

void Timer4A_Handler(void){
//  PF2 ^= 0x04;
  TIMER4_ICR_R = TIMER_ICR_TATOCINT;

  (*PeriodicTask)();
//  PF2 ^= 0x04;
}
