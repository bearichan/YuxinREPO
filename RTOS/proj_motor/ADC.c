// ADC.c
// Runs on LM4F120/TM4C123
// ADC0 SS3 to be triggered by software and trigger a conversion, wait for it to finish, and return the result.
// ADC0 SS0 triggered by Timer0
// Daniel Valvano and Jonathan Valvano
// 3/9/17

/* This example accompanies the books
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
   ISBN: 978-1466468863, Jonathan Valvano, copyright (c) 2017

 Copyright 2017 by Jonathan W. Valvano, valvano@mail.utexas.edu
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


#include "ADC.h"
#include "OS.h"
#include "tm4c123gh6pm.h"
#define NVIC_EN0_INT14          0x00004000  // Interrupt 14 enable
#define NVIC_EN0_INT17          0x00020000  // Interrupt 17 enable

// This initialization function sets up the ADC according to the
// following parameters.  Any parameters not explicitly listed
// below are not modified:
// Max sample rate: <=1,000,000 samples/second
// Sequencer 0 priority: 1st (highest)
// Sequencer 1 priority: 2nd
// Sequencer 2 priority: 3rd
// Sequencer 3 priority: 4th (lowest)
// SS3 triggering event: software trigger
// SS3 1st sample source: programmable using variable 'channelNum' [0:11]
// SS3 interrupts: enabled but not promoted to controller
void ADC_Init(void){ 
  unsigned long port = SYSCTL_RCGCGPIO_R4;
	port = SYSCTL_RCGCGPIO_R4;
  SYSCTL_RCGCGPIO_R |= port;
  while((SYSCTL_PRGPIO_R&port)==0){};
  //      Ain0 is on PE3
  GPIO_PORTE_DIR_R &= ~0x08;  // 3.0) make PE3 input
	GPIO_PORTE_AFSEL_R |= 0x08; // 4.0) enable alternate function on PE3
	GPIO_PORTE_DEN_R &= ~0x08;  // 5.0) disable digital I/O on PE3
	GPIO_PORTE_AMSEL_R |= 0x08; // 6.0) enable analog functionality on PE3
	//      Ain1 is on PE2
	GPIO_PORTE_DIR_R &= ~0x04;  // 3.1) make PE2 input
	GPIO_PORTE_AFSEL_R |= 0x04; // 4.1) enable alternate function on PE2
	GPIO_PORTE_DEN_R &= ~0x04;  // 5.1) disable digital I/O on PE2
	GPIO_PORTE_AMSEL_R |= 0x04; // 6.1) enable analog functionality on PE2
											 //      Ain2 is on PE1
	GPIO_PORTE_DIR_R &= ~0x02;  // 3.2) make PE1 input
	GPIO_PORTE_AFSEL_R |= 0x02; // 4.2) enable alternate function on PE1
	GPIO_PORTE_DEN_R &= ~0x02;  // 5.2) disable digital I/O on PE1
	GPIO_PORTE_AMSEL_R |= 0x02; // 6.2) enable analog functionality on PE1
										 //      Ain3 is on PE0
	GPIO_PORTE_DIR_R &= ~0x01;  // 3.3) make PE0 input
	GPIO_PORTE_AFSEL_R |= 0x01; // 4.3) enable alternate function on PE0
	GPIO_PORTE_DEN_R &= ~0x01;  // 5.3) disable digital I/O on PE0
	GPIO_PORTE_AMSEL_R |= 0x01; // 6.3) enable analog functionality on PE0
 
  // 7) activate ADC0 
  SYSCTL_RCGCADC_R |= 0x00000001; 
  while((SYSCTL_PRADC_R&0x01)==0){};      

  ADC0_PC_R = 0x01;         // configure for 125K samples/sec
  ADC0_SSPRI_R = 0x3210;    // sequencer 0 is highest, sequencer 3 is lowest
	ADC0_ACTSS_R &= ~0x02;    // disable sample sequencer 1
  ADC0_EMUX_R = (ADC0_EMUX_R&0xFFFFFF0F)+0x50; // timer trigger event
  ADC0_SSMUX1_R = 0x3210; 				// initially sample channel 0
  ADC0_SSCTL1_R = 0x6000;					// set flag and end
  ADC0_IM_R |= 0x02;							// enable SS1 interrupts
  ADC0_ACTSS_R |= 0x02;						// enable sample sequencer 1
}

void ADC0Seq1_Handler(void) {
	unsigned long data[4];
	data[0] = ADC0_SSFIFO1_R&0xFFF;   // 3) read result
	data[1] = ADC0_SSFIFO1_R&0xFFF;   // 3) read result
	data[2] = ADC0_SSFIFO1_R&0xFFF;   // 3) read result
	data[3] = ADC0_SSFIFO1_R&0xFFF;   // 3) read result
  ADC0_ISC_R = 0x0002;             // 4) acknowledge completion
	OS_MailBox_Send(data);
}

//------------ADC_In------------
// Busy-wait Analog to digital conversion
// Input: none
// Output: 12-bit result of ADC conversion
void ADC_In(unsigned long *data){
  while((ADC0_RIS_R&0x02)==0){};   // 2) wait for conversion done
}

void ADC_Start(void) {
	ADC0_PSSI_R = 0x0002;            // 1) initiate SS1
}
