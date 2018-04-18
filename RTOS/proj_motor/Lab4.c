//*****************************************************************************
//
// Lab4.c - user programs, File system, stream data onto disk
//*****************************************************************************

// Jonathan W. Valvano 3/7/17, valvano@mail.utexas.edu
// EE445M/EE380L.6 
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file to do Lab 4
// as long as the basic functionality is simular
// 1) runs on your Lab 2 or Lab 3
// 2) implements your own eFile.c system with no code pasted in from other sources
// 3) streams real-time data from robot onto disk
// 4) supports multiple file reads/writes
// 5) has an interpreter that demonstrates features
// 6) interactive with UART input, and switch input

// LED outputs to logic analyzer for OS profile 
// PF1 is preemptive thread switch
// PF2 is periodic task
// PF3 is SW1 task (touch PF4 button)

// Button inputs
// PF0 is SW2 task 
// PF4 is SW1 button input

// Analog inputs
// PE3 sequencer 3, channel 3, J8/PE0, sampling in DAS(), software start
// PE0 timer-triggered sampling, channel 0, J5/PE3, 50 Hz, processed by Producer
//******Sensor Board I/O*******************
// **********ST7735 TFT and SDC*******************
// ST7735
// Backlight (pin 10) connected to +3.3 V
// MISO (pin 9) unconnected
// SCK (pin 8) connected to PA2 (SSI0Clk)
// MOSI (pin 7) connected to PA5 (SSI0Tx)
// TFT_CS (pin 6) connected to PA3 (SSI0Fss)
// CARD_CS (pin 5) connected to PB0
// Data/Command (pin 4) connected to PA6 (GPIO), high for data, low for command
// RESET (pin 3) connected to PA7 (GPIO)
// VCC (pin 2) connected to +3.3 V
// Gnd (pin 1) connected to ground

// HC-SR04 Ultrasonic Range Finder 
// J9X  Trigger0 to PB7 output (10us pulse)
// J9X  Echo0    to PB6 T0CCP0
// J10X Trigger1 to PB5 output (10us pulse)
// J10X Echo1    to PB4 T1CCP0
// J11X Trigger2 to PB3 output (10us pulse)
// J11X Echo2    to PB2 T3CCP0
// J12X Trigger3 to PC5 output (10us pulse)
// J12X Echo3    to PF4 T2CCP0

// Ping))) Ultrasonic Range Finder 
// J9Y  Trigger/Echo0 to PB6 T0CCP0
// J10Y Trigger/Echo1 to PB4 T1CCP0
// J11Y Trigger/Echo2 to PB2 T3CCP0
// J12Y Trigger/Echo3 to PF4 T2CCP0

// IR distance sensors
// J5/A0/PE3
// J6/A1/PE2
// J7/A2/PE1
// J8/A3/PE0  

// ESP8266
// PB1 Reset
// PD6 Uart Rx <- Tx ESP8266
// PD7 Uart Tx -> Rx ESP8266

// Free pins (debugging)
// PF3, PF2, PF1 (color LED)
// PD3, PD2, PD1, PD0, PC4

#include "OS.h"
#include "tm4c123gh6pm.h"
#include <stdint.h>
#include "Processing.h"
#include "ST7735.h"
#include "pwm.h"
#define IR
//#define PING

//#define PF0  (*((volatile unsigned long *)0x40025004))
//#define PF1  (*((volatile unsigned long *)0x40025008))
//#define PF2  (*((volatile unsigned long *)0x40025010))
//#define PF3  (*((volatile unsigned long *)0x40025020))
//#define PF4  (*((volatile unsigned long *)0x40025040))
  

	
void WaitForInterrupt(void);

unsigned long NumCreated;   // number of foreground threads created

int Running;                // true while robot is running

#define TIMESLICE 2*TIME_1MS  // thread switch time in system time units

//******** IdleTask  *************** 
// foreground thread, runs when no other work needed
// the system is put into an idle state that prevents OS from crashing
// system waits for interrupt and continues execution
// because the system goes into an idle state, the system wastes less power
// inputs:  none
// outputs: none
unsigned long Idlecount=0;
void IdleTask(void){ 
  for(;;){
		WaitForInterrupt();
	}
}

//******** Interpreter **************
// your intepreter from Lab 4 
// foreground thread, accepts input from UART port, outputs to UART port
// inputs:  none
// outputs: none
extern void Interpreter(void); 

//*******************lab 4 main **********
int main(void){        // lab 4 real main
  OS_Init();           // initialize, disable interrupts
  ST7735_InitR(INITR_REDTAB);
  ST7735_FillScreen(0);
  Running = 0;         // robot not running
  Motor_PortF_Init();
 
  Left_Init(12500, 12500,1);          // initialize PWM0, 100 Hz
  Right_InitB(12500, 12500,1);   // initialize PWM0, 100 Hz
  
//  Left_InitB(12500, 1000,1);          // initialize PWM0, 100 Hz
//  Right_Init(12500, 1000,1);   // initialize PWM0, 100 Hz
  
  Servo_Init(25000, 2000);   
  //  Servo_Init(25000, 2550);   
  
//*******attach background tasks***********

  NumCreated = 0 ;
// create initial foreground threads
  //NumCreated += OS_AddThread(&Interpreter,128,2); 
  NumCreated += OS_AddThread(&IdleTask,128,7);  // runs when nothing useful to do
  Processing_Init(); 

  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}
