//*****************************************************************************
// user.c
// Runs on LM4F120/TM4C123
// An example user program that initializes the simple operating system
//   Schedule three independent threads using preemptive round robin  
//   Each thread rapidly toggles a pin on Port D and increments its counter 
//   TIMESLICE is how long each thread runs

// Daniel Valvano
// January 29, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

   Programs 4.4 through 4.12, section 4.2

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
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
 
 /* development plan
 
 */

#include "OS.h"
#include "../inc/tm4c123gh6pm.h"
#include "ST7735.h"
#include "ADC.h"
#include "UART.h"
#include <string.h> 
#include <stdint.h>
#include <stdio.h>
#include "PLL.h"
#include "Timer2.h"


#define LEDS      (*((volatile uint32_t *)0x40025038))
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08
#define WHEELSIZE 8           // must be an integer multiple of 2
#define PARSE_MAX_SIZE 3

#define TIMESLICE               TIME_2MS*5    // thread switch time in system time units


#define GPIO_PORTD1             (*((volatile uint32_t *)0x40007008))
#define GPIO_PORTD2             (*((volatile uint32_t *)0x40007010))
#define GPIO_PORTD3             (*((volatile uint32_t *)0x40007020))
#define GPIO_PORTD_DIR_R        (*((volatile uint32_t *)0x40007400))
#define GPIO_PORTD_AFSEL_R      (*((volatile uint32_t *)0x40007420))
#define GPIO_PORTD_DEN_R        (*((volatile uint32_t *)0x4000751C))
#define GPIO_PORTD_AMSEL_R      (*((volatile uint32_t *)0x40007528))
#define GPIO_PORTD_PCTL_R       (*((volatile uint32_t *)0x4000752C))
#define SYSCTL_RCGCGPIO_R       (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_RCGCGPIO_R3      0x00000008  // GPIO Port D Run Mode Clock
                                            // Gating Control
#define SYSCTL_PRGPIO_R         (*((volatile uint32_t *)0x400FEA08))
#define SYSCTL_PRGPIO_R3        0x00000008  // GPIO Port D Peripheral Ready

#define Lab2 1
#define Lab3 0


#define DEBUG
//*********Prototype for FFT in cr4_fft_64_stm32.s, STMicroelectronics
void cr4_fft_64_stm32(void *pssOUT, void *pssIN, unsigned short Nbin);
//*********Prototype for PID in PID_stm32.s, STMicroelectronics
short PID_stm32(short Error, short *Coeff);

unsigned long NumCreated;   // number of foreground threads created
unsigned long PIDWork;      // current number of PID calculations finished
unsigned long FilterWork;   // number of digital filter calculations finished
unsigned long NumSamples;   // incremented every ADC sample, in Producer
#define FS 400              // producer/consumer sampling
#define RUNLENGTH (20*FS)   // display results and quit when NumSamples==RUNLENGTH
// 20-sec finite time experiment duration 

#define PERIOD TIME_500US   // DAS 2kHz sampling period in system time units
long x[64],y[64];           // input and output arrays for FFT

//---------------------User debugging-----------------------
unsigned long DataLost;     // data sent by Producer, but not received by Consumer
#if Lab2
long MaxJitter;             // largest time jitter between interrupts in usec
#define JITTERSIZE 64
unsigned long const JitterSize=JITTERSIZE;
unsigned long JitterHistogram[JITTERSIZE]={0,};
#endif
unsigned long TotalWithI1;
unsigned short MaxWithI1;

unsigned long Count1;   // number of times thread1 loops
unsigned long Count2;   // number of times thread2 loops
unsigned long Count3;   // number of times thread3 loops
unsigned long Count4;   // number of times thread4 loops
unsigned long Count5;   // number of times thread5 loops

#define PE0  (*((volatile unsigned long *)0x40024004))
#define PE1  (*((volatile unsigned long *)0x40024008))
#define PE2  (*((volatile unsigned long *)0x40024010))
#define PE3  (*((volatile unsigned long *)0x40024020))

void PortE_Init(void){ 
  SYSCTL_RCGCGPIO_R |= 0x10;       // activate port E
  while((SYSCTL_PRGPIO_R&0x10)==0){};  
  //unsigned long volatile delay;
  //delay = SYSCTL_RCGC2_R;        
  //delay = SYSCTL_RCGC2_R;      
  GPIO_PORTE_DIR_R |= 0x0F;    // make PE3-0 output heartbeats
  GPIO_PORTE_AFSEL_R &= ~0x0F;   // disable alt funct on PE3-0
  GPIO_PORTE_DEN_R |= 0x0F;     // enable digital I/O on PE3-0
  GPIO_PORTE_PCTL_R &= ~0x0000FFFF;
  GPIO_PORTE_AMSEL_R &= ~0x0F;;      // disable analog functionality on PF
}

#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
#define PF4       (*((volatile uint32_t *)0x40025040))
  
void PortF_Init(void){
  SYSCTL_RCGCGPIO_R |= 0x20;  // activate port F
  GPIO_PORTF_DIR_R |= 0x07;   // make PF2 output (PF2 built-in LED)
  GPIO_PORTF_AFSEL_R &= ~0x07;// disable alt funct on PF2
  GPIO_PORTF_DEN_R |= 0x07;   // enable digital I/O on PF2
                                // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;     // disable analog functionality on PF
}

#define PB1  (*((volatile unsigned long *)0x40005008))
#define PB2  (*((volatile unsigned long *)0x40005010))
#define PB3  (*((volatile unsigned long *)0x40005020))
#define PB4  (*((volatile unsigned long *)0x40005040))


void PortB_Init(void){ unsigned long volatile delay;
  SYSCTL_RCGC2_R |= 0x22;       // activate port B 
  GPIO_PORTB_DIR_R |= 0x3C;    // make PB5-2 output heartbeats
  GPIO_PORTB_AFSEL_R &= ~0x3C;   // disable alt funct on PB5-2
  GPIO_PORTB_DEN_R |= 0x3C;     // enable digital I/O on PB5-2
  GPIO_PORTB_PCTL_R = ~0x00FFFF00;
  GPIO_PORTB_AMSEL_R &= ~0x3C;      // disable analog functionality on PB
  GPIO_PORTF_DIR_R |= 0xA; // LEDs are outputs so write 1's to DIR
  GPIO_PORTF_AFSEL_R &= ~(0xA);    // Turn alternate function OFF for all input and output pins
  GPIO_PORTF_DEN_R |= 0xA;    // Digital enable all input and output pins
}

void Thread1(void){
  Count1 = 0;          
  for(;;){
    PB2 ^= 0x04;       // heartbeat
    Count1++;
    OS_Suspend();      // cooperative multitasking
  }
}
void Thread2(void){
  Count2 = 0;          
  for(;;){
    PB3 ^= 0x08;       // heartbeat
    Count2++;
    OS_Suspend();      // cooperative multitasking
  }
}
void Thread3(void){
  Count3 = 0;          
  for(;;){
    PB4 ^= 0x10;       // heartbeat
    Count3++;
    OS_Suspend();      // cooperative multitasking
  }
}

int Testmain1(void){  // Testmain1
  OS_Init();          // initialize, disable interrupts
  PortB_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1,128,1); 
  NumCreated += OS_AddThread(&Thread2,128,2); 
  NumCreated += OS_AddThread(&Thread3,128,3); 
  // Count1 Count2 Count3 should be equal or off by one at all times
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Second TEST**********
// Once the initalize test runs, test this (Lab 1 part 1)
// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores
void Thread1b(void){
  Count1 = 0;   
  for(;;){
    PB2 ^= 0x04;       // heartbeat
    Count1++;
  }
}
void Thread2b(void){
  Count2 = 0;  	
  for(;;){
    PB3 ^= 0x08;       // heartbeat
    Count2++;
  }
}
void Thread3b(void){
  Count3 = 0;    
  for(;;){
    PB4 ^= 0x10;       // heartbeat
    Count3++;
  }
}
int Testmain2(void){  // Testmain2
  OS_Init();           // initialize, disable interrupts
  PortB_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1b,128,1); 
  NumCreated += OS_AddThread(&Thread2b,128,2); 
  NumCreated += OS_AddThread(&Thread3b,128,3); 
  // Count1 Count2 Count3 should be equal on average
  // counts are larger than testmain1
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Third TEST**********
// Once the second test runs, test this (Lab 1 part 2)
// no UART1 interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// Timer interrupts, with or without period established by OS_AddPeriodicThread
// PortF GPIO interrupts, active low
// no ADC serial port or LCD output
// tests the spinlock semaphores, tests Sleep and Kill

Sema4Type Readyc;        // set in background
int Lost;
void BackgroundThread1c(void){   // called at 1000 Hz
  Count1++;
  OS_Signal(&Readyc);
}
void Thread5c(void){
  for(;;){
    OS_Wait(&Readyc);
    Count5++;   // Count2 + Count5 should equal Count1 
    Lost = Count1-Count5-Count2;
  }
}
unsigned long time;
void Thread2c(void){
  
  OS_InitSemaphore(&Readyc,0);
  Count1 = 0;    // number of times signal is called      
  Count2 = 0;    
  Count5 = 0;    // Count2 + Count5 should equal Count1  
  NumCreated += OS_AddThread(&Thread5c,128,3); 
  OS_AddPeriodicThread(&BackgroundThread1c,TIME_1MS,0); 
  for(;;){
    time = OS_Time();
    OS_Wait(&Readyc);
    Count2++;   // Count2 + Count5 should equal Count1
  }
}

void Thread3c(void){
  Count3 = 0;          
  for(;;){
    Count3++;
  }
}
void Thread4c(void){ int i;
  for(i=0;i<64;i++){
    Count4++;
    OS_Sleep(10);
  }
  OS_Kill();
  Count4 = 0;
}
void BackgroundThread5c(void){   // called when Select button pushed
  NumCreated += OS_AddThread(&Thread4c,128,3); 
}
      
int Testmain3(void){		   // Testmain3
  Count4 = 0;          
  OS_Init();           // initialize, disable interrupts

  ST7735_InitR(INITR_REDTAB);
  
  UART_Init();
  ST7735_Message(0,0, "test this ", 0);
  ST7735_Message(1,1, "test this ", 1);
	
// Count2 + Count5 should equal Count1
  NumCreated = 0 ;
  OS_AddSW1Task(&BackgroundThread5c,2);
  NumCreated += OS_AddThread(&Thread2c,128,2); 
  NumCreated += OS_AddThread(&Thread3c,128,3); 
  NumCreated += OS_AddThread(&Thread4c,128,3); 
  OS_Launch(TIME_2MS);  // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}


//******************* Lab 3 Measurement of context switch time**********
// Run this to measure the time it takes to perform a task switch
// UART0 not needed 
// SYSTICK interrupts, period established by OS_Launch
// first timer not needed
// second timer not needed
// SW1 not needed, 
// SW2 not needed
// logic analyzer on PF1 for systick interrupt (in your OS)
//                on PE0 to measure context switch time
void Thread8(void){       // only thread running
  while(1){
    PF1 ^= 0x02;      // debugging profile  
  }
}
int main(void){       // Testmain7
  PortF_Init();
  OS_Init();           // initialize, disable interrupts
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread8,128,2); 
  OS_Launch(TIME_1MS/10); // 100us, doesn't return, interrupts enabled in here
  return 0;             // this never executes
}


//------------------Task 1--------------------------------fix bw
// 2 kHz sampling ADC channel 1, using software start trigger
// background thread executed at 2 kHz
// 60-Hz notch high-Q, IIR filter, assuming fs=2000 Hz
// y(n) = (256x(n) -503x(n-1) + 256x(n-2) + 498y(n-1)-251y(n-2))/256 (2k sampling)
// y(n) = (256x(n) -476x(n-1) + 256x(n-2) + 471y(n-1)-251y(n-2))/256 (1k sampling)
long Filter(long data){
static long x[6]; // this MACQ needs twice
static long y[6];
static unsigned long n=3;   // 3, 4, or 5
  n++;
  if(n==6) n=3;     
  x[n] = x[n-3] = data;  // two copies of new data
  y[n] = (256*(x[n]+x[n-2])-503*x[n-1]+498*y[n-1]-251*y[n-2]+128)/256;
  y[n-3] = y[n];         // two copies of filter outputs too
  return y[n];
} 
//******** DAS *************** 
// background thread, calculates 60Hz notch filter
// runs 2000 times/sec
// samples channel 4, PD3,
// inputs:  none
// outputs: none
unsigned long DASoutput;
#if Lab2
void DAS(void){
unsigned long input;  
unsigned static long LastTime;  // time at previous ADC sample
unsigned long thisTime;         // time at current ADC sample
long jitter;                    // time between measured and expected, in us
  if(NumSamples < RUNLENGTH){   // finite time run
#ifdef DEBUG
    PB2 ^= 0x04;
#endif
    input = ADC_In();           // channel set when calling ADC_Init
#ifdef DEBUG
    PB2 ^= 0x04;
#endif
    thisTime = OS_Time();       // current time, 12.5 ns
    DASoutput = Filter(input);
    FilterWork++;        // calculation finished
    if(FilterWork>1){    // ignore timing of first interrupt
      unsigned long diff = OS_TimeDifference(LastTime,thisTime);
      if(diff>PERIOD){
        jitter = (diff-PERIOD+4)/8;  // in 0.1 usec
      }else{
        jitter = (PERIOD-diff+4)/8;  // in 0.1 usec
      }
      if(jitter > MaxJitter){
        MaxJitter = jitter; // in usec
      }       // jitter should be 0
      if(jitter >= JitterSize){
        jitter = JITTERSIZE-1;
      }
      JitterHistogram[jitter]++; 
    }
    LastTime = thisTime;
#ifdef DEBUG
    PB2 ^= 0x04;
#endif
  }
}
#endif
#if Lab3
void DAS(void){
unsigned long input;  
  if(NumSamples < RUNLENGTH){   // finite time run
#ifdef DEBUG
    PE0 ^= 0x01;
#endif
    input = ADC_In();           // channel set when calling ADC_Init
#ifdef DEBUG
    PE0 ^= 0x01;
#endif
    DASoutput = Filter(input);
    FilterWork++;        // calculation finished
#ifdef DEBUG
    PE0 ^= 0x01;
#endif
  }
}
#endif
//--------------end of Task 1-----------------------------

//------------------Task 2--------------------------------IO bound
// background thread executes with SW1 button
// one foreground task created with button push
// foreground treads run for 2 sec and die
// ***********ButtonWork*************
#if Lab3
extern unsigned long MaxJitter;
#endif
void ButtonWork(void){
  unsigned long myId = OS_Id(); 
#ifdef DEBUG
  PF1 ^= 0x02;
#endif
  ST7735_Message(1,0,"NumCreated =",NumCreated); 
#ifdef DEBUG
  PF1 ^= 0x02;
#endif
  OS_Sleep(50);     // set this to sleep for 50msec
  ST7735_Message(1,2,"PIDWork     =",PIDWork);
  ST7735_Message(1,3,"DataLost    =",DataLost);
  ST7735_Message(1,4,"Jitter 0.1us=",MaxJitter);
#ifdef DEBUG
  PF1 ^= 0x02;
#endif
  OS_Kill();  // done, OS does not return from a Kill
//  while(1);
}

//************SW1Push*************
// Called when SW1 Button pushed
// Adds another foreground task
// background threads execute once and return
void SW1Push(void){
#ifdef DEBUG
  PF3 = 0x08;
#endif
  if(OS_MsTime() > 20)// debounce
  {
    if(OS_AddThread(&ButtonWork,100,2))
    {
      NumCreated++; 
    }
    OS_ClearMsTime();  // at least 20ms between touches
  }
#ifdef DEBUG
  PF3 = 0x00;
#endif
}
//************SW2Push*************
// Called when SW2 Button pushed, Lab 3 only
// Adds another foreground task
// background threads execute once and return
void SW2Push(void){
  if(OS_MsTime() > 20){ // debounce
    if(OS_AddThread(&ButtonWork,100,2)){
      NumCreated++; 
    }
    OS_ClearMsTime();  // at least 20ms between touches
  }
}
//--------------end of Task 2-----------------------------

//------------------Task 3--------------------------------fix bw
// hardware timer-triggered ADC sampling at 400Hz
// Producer runs as part of ADC ISR
// Producer uses fifo to transmit 400 samples/sec to Consumer
// every 64 samples, Consumer calculates FFT
// every 2.5ms*64 = 160 ms (6.25 Hz), consumer sends data to Display via mailbox
// Display thread updates LCD with measurement

//******** Producer *************** 
// The Producer in this lab will be called from your ADC ISR
// A timer runs at 400Hz, started by your ADC_Collect
// The timer triggers the ADC, creating the 400Hz sampling
// Your ADC ISR runs when ADC data is ready
// Your ADC ISR calls this function with a 12-bit sample 
// sends data to the consumer, runs periodically at 400Hz
// inputs:  none
// outputs: none
void Producer(unsigned long data){  
  if(NumSamples < RUNLENGTH){   // finite time run
    NumSamples++;               // number of samples
    if(OS_Fifo_Put(data) == 0){ // send to consumer
      DataLost++;
    }
  } 
}
void Display(void); 

//******** Consumer *************** 
// foreground thread, accepts data from producer
// calculates FFT, sends DC component to Display
// inputs:  none
// outputs: none
void Consumer(void){ 
unsigned long data,DCcomponent;   // 12-bit raw ADC sample, 0 to 4095
unsigned long t;                  // time in 2.5 ms
unsigned long myId = OS_Id(); 
  ADC_Collect(5, FS, &Producer); 
  NumCreated += OS_AddThread(&Display,128,0); 
  while(NumSamples < RUNLENGTH) {
    #ifdef DEBUG
    PB4 = 0x10;
    #endif
    for(t = 0; t < 64; t++){   // collect 64 ADC samples
      data = OS_Fifo_Get();    // get from producer
      x[t] = data;             // real part is 0 to 4095, imaginary part is 0
    }
    #ifdef DEBUG
    PB4 = 0x00;
    #endif
    cr4_fft_64_stm32(y,x,64);  // complex FFT of last 64 ADC values
    DCcomponent = y[0]&0xFFFF; // Real part at frequency 0, imaginary part should be zero
    OS_MailBox_Send(DCcomponent); // called every 2.5ms*64 = 160ms
  }
  OS_Kill();  // done
}
//******** Display *************** 
// foreground thread, accepts data from consumer
// displays calculated results on the LCD
// inputs:  none                            
// outputs: none
void Display(void){ 
unsigned long data,voltage;
  ST7735_Message(0,1,"Run length = ",(RUNLENGTH)/FS);   // top half used for Display
  while(NumSamples < RUNLENGTH) 
  {
    data = OS_MailBox_Recv();
    voltage = 3000*data/4095;               // calibrate your device so voltage is in mV
#ifdef DEBUG
    PB3 = 0x08;
#endif
    OS_Sleep(50);
    ST7735_Message(0,2,"v(mV) =",voltage);  
#ifdef DEBUG
    PB3 = 0x00;
#endif
  }
  OS_Kill();  // done
}

//--------------end of Task 3-----------------------------

//------------------Task 4--------------------------------CPU bound
// foreground thread that runs without waiting or sleeping
// it executes a digital controller 
//******** PID *************** 
// foreground thread, runs a PID controller
// never blocks, never sleeps, never dies
// inputs:  none
// outputs: none
short IntTerm;     // accumulated error, RPM-sec
short PrevError;   // previous error, RPM
short Coeff[3];    // PID coefficients
short Actuator;
void PID(void){ 
  short err;  // speed error, range -100 to 100 RPM
  unsigned long myId = OS_Id(); 
//#ifdef DEBUG
//  PF4 = 0x10;
//#endif
  PIDWork = 0;
  IntTerm = 0;
  PrevError = 0;
  Coeff[0] = 384;   // 1.5 = 384/256 proportional coefficient
  Coeff[1] = 128;   // 0.5 = 128/256 integral coefficient
  Coeff[2] = 64;    // 0.25 = 64/256 derivative coefficient*
//#ifdef DEBUG
//  PF4 = 0x00;
//#endif
  while(NumSamples < RUNLENGTH) { 
    for(err = -1000; err <= 1000; err++){    // made-up data
      Actuator = PID_stm32(err,Coeff)/256;
    }
    PIDWork++;        // calculation finished
  }
  ST7735_Message(1,6,"sample done",0);  
  for(;;){ }          // done
}
//--------------end of Task 4-----------------------------

//------------------Task 5--------------------------------IO bound
// UART background ISR performs serial input/output
// Two software fifos are used to pass I/O data to foreground
// The interpreter runs as a foreground thread
// The UART driver should call OS_Wait(&RxDataAvailable) when foreground tries to receive
// The UART ISR should call OS_Signal(&RxDataAvailable) when it receives data from Rx
// Similarly, the transmit channel waits on a semaphore in the foreground
// and the UART ISR signals this semaphore (TxRoomLeft) when getting data from fifo
// Modify your intepreter from Lab 1, adding commands to help debug 
// Interpreter is a foreground thread, accepts input from serial port, outputs to serial port
// inputs:  none
// outputs: none
char bufPt[20];
int data;
//---------------------OutCRLF---------------------
// Output a CR,LF to UART to go to a new line
// Input: none
// Output: none
void OutCRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}

void Interpreter(void)    // just a prototype, link to your interpreter
{
  while(1)
  {
    UART_InString(bufPt, 19);
    OutCRLF();
    UART_OutString("received");
    OutCRLF();
    if (strncmp(bufPt+1, "adc", 3) == 0){
      ADC_Open(0);
      ST7735_Message(0,4,"ADC collect:", ADC_In());
      OutCRLF();
    }
    else if(strncmp(bufPt+1, "pid", 3) == 0){
      ST7735_Message(0,5,"hello pid", PIDWork);
      OutCRLF();
    }
    else{
      UART_OutString("wrong input");
      OutCRLF();
    }
  }
}
// add the following commands, leave other commands, if they make sense
// 1) print performance measures 
//    time-jitter, number of data points lost, number of calculations performed
//    i.e., NumSamples, NumCreated, MaxJitter, DataLost, FilterWork, PIDwork
      
// 2) print debugging parameters 
//    i.e., x[], y[] 
//--------------end of Task 5-----------------------------



//board support package
void BSP_init(void)
{
  PortF_Init();
  PortE_Init();
  PortB_Init();
  DataLost = 0;        // lost data between producer and consumer
  NumSamples = 0;
  MaxJitter = 0;       // in 1us units
  NumCreated = 0 ;
  ST7735_InitR(INITR_REDTAB);
  ADC_Open(4);
  UART_Init();

//  ST7735_Message(0,0, "test this ", 0);
//  ST7735_Message(1,1, "test this ", 1);
}

int finalmain(void)
{
  OS_Init();           // initialize, disable interrupts
  
  BSP_init();
//********initialize communication channels
  OS_MailBox_Init();
  OS_Fifo_Init(128);    // ***note*** 4 is not big enough*****
                        //not use this 128 as size input, use defined FIFO_SIZE in os.c
//*******attach background tasks***********
  OS_AddSW1Task(&SW1Push,2);
#if Lab3
  OS_AddSW2Task(&SW2Push,2);  // add this line in Lab 3
#endif
//  
  OS_AddPeriodicThread(&DAS,PERIOD,1); // 2 kHz real time sampling of PD3

// create initial foreground threads
  NumCreated += OS_AddThread(&Interpreter,128,2); 
  NumCreated += OS_AddThread(&Consumer,128,1); 
  NumCreated += OS_AddThread(&PID,128,3);  // Lab 3, make this lowest priority
 
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Fourth TEST**********
// Once the third test runs, run this example (Lab 1 part 2)
// Count1 should exactly equal Count2
// Count3 should be very large
// Count4 increases by 640 every time select is pressed
// NumCreated increase by 1 every time select is pressed

// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// Timer interrupts, with or without period established by OS_AddPeriodicThread
// Select switch interrupts, active low
// no ADC serial port or LCD output
// tests the spinlock semaphores, tests Sleep and Kill
Sema4Type Readyd;        // set in background
void BackgroundThread1d(void){   // called at 1000 Hz
static int i=0;
  i++;
  if(i==50){
    i = 0;         //every 50 ms
    Count1++;
    OS_bSignal(&Readyd);
  }
}
void Thread2d(void){
  OS_InitSemaphore(&Readyd,0);
  Count1 = 0;          
  Count2 = 0;          
  for(;;){
    OS_bWait(&Readyd);
    Count2++;     
  }
}
void Thread3d(void){
  Count3 = 0;          
  for(;;){
    Count3++;
  }
}
void Thread4d(void){ int i;
  for(i=0;i<640;i++){
    Count4++;
    OS_Sleep(1);
  }
  OS_Kill();
}
void BackgroundThread5d(void){   // called when Select button pushed
  NumCreated += OS_AddThread(&Thread4d,128,3); 
}
int Testmain4(void){   // Testmain4
  Count4 = 0;          
  OS_Init();           // initialize, disable interrupts
  NumCreated = 0 ;
  OS_AddPeriodicThread(&BackgroundThread1d,PERIOD,0); 
  OS_AddSW1Task(&BackgroundThread5d,2);
  NumCreated += OS_AddThread(&Thread2d,128,2); 
  NumCreated += OS_AddThread(&Thread3d,128,3); 
  NumCreated += OS_AddThread(&Thread4d,128,3); 
  OS_Launch(TIME_250US); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}






