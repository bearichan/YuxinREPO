#include "OS.h"
#include <stdint.h>
#include <stdio.h>
#include "PLL.h"
#include "../inc/tm4c123gh6pm.h"
#include "ST7735.h"
#include "uart.h"
#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_INT_CTRL_R         (*((volatile uint32_t *)0xE000ED04))
#define NVIC_INT_CTRL_PENDSTSET 0x04000000  // Set pending SysTick interrupt
#define NVIC_SYS_PRI3_R         (*((volatile uint32_t *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority

#if Lab3
unsigned long MaxJitter;    
unsigned long MaxJitter2;   
#define JITTERSIZE 64
unsigned long const JitterSize=JITTERSIZE;
unsigned long JitterHistogram[JITTERSIZE]={0,};
unsigned long JitterHistogram2[JITTERSIZE]={0,};
#endif

void DisableInterrupts(void);
void EnableInterrupts(void);

volatile uint32_t counter;
void (*PeriodicTask)(void);
void (*PeriodicTask2)(void);
unsigned int PeriodicTaskPeriod;
unsigned int PeriodicTask2Period;
void (*SW1task)(void);
void (*SW2task)(void);
void OS_DisableInterrupts(void); // Disable interrupts
void OS_EnableInterrupts(void);  // Enable interrupts
int32_t StartCritical(void);
void EndCritical(int32_t primask);
void StartOS(void);


#define NUMTHREADS  30        // maximum number of threads
#define STACKSIZE   100      // number of 32-bit words in stack

int32_t blockUsed[NUMTHREADS];
tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
tcbType *threadHead, *threadTail;
int32_t Stacks[NUMTHREADS][STACKSIZE];
int32_t totalThread;
int32_t curThread;
tcbType *sleepHead, *sleepTail;


// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: systick, 50 MHz PLL
// input:  none
// output: none
void OS_Init(void){
  DisableInterrupts();
  PLL_Init(Bus80MHz);                 // set processor clock to 50 MHz
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0x00FFFFFF)|0xC0000000; // priority 6 SysTick
  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0xFF00FFFF)|0x00E00000; // priority 7 PendSV
  threadHead = 0;
  threadTail = 0;
  sleepHead = 0;
  sleepTail = 0;
  curThread = 0;
  totalThread = 0;
  OS_ProfileTimer_Init();
  OS_MScounter_Init();
  for(int i = 0; i < NUMTHREADS; i++){
    blockUsed[i] = 0;   //not used
  }
  RunPt = threadHead;
  EnableInterrupts();           // (i) Clears the I bit
}

//**************list operations*********************


// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, long value){
  long st = StartCritical();
  semaPt->Value = value;
  semaPt->blockHead = 0;
  semaPt->blockTail = 0;
  EndCritical(st);
}

void removeFromRunList(tcbType *pt){
  long status = StartCritical();
  totalThread--;
  if(totalThread > 0)
  {
    pt->next->before = pt->before;
    pt->before->next = pt->next;
    if(pt == threadTail){
      threadTail = pt->before;
    }
    if(pt == threadHead){
      threadHead = pt->next;
    }
  }  
//  else{
//    pt->next = 0;	//clear next for Sleeping and Blocking lists
//    pt->before = 0;	//clear prev for Sleeping and Blocking lists
//  }
  //OS_Suspend();
  EndCritical(status);
  
}

void addBlockList(Sema4Type *semaPt, tcbType *pt){
  if(semaPt->blockHead){
    semaPt->blockTail->next = pt;
    pt->before = semaPt->blockTail;
    semaPt->blockTail = pt;
  }
  else{
    pt->next = pt;
    pt->before = pt;
    semaPt->blockHead = pt;
    semaPt->blockTail = pt;
  }
}

void BlockThread(Sema4Type *semaPt, tcbType *pt){
  removeFromRunList(pt);
  addBlockList(semaPt, pt);
}

tcbType *popBlockList(Sema4Type *semaPt){
  tcbType *pt;
  pt = semaPt->blockHead;
  if(semaPt->blockHead == semaPt->blockTail) {
    semaPt->blockHead = 0;
    semaPt->blockTail = 0;
  }
  else{
    semaPt->blockHead = pt->next;
  }
  return pt;
}

void addRunList(tcbType *pt){
  if(RunPt == 0){
    RunPt = pt;
    RunPt->next = RunPt;
    RunPt->before = RunPt;
    threadHead = RunPt;
    threadTail = RunPt;
  }
  else{
    tcbType *threadPt = threadHead;
    //insert list with priority
    if((pt->priority < threadHead-> priority) || (pt->priority >= threadTail->priority)){
      pt->next = threadHead;
      pt->before = threadTail;
      threadHead->before = pt; 
      threadTail->next = pt;
      if(pt->priority < threadHead-> priority) threadHead = pt;
      else if(pt->priority >= threadTail->priority) threadTail = pt;
    }
    else{
      while(pt->priority >= threadPt->priority){
        threadPt = threadPt->next;
      }
      threadPt->before->next = pt;
      tcbType *threadptb;
      threadptb = threadPt->before;
      threadPt->before = pt;
      pt->before = threadptb;
      pt->next = threadPt;
    }
  }
  totalThread++;
}
void UnblockThread(Sema4Type *semaPt){
  addRunList(popBlockList(semaPt));
}
// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt){
  OS_DisableInterrupts();
#if Lab2
  while(semaPt->Value == 0){
    OS_EnableInterrupts();
    OS_DisableInterrupts();
  }
  semaPt->Value --;
#endif
#if Lab3
  semaPt->Value --;
  if(semaPt->Value < 0){
    //RunPt->blockedState = semaPt;
    BlockThread(semaPt, RunPt);
    OS_EnableInterrupts();
    OS_Suspend();
//    OS_DisableInterrupts();
  }
#endif
  OS_EnableInterrupts();
}

// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt){
  long status;
  status = StartCritical();
#if Lab2
  semaPt->Value ++;
#endif
  
#if Lab3
  tcbType *pt;
  semaPt->Value ++;
  if(semaPt->Value <= 0){
//    pt = threadHead;
//    while(pt->blockedState != semaPt){
//      pt = pt->next;
//      if(pt == threadHead) break;
//    }
//    pt->blockedState = 0;
    UnblockThread(semaPt);
  }
#endif
  EndCritical(status);
}


// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt){
  OS_DisableInterrupts();
#if Lab2
  while(semaPt->Value == 0){
    OS_EnableInterrupts();
    OS_DisableInterrupts();
  }
  semaPt->Value --;
#endif
#if Lab3
  semaPt->Value --;
  if(semaPt->Value < 0){
    //RunPt->blockedState = semaPt;
    BlockThread(semaPt, RunPt);
    OS_EnableInterrupts();
    OS_Suspend();
//    OS_DisableInterrupts();
  }
#endif
  OS_EnableInterrupts();
}

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt){
 long status;
  status = StartCritical();
#if Lab2
  semaPt->Value ++;
#endif
  
#if Lab3
  tcbType *pt;
  semaPt->Value ++;
  if(semaPt->Value <= 0){
//    pt = threadHead;
//    while(pt->blockedState != semaPt){
//      pt = pt->next;
//      if(pt == threadHead) break;
//    }
//    pt->blockedState = 0;
    UnblockThread(semaPt);
  }
#endif
  EndCritical(status);
}


void SetInitialStack(int i){
  tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer
  Stacks[i][STACKSIZE-1] = 0x01000000;   // thumb bit
  Stacks[i][STACKSIZE-3] = 0x14141414;   // R14
  Stacks[i][STACKSIZE-4] = 0x12121212;   // R12
  Stacks[i][STACKSIZE-5] = 0x03030303;   // R3
  Stacks[i][STACKSIZE-6] = 0x02020202;   // R2
  Stacks[i][STACKSIZE-7] = 0x01010101;   // R1
  Stacks[i][STACKSIZE-8] = 0x00000000;   // R0
  Stacks[i][STACKSIZE-9] = 0x11111111;   // R11
  Stacks[i][STACKSIZE-10] = 0x10101010;  // R10
  Stacks[i][STACKSIZE-11] = 0x09090909;  // R9
  Stacks[i][STACKSIZE-12] = 0x08080808;  // R8
  Stacks[i][STACKSIZE-13] = 0x07070707;  // R7
  Stacks[i][STACKSIZE-14] = 0x06060606;  // R6
  Stacks[i][STACKSIZE-15] = 0x05050505;  // R5
  Stacks[i][STACKSIZE-16] = 0x04040404;  // R4
}



///******** OS_Launch ***************
// start the scheduler, enable interrupts
// Inputs: number of 20ns clock cycles for each time slice
//         (maximum of 24 bits)
// Outputs: none (does not return)
void OS_Launch(unsigned long theTimeSlice){
  NVIC_ST_RELOAD_R = theTimeSlice - 1; // reload value
  NVIC_ST_CTRL_R = 0x00000007; // enable, core clock and interrupt arm
  StartOS();                   // start on the first task
}
int numPeriodicThread = 0;
int OS_AddPeriodicThread(void(*task)(void),uint32_t period, uint32_t priority){
  int32_t status = StartCritical();
  if(numPeriodicThread == 2)
  {
    EndCritical(status);
    return 0;
  }
  if(numPeriodicThread == 0)
  {
    PeriodicTask = task;          // user function
    PeriodicTaskPeriod = period;
    SYSCTL_RCGCWTIMER_R |= 0x01;   //  activate WTIMER0
    WTIMER0_CTL_R = (WTIMER0_CTL_R&~0x0000001F);    // disable Wtimer0A during setup
    WTIMER0_CFG_R = 0x00000004;    // configure for 32-bit timer mode
    WTIMER0_TAMR_R = 0x00000002;   // configure for periodic mode, default down-count settings
    WTIMER0_TAPR_R = 0;            // prescale value for trigger
    WTIMER0_ICR_R = 0x00000001;    // 6) clear WTIMER0A timeout flag
    NVIC_EN2_R = 0x40000000;              // enable interrupt 94 in NVIC
    NVIC_PRI23_R = (NVIC_PRI23_R&0xFF00FFFF)| (priority << 21); //set priority 
    WTIMER0_IMR_R = (WTIMER0_IMR_R&~0x0000001F)|0x00000001;    // enable timeout interrupts
    WTIMER0_TAILR_R = (PeriodicTaskPeriod)-1;
    WTIMER0_CTL_R |= 0x00000001;   // enable Wtimer0A 32-b, periodic
  }
  else if(numPeriodicThread == 1)
  {
    PeriodicTask2 = task;
    PeriodicTask2Period = period;
    SYSCTL_RCGCWTIMER_R |= 0x01;   //  activate WTIMER0
    WTIMER0_CTL_R = (WTIMER0_CTL_R&~0x00001F00);    // disable Wtimer0B during setup
//    WTIMER0_CFG_R = 0x00000004;    // configure for 32-bit timer mode
//    WTIMER0_TBMR_R = 0x00000002;   // configure for periodic mode, default down-count settings
//    WTIMER0_TBPR_R = 0;            // prescale value for trigger
//    WTIMER0_ICR_R = 0x00000100;    // 6) clear WTIMER0B timeout flag
//    NVIC_EN2_R = 0x80000000;              // enable interrupt 95 in NVIC
    NVIC_PRI23_R = (NVIC_PRI23_R&0x00FFFFFF)| (priority << 29); //set priority 
    WTIMER0_IMR_R = (WTIMER0_IMR_R&~0x00001F00)|0x00000100;    // enable timeout interrupts
    WTIMER0_TBILR_R = (PeriodicTask2Period)-1;    // start value for trigger
    WTIMER0_CTL_R |= 0x00000100;   // enable wtimer0B
  }
  numPeriodicThread++;
  EndCritical(status);
  return 0;
}

void OS_ClearTimePeriod(void){
  counter = 0;
}

uint32_t OS_ReadPeriodTime(void){
  return counter;
}

unsigned long PeriodicTaskCount,PeriodicTask2Count;
void WideTimer0A_Handler(void){
  WTIMER0_ICR_R |= TIMER_ICR_TATOCINT;
  PeriodicTask();
  unsigned static long LastTime;
  unsigned long jitter;
  unsigned long thisTime = OS_Time();
 
  if(PeriodicTaskCount)
  {
    unsigned long diff = OS_TimeDifference(LastTime,thisTime);
    if(diff>PeriodicTaskPeriod)
    {
      jitter = (diff-PeriodicTaskPeriod+4)/8;  // in 0.1 usec
    }
    else
    {
      jitter = (PeriodicTaskPeriod-diff+4)/8;  // in 0.1 usec
    }
    if(jitter > MaxJitter)
    {
      MaxJitter = jitter; // in usec
    }       // jitter should be 0
    if(jitter >= JITTERSIZE)
    {
      jitter = JITTERSIZE-1;
    }
      JitterHistogram[jitter]++;
  }
  LastTime = thisTime;
  PeriodicTaskCount++;
}

void WideTimer0B_Handler(void){
  WTIMER0_ICR_R |= TIMER_ICR_TBTOCINT;
  (*PeriodicTask2)();
  
  unsigned static long LastTime;
  unsigned long jitter;
  unsigned long thisTime = OS_Time();
  if(PeriodicTask2Count)
  {
    unsigned long diff = OS_TimeDifference(LastTime,thisTime);
    if(diff>PeriodicTask2Period)
    {
      jitter = (diff-PeriodicTask2Period+4)/8;  // in 0.1 usec
    }
    else
    {
      jitter = (PeriodicTask2Period-diff+4)/8;  // in 0.1 usec
    }
    if(jitter > MaxJitter2)
    {
      MaxJitter2 = jitter; // in usec
    }       // jitter should be 0
    if(jitter >= JITTERSIZE)
    {
      jitter = JITTERSIZE-1;
    }
      JitterHistogram2[jitter]++;
  }
  LastTime = thisTime;
  PeriodicTask2Count++;
}
void Jitter(void){
  ST7735_Message(0,3,"Jitter 0.1us=",MaxJitter);
  ST7735_Message(0,4,"Jitter 0.1us=",MaxJitter2);
}

tcbType *prev_pt;
void Scheduler(void){
#if Lab2
  RunPt = RunPt->next;   //round robin
  while(RunPt -> sleepState){
    RunPt = RunPt->next;
  }
#endif

#if Lab3
  while(RunPt == 0){
    OS_EnableInterrupts();
    OS_DisableInterrupts();
  }
  RunPt = threadHead;
//  RunPt = RunPt->next;   //round robin
  //while(RunPt -> sleepState){
   // RunPt = RunPt->next;
    if(RunPt == prev_pt)
    {
      RunPt = RunPt->next;
    }
    //OS_EnableInterrupts();
    //OS_DisableInterrupts();
  //}
  //guatantee same task not picked again
  //(same task continuously running) just happens when there is only one task left
  prev_pt = RunPt;
#endif
}
//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word dary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
//====================coded by Yuxin Wang====================
int OS_AddThread(void(*task)(void), unsigned long stackSize, unsigned long priority)
{
  int32_t status = StartCritical();
#if Lab2
  curThread = NUMTHREADS;
  for (int i = 0; i<NUMTHREADS; i++){
    if(blockUsed[i] == 0){
      blockUsed[i] = 1;
      curThread = i;
      break;
    }
  }
  if(curThread == NUMTHREADS) {
    EndCritical(status);
    return 0;
  }
  if(totalThread == 0){
    tcbs[curThread].next = &tcbs[curThread];
    tcbs[curThread].before = &tcbs[curThread];
    threadHead = &tcbs[curThread];
    threadTail = &tcbs[curThread];
    RunPt = threadHead;
  }
  else{
    tcbs[curThread].next = threadHead;
    tcbs[curThread].before = threadTail;
    threadHead->before = &tcbs[curThread]; 
    threadTail->next = &tcbs[curThread];
    threadTail = &tcbs[curThread];
  }
  tcbs[curThread].priority = priority;
  tcbs[curThread].sleepState = 0;
  tcbs[curThread].id = curThread;
  SetInitialStack(curThread); 
  Stacks[curThread][STACKSIZE-2] = (int32_t)(task); // PC
  totalThread ++;
  
#endif

  #if Lab3
  curThread = NUMTHREADS;
  tcbType *pt;
  for (int i = 0; i<NUMTHREADS; i++){
    if(blockUsed[i] == 0){
      blockUsed[i] = 1;
      curThread = i;
      break;
    }
  }
  if(curThread == NUMTHREADS) {
    EndCritical(status);
    return 0;
  }
  if(totalThread == 0){
    tcbs[curThread].next = &tcbs[curThread];
    tcbs[curThread].before = &tcbs[curThread];
    threadHead = &tcbs[curThread];
    threadTail = &tcbs[curThread];
    RunPt = threadHead;
  }
  else{
    pt = threadHead;
    //insert list with priority
    if(priority < (threadHead-> priority) ){
      tcbs[curThread].next = threadHead;
      tcbs[curThread].before = threadTail;
      threadHead->before = &tcbs[curThread]; 
      threadTail->next = &tcbs[curThread];
      threadHead = &tcbs[curThread];
    }
    else if(priority >= (threadTail->priority))
    {
      tcbs[curThread].next = threadHead;
      tcbs[curThread].before = threadTail;
      threadHead->before = &tcbs[curThread]; 
      threadTail->next = &tcbs[curThread];
      threadTail = &tcbs[curThread];
    }
    else{
      while(priority >= pt->priority){
        pt = pt->next;
      }
      pt->before->next = &tcbs[curThread];
      tcbType *ptb;
      ptb = pt->before;
      pt->before = &tcbs[curThread];
      tcbs[curThread].before = ptb;
      tcbs[curThread].next = pt;
    }
  }
  tcbs[curThread].priority = priority;
  tcbs[curThread].sleepState = 0;
  tcbs[curThread].id = curThread;
//  tcbs[curThread].blockedState = 0;
  SetInitialStack(curThread); 
  Stacks[curThread][STACKSIZE-2] = (int32_t)(task); // PC
  totalThread ++;
  #endif

  EndCritical(status);
  return 1;	 
}


unsigned long OS_Id(void){
  return RunPt->id;
}


void PortF_KEY_Init(void){
  SYSCTL_RCGCGPIO_R |= 0x20; // activate port F
  while((SYSCTL_PRGPIO_R&0x20)==0){}; // allow time for clock to start 
  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
  GPIO_PORTF_CR_R |= 0x11;
  GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x11;  //     disable alt funct on PF0, PF4
  GPIO_PORTF_DEN_R |= 0x11;     //     enable digital I/O on PF4
  GPIO_PORTF_PCTL_R &= ~0x000F000F; //  configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x11;  //    disable analog functionality on PF4
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flag4
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
  
}


//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
volatile uint32_t FallingEdges = 0;
uint32_t SW1_pri;
int OS_AddSW1Task(void(*task)(void), unsigned long priority){
  SW1task = task;
  SW1_pri = priority;
  GPIO_PORTF_IM_R |= 0x10;     
    
  return 1;
}

//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function can be ignored
// In lab 3, this command will be called will be called 0 or 1 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
uint32_t SW2_pri;
int OS_AddSW2Task(void(*task)(void), unsigned long priority){
  SW2task = task;
  SW2_pri = priority;
  GPIO_PORTF_IM_R |= 0x01;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***

  return 1;

}

//extern unsigned long PIDWork;      // current number of PID calculations finished
void GPIOPortF_Handler(void){
  if(GPIO_PORTF_MIS_R&0x10){
    GPIO_PORTF_ICR_R = 0x10;
    SW1task();
  }
	if(GPIO_PORTF_MIS_R&0x01){
    GPIO_PORTF_ICR_R = 0x01;
    SW2task();
  }
}

void removeFromSleepList(tcbType *pt){
  long status = StartCritical();
  if(sleepHead == sleepTail)
  {
    sleepHead = 0;
    sleepTail = 0;
  }
  else {
    pt->next->before = pt->before;
    pt->before->next = pt->next;
    if(pt == sleepTail){
      sleepTail = pt->before;
    }
    if(pt == sleepHead){
      sleepHead = pt->next;
    }
  }  

  EndCritical(status);
}

void wakeupSleepList(tcbType *pt){
  removeFromSleepList(pt);
  addRunList(pt);
}

void addSleepList(tcbType *pt){
  if(sleepHead){
    sleepTail->next = pt;
    pt->before = sleepTail;
    sleepTail = pt;
  }
  else{
    pt->next = pt;
    pt->before = pt;
    sleepHead = pt;
    sleepTail = pt;
  }
}

void SleepList(tcbType *pt){
  removeFromRunList(pt);
  addSleepList(pt);
}
// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(unsigned long sleepTime){
  int32_t status = StartCritical();
  RunPt->sleepState = sleepTime;
  SleepList(RunPt);
  EndCritical(status);
  OS_Suspend();
}

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void){
  int32_t status = StartCritical();
  
  totalThread --;
  if(totalThread > 0)
  {
    RunPt->next->before = RunPt->before;
    RunPt->before->next = RunPt->next;
    if(RunPt == threadTail){
      threadTail = RunPt->before;
    }
    if(RunPt == threadHead){
      threadHead = RunPt->next;
    }
  }
//  else{
//    RunPt = 0;
//    RunPt->next = 0;
//    RunPt->before = 0;
//  }
  blockUsed[RunPt->id] = 0;
  OS_Suspend();
  EndCritical(status);
  
}

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
#define PF1       (*((volatile uint32_t *)0x40025008))
void OS_Suspend(void){
//  PF1 ^= 0x02;       // heartbeat
//  NVIC_ST_CURRENT_R = 0;
//  NVIC_INT_CTRL_R = 0x04000000; //Trigger SysTick
  NVIC_INT_CTRL_R |= 0x10000000; //PENDSV set to 1->do context switch
}
 
// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
#define FIFO_SIZE 128
uint32_t OS_FIFO[FIFO_SIZE];
int FIFOused;
uint32_t FIFOtail;
uint32_t FIFOhead;
Sema4Type FIFOfull, FIFOempty, FIFOmutex;
void OS_Fifo_Init(unsigned long size){
  int32_t sr = StartCritical();
  FIFOused = 0;
  FIFOtail = 0;
  FIFOhead = 0;
  OS_InitSemaphore(&FIFOfull,FIFO_SIZE);
  OS_InitSemaphore(&FIFOmutex,1);
  OS_InitSemaphore(&FIFOempty,0);
  EndCritical(sr);
}

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(unsigned long data){
  if(FIFOused == FIFO_SIZE) 
  {
    return 0;
  }
//  OS_Wait(&FIFOmutex);
  int32_t sr = StartCritical();
  OS_FIFO[FIFOtail] = data;
  FIFOused ++;
  FIFOtail = (FIFOtail + 1)%FIFO_SIZE;
  EndCritical(sr);
//  OS_Signal(&FIFOmutex);
  OS_Signal(&FIFOempty);
 
  return 1;
}

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
unsigned long OS_Fifo_Get(void){
  unsigned long data;
  OS_Wait(&FIFOempty);
//  OS_Wait(&FIFOmutex);
  int32_t sr = StartCritical();
  data = OS_FIFO[FIFOhead];
  FIFOused --;
  FIFOhead = (FIFOhead + 1)%FIFO_SIZE;
  EndCritical(sr);
//  OS_Signal(&FIFOmutex);
  
  return data;
}

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
long OS_Fifo_Size(void){
  return FIFOused;
}

// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
unsigned long mail;
Sema4Type boxFree, boxValid;
void OS_MailBox_Init(void){
  mail = 0;
  OS_InitSemaphore(&boxFree,1);
  OS_InitSemaphore(&boxValid,0);  
}

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(unsigned long data){
  OS_bWait(&boxFree);
  mail = data;
  OS_bSignal(&boxValid);
}

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
unsigned long OS_MailBox_Recv(void){
  unsigned long rec_data;
  OS_bWait(&boxValid);
  rec_data = mail;
  OS_bSignal(&boxFree);
  return rec_data;
}

void OS_ProfileTimer_Init(void){
  DisableInterrupts();
  SYSCTL_RCGCTIMER_R |= 0x01;   // 0) activate TIMER0
//  PeriodicTask = task;          // user function
  TIMER0_CTL_R = 0x00000000;    // 1) disable TIMER0A during setup
  TIMER0_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER0_TAMR_R = 0x00000092;   // 3) configure for periodic mode, default up-count settings,snapshot
  TIMER0_TAILR_R = 0xFFFFFFFF;    // 4) reload value
  TIMER0_TAPR_R = 0;            // 5) bus clock resolution
  TIMER0_ICR_R = 0x00000001;    // 6) clear TIMER0A timeout flag
  TIMER0_IMR_R = 0x00000000;    // 7) disarm timeout interrupt
//  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x80000000; // 8) priority 4
// interrupts enabled in the main program after all devices initialized
// vector number 35, interrupt number 19
//  NVIC_EN0_R = 1<<19;           // 9) enable IRQ 19 in NVIC
  TIMER0_CTL_R = 0x00000001;    // 10) enable TIMER0A
  EnableInterrupts();
}
unsigned long MsTime = 0;
void OS_MScounter_Init(void){ //Sleep
  SYSCTL_RCGCTIMER_R |= 0x20;   //  activate TIMER5
  MsTime = 0;
  TIMER5_CTL_R = 0x00000000;    // disable timer5A during setup
  TIMER5_CFG_R = 0x00000000;             // configure for 32-bit timer mode
  TIMER5_TAMR_R = 0x00000002;   // configure for periodic mode, default down-count settings
  TIMER5_TAPR_R = 0;            // prescale value for trigger
  TIMER5_ICR_R = 0x00000001;    // 6) clear TIMER4A timeout flag
  TIMER5_TAILR_R = (1*(50000000/1000))-1;    // start value for trigger
  NVIC_PRI23_R = (NVIC_PRI23_R&0xFFFFFF00)|0x00000060; // 8) priority 3
  NVIC_EN2_R = 0x10000000;        // 9) enable interrupt 19 in NVIC
  TIMER5_IMR_R = 0x00000001;    // enable timeout interrupts
  TIMER5_CTL_R |= 0x00000001;   // enable timer5A 32-b, periodic, no interrupts
}

void Timer5A_Handler(){
  //PF1 ^= 0x02;
  //PF1 ^= 0x02;
  TIMER5_ICR_R |= 0x01;
  MsTime += 1;
  
  long st = StartCritical();
  tcbType *spt = sleepHead;
  tcbType *nspt = sleepHead;
  if(sleepHead){
    if(sleepHead == sleepTail){
      spt->sleepState --;
      if(spt->sleepState == 0) wakeupSleepList(spt);
    }
    else{
      while(spt != sleepTail){
        spt->sleepState --;
        nspt = spt->next;
        if(spt->sleepState == 0) wakeupSleepList(spt);
        spt = nspt;
      }
      spt->sleepState --;
      if(spt->sleepState == 0) wakeupSleepList(spt);
    }
  }
  EndCritical(st);
//  if(totalThread)
//  {
//    tcbType *firstPt = RunPt;
//    tcbType *currentPt = RunPt;
//    for(int i=0; i<NUMTHREADS; i++)
//    {
//      if(currentPt->sleepState)
//      {
//        currentPt->sleepState--;
//      }
//      currentPt = currentPt->next;
//      if(currentPt == firstPt)
//      {
//        break;
//      }
//    }
//  }
  //PF1 ^= 0x02;
}

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295 2^32
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
unsigned long OS_Time(void){
  return TIMER0_TAV_R;
}

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less thaen or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
unsigned long OS_TimeDifference(unsigned long start, unsigned long stop){
  long time_diff = stop - start;
  if(time_diff < 0){
    time_diff += 0xFFFFFFFF;
  }
  return time_diff;
}

// ******** OS_ClearMsTime ************
// sets the system time to zero (from Lab 1)
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void){
  MsTime = 0;
}

// ******** OS_MsTime ************
// reads the current time in msec (from Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// It is ok to make the resolution to match the first call to OS_AddPeriodicThread

unsigned long OS_MsTime(void){
  return MsTime;
}

unsigned long nointTime;
unsigned long totalnointTime;
unsigned long maxnointTime = 0;

void OS_disintTime(void){
  nointTime = OS_Time();
}
unsigned long int_time_diff;
void OS_enintTime(void){
  int_time_diff = OS_TimeDifference(nointTime, OS_Time());
  if(int_time_diff > maxnointTime){
    maxnointTime = int_time_diff;
  }
  totalnointTime += int_time_diff;
}


#define totalRecordNum	100
unsigned long timestamp[totalRecordNum];
unsigned long curId[totalRecordNum];
unsigned long curRecordingThread = 0;


void OS_Recorder(void)
{
  if(curRecordingThread < totalRecordNum){
    timestamp[curRecordingThread] = OS_Time();
    curId[curRecordingThread] = OS_Id();
    curRecordingThread++;
  }
}
void OS_ResetRecorder(void)
{
  curRecordingThread = 0;
}
void OS_ClearRecorder(void)
{
  for(int i=0; i<totalRecordNum; i++)
  {
    timestamp[i] = 0;
    curId[i] = 0;
  }
}
void OS_DumpRecorder(void)
{
  for(int i=0; i<totalRecordNum; i++)
  {
    UART_OutUDec(timestamp[i]);
    UART_OutChar(' ');
    UART_OutUDec(curId[i]);
    UART_OutChar(CR);
    UART_OutChar(LF);
  }
}

