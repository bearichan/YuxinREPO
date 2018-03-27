#include "OS.h"
#include <stdint.h>
#include "PLL.h"
#include "../inc/tm4c123gh6pm.h"
#include "ST7735.h"

#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_INT_CTRL_R         (*((volatile uint32_t *)0xE000ED04))
#define NVIC_INT_CTRL_PENDSTSET 0x04000000  // Set pending SysTick interrupt
#define NVIC_SYS_PRI3_R         (*((volatile uint32_t *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority


volatile uint32_t counter;
void (*PeriodicTask)(void);
void (*SW1task)(void);
void OS_DisableInterrupts(void); // Disable interrupts
void OS_EnableInterrupts(void);  // Enable interrupts
int32_t StartCritical(void);
void EndCritical(int32_t primask);
void StartOS(void);
#define Lab2 1
#define Lab3 0

#define NUMTHREADS  30        // maximum number of threads
#define STACKSIZE   100      // number of 32-bit words in stack
struct tcb{
  int32_t *sp;       // pointer to stack (valid for threads not running
  struct tcb *next;  // linked-list pointer
  struct tcb *before;
  int32_t id;
  int32_t sleepState;
  int32_t priority;
  int32_t blockedState;
};
int32_t blockUsed[NUMTHREADS];
typedef struct tcb tcbType;
tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
tcbType *threadHead, *threadTail;
int32_t Stacks[NUMTHREADS][STACKSIZE];
int32_t totalThread;
int32_t curThread;

// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: systick, 50 MHz PLL
// input:  none
// output: none
void OS_Init(void){
  OS_DisableInterrupts();
  PLL_Init(Bus80MHz);                 // set processor clock to 50 MHz
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0x00FFFFFF)|0xC0000000; // priority 6 SysTick
	NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0xFF00FFFF)|0x00E00000; // priority 7 PendSV
  threadHead = 0;
  curThread = 0;
  totalThread = 0;
  OS_ProfileTimer_Init();
  OS_MScounter_Init();
  for(int i = 0; i < NUMTHREADS; i++){
    blockUsed[i] = 0;   //not used
  }
  RunPt = threadHead;
}

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, long value){
  semaPt->Value = value;
}

// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt){
  OS_DisableInterrupts();
  while(semaPt->Value == 0){
    OS_EnableInterrupts();
    OS_DisableInterrupts();
  }
  semaPt->Value --;
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
  semaPt->Value ++;
  EndCritical(status);
}

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt){
  OS_DisableInterrupts();
  while(semaPt->Value == 0){
    OS_EnableInterrupts();
    OS_DisableInterrupts();
  }
  semaPt->Value = 0 ;
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
  semaPt->Value = 1;
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



void Timer4A_Handler(void){
  TIMER4_ICR_R = TIMER_ICR_TATOCINT;
  (*PeriodicTask)();
}

void Scheduler(void){
  
  RunPt = RunPt->next;   //round robin
//  if((RunPt -> sleepState) > 0){
//    
//  }
  while(RunPt -> sleepState){
    RunPt = RunPt->next;
  }
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
//====================coded by Yuxin Wang=====================
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
	//consider priority and stacksize in lab3
	
  #if Lab3
	
  #endif

  EndCritical(status);
  return 1;	 
}


unsigned long OS_Id(void){
  return RunPt->id;
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
  
  SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
  while((SYSCTL_PRGPIO_R&0x20)==0){};   //wait for done
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
  OS_EnableInterrupts();           // (i) Clears the I bit
  return 1;
}
//extern unsigned long PIDWork;      // current number of PID calculations finished
void GPIOPortF_Handler(void){
  GPIO_PORTF_ICR_R = 0x10;      // acknowledge flag4
//  ST7735_Message(1,2,"PIDWork     =",PIDWork);
  SW1task();
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
int OS_AddSW2Task(void(*task)(void), unsigned long priority){
  return 1;
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
  }
  blockUsed[RunPt->id] = 0;
  
  EndCritical(status);
  OS_Suspend();
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
#define FIFO_SIZE 2048
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
  if(FIFOused == FIFO_SIZE) return 0;
  
//  OS_bWait(&FIFOmutex);
  int32_t sr = StartCritical();
  OS_FIFO[FIFOtail] = data;
  FIFOused ++;
  FIFOtail = (FIFOtail + 1)%FIFO_SIZE;
  EndCritical(sr);
//  OS_bSignal(&FIFOmutex);
//  OS_Signal(&FIFOempty);
 
  return 1;
}

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
unsigned long OS_Fifo_Get(void){
  unsigned long data;
//  OS_Wait(&FIFOempty);
//  OS_bWait(&FIFOmutex);
  int32_t sr = StartCritical();
  data = OS_FIFO[FIFOhead];
  FIFOused --;
  FIFOhead = (FIFOhead + 1)%FIFO_SIZE;
  EndCritical(sr);
//  OS_bSignal(&FIFOmutex);
  
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
  long sr;
  sr = StartCritical(); 
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
  EndCritical(sr);
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
  
  if(totalThread)
  {
		tcbType *firstPt = RunPt;
		tcbType *currentPt = RunPt;
		for(int i=0; i<NUMTHREADS; i++)
    {
			if(currentPt->sleepState)
      {
				currentPt->sleepState--;
			}
			currentPt = currentPt->next;
			if(currentPt == firstPt)
      {
				break;
			}
		}
	}
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
  unsigned long time_diff = stop - start;
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





