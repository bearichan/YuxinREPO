/**
 * This file includes core OS functionality. Included are routines for thread scheduling, sleeping, and killing. This
 * file also includes a number of utility functions. Several functions are actually implemented in osasm.s, but
 * we will explain their functionality in comments in this file.
 */

// #includes
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "OS.h"
#include <stddef.h>
#include <string.h>
#include "PLL.h"

// #defines
#define STACKSIZE 512
#define NUMTHREADS 8
#define MAXPRIORITY 8
#define INC_SYSTICK
#define PF4                     (*((volatile uint32_t *)0x40025040))
#define PF3     								(*((volatile uint32_t *)0x40025020))
#define PF0											(*((volatile uint32_t *)0x40025004))
#define SW1       0x10                      // on the left side of the Launchpad board
#define SW2       0x01                      // on the right side of the Launchpad board
#define SWITCHES                (*((volatile uint32_t *)0x40025044))
#define DEBUG

int DataLost = 0;

/**
 * Globally disables interrupts
 * @note this code is directly from ValvanoWare
 */
void OS_DisableInterrupts(void);

/**
 * Globally enables interrupts
 * @note this code is directly from ValvanoWare
 */
void OS_EnableInterrupts(void);

/**
 * Starts a critical section by saving the current primask and disabling interrupts. Should be used in tandem
 * with EndCritical
 * @return primask before disabling interrupts
 * @note this code is directly from ValvanoWare
 */
long StartCritical(void);

/**
 * Ends a critical section by restoring the previous primask. Should be used in tandem with StartCritical
 * @param primask primask to restore
 * @note this code is directly from ValvanoWare
 */
void EndCritical(long primask);


/**
 * Simulates an initial context switch to kick off the OS's first thread. Initially, all TCB stacks are
 * loaded with values that make it appear they have been interrupted. This function pops those values off the
 * stack for the first TCB in the threadDLL and sets the stack pointer, which simulates the actual switch. After this
 * function completes, the OS is now running (scheduling, interrupting, etc).
 * @note code directly from ValvanoWare and lecture slides; slightly modified
 */
void StartOS(void);

/**
 * PendSV_Handler is where the OS context switch takes place. By default, this function will first call the OS_Schedule
 * function to determine what TCB to switch to. Then, it pushes registers R4-R11 on to the current stack and stores
 * the current stack pointer in the current TCB. The pointer to the next TCB is then loaded in and the stack pointer is
 * updated.
 * @note code directly from ValvanoWare and lecture slides; slightly modified
 */
void PendSV_Handler(void);

/**
 * CheckMode is a function that returns the IPSR
 * Is used to check what kind of state we are executing
 * Only used in OS_AddThread
 */
unsigned long CheckMode(void);

// function declarations
void (*task_a)(void);
void (*task_b)(void);
void WideTimer3_Init(void);

// global declarations
unsigned long systemTime = 0;
unsigned long overflow = 0;
uint32_t periodictimer = 0;
uint8_t running = 0;

// used for context switch
TCB *curThread = NULL;
TCB *nextThread = NULL;
uint8_t curTID = 0;

// pool of pcbs, tcbs, and stacks
TCB tcbs[NUMTHREADS] = {0};
int32_t stacks[NUMTHREADS][STACKSIZE] = {0};

// thread queues
DLL priorityDLL[MAXPRIORITY] = {0};
DLL sleepDLL = {0};

/////////////////////////////////////////////////////////////////////////////
/// other OS functions
/////////////////////////////////////////////////////////////////////////////

/**
 * Initializes Port F for debugging purposes
 */
void PortF_Init(void) {
	volatile int delay;
	SYSCTL_RCGCGPIO_R |= 0x20;  // activate port F
	delay = 123;
  GPIO_PORTF_DIR_R |= 0x0E;   // make PF2 output (PF2 built-in LED)
  GPIO_PORTF_AFSEL_R &= ~0x0E;// disable alt funct on PF2
  GPIO_PORTF_DEN_R |= 0x0E;   // enable digital I/O on PF2
                              // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;     // disable analog functionality on PF
}

/////////////////////////////////////////////////////////////////////////////
/// end other OS functions
/////////////////////////////////////////////////////////////////////////////

/**
 * Pick the next TCB to run. For round-robin scheduling, the TCB to run next will always be
 * stored in the current TCB's struct. Thus, scheduling is as simple as getting the next pointer.
 * However, cases where the OS sleeps can cause some errors here due to the TCB's pointers being switched
 * around (remember, round-robin is dependent on these pointers being correct) when it is taken from the active thread queue.
 * So, OS_Sleep schedules the next thread before doing  queue rearrangement.
 * We don't want to schedule again if this situation happens, hence we only update next thread if scheduled is not set.
 */
void OS_Scheduler(void) {
	int i;
	long primask = StartCritical();

	for(i = 0; i < MAXPRIORITY; i++) {
		if(priorityDLL[i].size != 0) {
			if(priorityDLL[i].size > 1 && priorityDLL[i].nextThread == curThread) {
				priorityDLL[i].nextThread = curThread->nextTCB;
			}
			nextThread = priorityDLL[i].nextThread; 
			priorityDLL[i].nextThread = nextThread->nextTCB; 
			break; 
		}
	}
	
	EndCritical(primask);
}

/**
 * Removes a TCB from the active thread queue and places it in another queue. Used primarily for sleeping/blocking
 * threads. Because round robin is dependent on the next pointers stored in a TCB, we schedule here before removing
 * from the main thread queue. Now, even if a SysTick occurs before a context switch is forced, the next thread
 * pointer will be pointing to the correct place.
 * @param toDLL
 */
void removeFromActive(DLL * toDLL) {
	long primask = StartCritical();
	
	DLL_Remove(&priorityDLL[curThread->priority], curThread);
	DLL_Add(toDLL, curThread);

	EndCritical(primask);
}

/**
 * Initializes a TCB's stack to make it appear as if it has been interrupted. Important so that context switch works
 * correctly the first time a thread runs.
 * @param tcb TCB that needs stack initialized
 * @note directly from ValvanoWare.
 */
void stackInit(TCB * tcb) {
	uint8_t pid = tcb->pid;
	tcb->sPtr = &(stacks[pid][STACKSIZE-16]); // thread stack pointer
  stacks[pid][STACKSIZE-1] = 0x01000000;   // thumb bit
  stacks[pid][STACKSIZE-3] = 0x14141414;   // R14
  stacks[pid][STACKSIZE-4] = 0x12121212;   // R12
  stacks[pid][STACKSIZE-5] = 0x03030303;   // R3
  stacks[pid][STACKSIZE-6] = 0x02020202;   // R2
 	stacks[pid][STACKSIZE-7] = 0x01010101;   // R1
  stacks[pid][STACKSIZE-8] = 0x00000000;   // R0
  stacks[pid][STACKSIZE-9] = 0x11111111;   // R11
  stacks[pid][STACKSIZE-10] = 0x10101010;  // R10
  stacks[pid][STACKSIZE-11] = 0x09090909;  // R9
  stacks[pid][STACKSIZE-12] = 0x08080808;  // R8
  stacks[pid][STACKSIZE-13] = 0x07070707;  // R7
  stacks[pid][STACKSIZE-14] = 0x06060606;  // R6
  stacks[pid][STACKSIZE-15] = 0x05050505;  // R5
  stacks[pid][STACKSIZE-16] = 0x04040404;  // R4
}

/**
 * Initialize all TCB's with status = FREE; finding the next available TCB is based on unusued TCB's having
 * status = FREE to be used, so this step is important.
 */
void tcbInit(void) {
	int iii;
	for(iii = 0; iii < NUMTHREADS; iii++) {
		tcbs[iii].status = FREE;
	}
}

/**
 * initialize operating system, disable interrupts until OS_Launch
 * initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers
 * @note this is directly from ValvanoWare
 */
void OS_Init(void) {
	OS_DisableInterrupts();
	PLL_Init(Bus80MHz);
#ifdef DEBUG
	PortF_Init();
#endif
	tcbInit();
	WideTimer3_Init();

	NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0x00FFFFFF)|0xE0000000; // priority 7

	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&(~NVIC_SYS_PRI3_PENDSV_M))|0x00E00000;; // priority 7
}


/**
 * suspend execution of currently running thread and trigger PendSV interrupt (aka context switch)
 * scheduler will choose another thread to execute
 * Same function as OS_Sleep(0)
 */
void OS_Suspend(void) {
	NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
}

/**
 * Finds the first available TCB that is free
 * If there is no free TCB, returns 0, else returns 1
 * Sets curTID, a global variable
 */
int findTCB(void) {
	int count = 0; 
	while(!(tcbs[curTID].status == FREE)) { // unallocated TCB has null SP
		count++;
		if(count == NUMTHREADS) {
			return 0;
		}
		curTID = (curTID + 1) % NUMTHREADS;
	}
	
	return 1; 
}

/**
 * adds an allocated tcb to the pool
 * @param newTCB a pointer to the tcb to add
 * @param priority priority of thread
 * @param task code for thread to execute 
 */ 
void addTCB(TCB * newTCB, uint8_t priority, void(*task)(void)) {
	newTCB->status = ACTIVE;
	newTCB->sleepCtr = 0;
	newTCB->pid = curTID;
	newTCB->priority = priority;
	stackInit(newTCB);
	stacks[curTID][STACKSIZE-2] = (int32_t)(task); // PC
	curTID = (curTID + 1) % NUMTHREADS;

	// add to the DLL
	DLL_Add(&priorityDLL[priority], newTCB);
}

/**
 * Add a foreground thread to the thread queue. PID is assigned on a round robin basis - the first
 * available PID is chosen. If no PIDs are available, the function returns a 0. New threads are added
 * at the end of the threadDLL for round robin scheduling
 * @param task task to add to thread queue
 * @param stackSize stack size for the new thread
 * @param priority priority of the new thread
 * @return 1 if thread add was successful; 0 if no TCBs are available
 */
int OS_AddThread(void(*task)(void), unsigned long stackSize, unsigned long priority) {
	int32_t primask, count;
	uint32_t mode = CheckMode();
	
	primask = StartCritical();
	TCB* newTCB; 
	if(!findTCB()) { // find next open TCB (if exists)
		EndCritical(primask);
		return 0; 
	}
	
	newTCB = &tcbs[curTID];
	
	addTCB(newTCB, priority, task); 
	
	EndCritical(primask);
	
	if(running) {
		OS_Suspend();
	}
	
	return 1;
}

/**
 * start the scheduler and enable interrupts
 * @param theTimeSlice number of 12.5ns clock cycles for each time slice
 * @note this is directly from ValvanoWare
 */
void OS_Launch(unsigned long theTimeSlice) {
	int i;
#ifdef INC_SYSTICK
	NVIC_ST_RELOAD_R = theTimeSlice - 1; // reload value
  NVIC_ST_CTRL_R = 0x00000007; // enable, core clock and interrupt arm
#endif
	if(periodictimer >= 1) WTIMER4_CTL_R |= TIMER_CTL_TAEN;
	if(periodictimer >= 2) WTIMER4_CTL_R |= TIMER_CTL_TBEN;
	for(i = 0; i < MAXPRIORITY; i++) {
		if(priorityDLL[i].size != 0) {
			curThread = priorityDLL[i].nextThread; 
			priorityDLL[i].nextThread = curThread->nextTCB; 
			break; 
		}
	}
	StartOS();
}

/**
 * At each SysTick interrupt, iterate through sleep queue and decrement the sleep counters.
 * If the sleep time is up for a TCB, add it back to the active thread queue. Trigger
 * a context switch at the end because a time-slice has finished.
 */
void SysTick_Handler(void) {
	TCB * curTCB = sleepDLL.head;
	TCB * next;
	
	systemTime++;
	if(curTCB != NULL) {
		volatile int x;
		x = 1;
	}
	while(curTCB != NULL && sleepDLL.size != 0) { // iterate through sleep queue and decrement counters
		next = curTCB->nextTCB == sleepDLL.head ? NULL : curTCB->nextTCB;

		curTCB->sleepCtr--;
		if(curTCB->sleepCtr == 0) {
			DLL_Remove(&sleepDLL, curTCB);
			uint8_t priority = curTCB->priority;
			
			OS_DisableInterrupts();
			DLL_Add(&priorityDLL[priority], curTCB);
			OS_EnableInterrupts();
		}

		curTCB = next;
	}

	OS_Suspend();
}


/**
 * Initialize a semaphore's starting value
 * @param semaPt semaphore to initialize
 * @param value value to initialize semaphore to
 */
void OS_InitSemaphore(Sema4Type *semaPt, long value) {
	semaPt->Value = value;
}

/**
 * Puts a thread to sleep. Remove sleeping thread from active queue and place in sleep queue.
 * OS_Sleep(0) is the same thing as OS_Suspend.
 * @param sleepTime time for thread to sleep
 */
void OS_Sleep(unsigned long sleepTime) {
	if(sleepTime == 0) {
		OS_Suspend();
		return;
	}

	curThread->sleepCtr = sleepTime;
	curThread->status = SLEEP;
	removeFromActive(&sleepDLL);
	OS_Suspend();
}


/**
 * kill the currently running thread; remove from active queue; release the stack and TCB
 */
void OS_Kill(void) {
	OS_DisableInterrupts();
	
	curThread->status = FREE;
	DLL_Remove(&priorityDLL[curThread->priority], curThread);

	OS_EnableInterrupts();
	OS_Suspend();
}

/**
 * Decrement semaphore; if semaphore value is < 0, thread blocks itself.
 * @param semaPt semaphore to decrement
 * @note this code is directly from the lecture
 */
void OS_Wait(Sema4Type *semaPt) {
	int primask = StartCritical();
	semaPt->Value -= 1;
	if(semaPt->Value < 0) {
		curThread->status = BLOCKED;
		removeFromActive(&(semaPt->blockedDLL));
		EndCritical(primask);
		OS_Suspend();
		return;
	}
	EndCritical(primask);
}

/**
 * Increment semaphore
 * If Value <= 0, will wake up the first semaphore in the blocked DLL
 * blocked DLL acts like a round robin system
 * @param semaPt semaphore to increment
 * @note this code is directly from lecture
 */
void OS_Signal(Sema4Type *semaPt) {
	int primask = StartCritical();
	semaPt->Value += 1;
	if(semaPt->Value <= 0) {
		TCB *head = semaPt->blockedDLL.head;
		DLL_Remove(&(semaPt->blockedDLL), head);
		head->status = ACTIVE;
		DLL_Add(&priorityDLL[head->priority], head);
		EndCritical(primask);
		OS_Suspend();
		return;
	}
	EndCritical(primask);
}

/**
 * Set semaphore to 0; if semaphore value is < 0, spin until the value increases
 * @param semaPt semaphore to decrement
 * @note this code is directly form the lecture
 */
void OS_bWait(Sema4Type *semaPt) {
	unsigned long primask = StartCritical();
	while(semaPt->Value == 0) {
		curThread->status = BLOCKED;
		removeFromActive(&(semaPt->blockedDLL));
		EndCritical(primask);
		OS_Suspend();
		primask = StartCritical();
	}
	semaPt->Value = 0;
	EndCritical(primask);
}

/**
 * Set semaphore value to 1
 * @param semaPt semaphore to increment
 * @note this is directly from the lecture
 */
void OS_bSignal(Sema4Type *semaPt) {
	int primask = StartCritical();	
	semaPt->Value = 1;
	if(semaPt->blockedDLL.size > 0) {
		TCB *head = semaPt->blockedDLL.head;
		DLL_Remove(&(semaPt->blockedDLL), head);
		head->status = ACTIVE;
		DLL_Add(&priorityDLL[head->priority], head);
		EndCritical(primask);
		OS_Suspend();
		return;
	}
	EndCritical(primask);
}

// mailboxes and fifos
unsigned long mailboxValue[4] = {0, };
Sema4Type mailboxRdy = {0};
Sema4Type mailboxAck = {0};

/**
 * Initialize mailbox communication and set up semaphores. Initially, the mailbox is acknowledged and
 * ready for someone to put data in it.
 */
void OS_MailBox_Init(void) {
	OS_InitSemaphore(&mailboxRdy, 0);
	OS_InitSemaphore(&mailboxAck, 1);
}

/**
 * Put data into the mailbox. Spins if data has not yet been received from the mailbox.
 * @param data data to put in the mailbox
 */
void OS_MailBox_Send(unsigned long *data) {
	if(mailboxAck.Value == 1) {
		memcpy(mailboxValue, data, sizeof(mailboxValue));
		OS_Signal(&mailboxRdy);
		mailboxAck.Value = 0;
	}
	else {
		DataLost++;
	}
}

/**
 * remove mail from the mailbox. Spins if data has not been put into the mailbox
 * @return data from mailbox
 */
void OS_MailBox_Recv(unsigned long *data) {
	OS_Wait(&mailboxRdy);
	memcpy(data, mailboxValue, sizeof(mailboxValue));
	OS_Signal(&mailboxAck);
}

/**
 * Initializes Wide Timer 4A for periodic thread calling
 * @param task task to call at each timer interrupt
 * @param period period at which to fire the timer interrupt
 * @param priority priority of timer interrupt
 * @note this is modified ValvanoWare
 */
void WideTimer4A_Init(void(*task)(void), uint32_t period, unsigned long priority) { 
	SYSCTL_RCGCWTIMER_R |= 0x10;
	while((SYSCTL_RCGCWTIMER_R&0x10) == 0){};
	task_a = task;
	WTIMER4_CTL_R &= ~TIMER_CTL_TAEN;
	WTIMER4_CFG_R = 0x00000004;
	WTIMER4_TAMR_R = 0x00000002;
	WTIMER4_TAPR_R = 0;
	WTIMER4_TAILR_R = period - 1;
	WTIMER4_ICR_R |= TIMER_ICR_TATOCINT;
	WTIMER4_IMR_R |= TIMER_IMR_TATOIM;
	NVIC_PRI25_R = (NVIC_PRI25_R & 0xFF00FFFF)|(priority << 21);
	NVIC_EN3_R = (1 << (102-96));
}

/**
 * Initializes Wide Timer 4B for periodic thread calling
 * @param task task to call at each timer interrupt
 * @param period period at which to fire the timer interrupt
 * @param priority priority of timer interrupt
@note this is modified ValvanoWare
 */
void WideTimer4B_Init(void(*task)(void), uint32_t period, unsigned long priority) {
	SYSCTL_RCGCWTIMER_R |= 0x10;
	while((SYSCTL_RCGCWTIMER_R&0x08) == 0){};
	task_b = task;
	WTIMER4_CTL_R &= ~TIMER_CTL_TBEN;
	WTIMER4_CFG_R = 0x00000004;
	WTIMER4_TBMR_R = 0x00000002;
	WTIMER4_TBPR_R = 0;
	WTIMER4_TBILR_R = period - 1;
	WTIMER4_ICR_R |= TIMER_ICR_TBTOCINT;
	WTIMER4_IMR_R |= TIMER_IMR_TBTOIM;
	NVIC_PRI25_R = (NVIC_PRI25_R&0x00FFFFFF)|(priority << 29);
	NVIC_EN3_R |= (1<<(103-96));      // 9) enable IRQ 35 in NVIC
}

/**
 * Add a background periodic task; this function will typically receive highest priority
 * @param task task to run periodically
 * @param period period the task should execute
 * @param priority priority of the background task
 * @return
 */
int OS_AddPeriodicThread(void(*task)(void), unsigned long period, unsigned long priority) {
	if(periodictimer == 0) {
		WideTimer4A_Init(task, period, priority);
		periodictimer = 1;
	}
	else {
		WideTimer4B_Init(task, period, priority);
		periodictimer += 1;
	}
	return 1;
}

/**
 * Call specified periodic task and increment the system time.
 * @note code copied and modified from ValvanoWare
 */
void WideTimer4A_Handler(void) {
	WTIMER4_ICR_R = TIMER_ICR_TATOCINT;
	task_a();
}

/**
 * Call specified periodic task and increment the system time.
 * @note code copied and modified from ValvanoWare
 */
void WideTimer4B_Handler(void) {
	WTIMER4_ICR_R = TIMER_ICR_TBTOCINT;
	task_b();
}


/**
 * Returns thread ID for the currently running thread
 * @return thread id (> 0)
 */
unsigned long OS_Id(void) {
	return curThread->pid;
}

/**
 * Return current system time in bus cycles. Since the timer used here is a countdown timer,
 * we subtract the current value from the maximum times value. The timer used decrements each
 * microsecond (thanks to a prescaler), so we also must multiply to convert to microseconds
 * to bus cycles.
 */
unsigned long OS_Time(void) {
	return (0xFFFFFFFF - WTIMER3_TAR_R);
}

/**
 * Calculate difference between two times.
 * @param start start time with 1 microsecond resolution, in 12.5 ns units
 * @param stop stop time with 1 microsecond resolution, in 12.5 ns units
 * @return time difference in 12.5ns units
 */
unsigned long OS_TimeDifference(unsigned long start, unsigned long stop) {
	uint32_t timeDiff;
	timeDiff = stop - start;
	return timeDiff;
}

/**
 * Clear system time value
 */
void OS_ClearMsTime(void) {
	systemTime = 0;
}

/**
 * Return system time in milliseconds. System time is stored in 500 microsecond units,
 * so we multiply by two to convert to milliseconds.
 * @return system time in milliseconds
 */
unsigned long OS_MsTime(void) {
	return systemTime>>1;
}

/**
 * Initialize wide timer 3 to keep overall system time. Wide timer 3 keeps time in 1 microsecond by using a
 * prescaler value of 80. It is initialized to to 0xFFFFFFFF and counts down from there. This timer will last
 * a little over an hour, which is plenty of time for us.
 * @note this code is modified ValvanoWare
 */
void WideTimer3_Init(void){
  SYSCTL_RCGCWTIMER_R |= 0x08;   // 0) activate TIMER3
  WTIMER3_CTL_R = 0x00000000;    // 1) disable TIMER3A during setup
  WTIMER3_CFG_R = 0x00000004;    // 2) configure for 32-bit mode
  WTIMER3_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  WTIMER3_TAILR_R = 0xFFFFFFFF;    // 4) reload value
  WTIMER3_TAPR_R = 79;            // 5) prescaler = 80 for 1 microsecond resolution
  WTIMER3_ICR_R = 0x00000001;    // 6) clear TIMER3A timeout flag
  WTIMER3_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI21_R = (NVIC_PRI21_R&0xFFFFFF00)|0x000000C0; // 8) priority 1
// interrupts enabled in the main program after all devices initialized
// vector number 51, interrupt number 35
  NVIC_EN3_R = 1<<(100-96);      // 9) enable IRQ 35 in NVIC
  WTIMER3_CTL_R = 0x00000001;    // 10) enable TIMER3A
}

// simple do nothing handler
// tells if time in system has overflown
void WideTimer3A_Handler(void){
	overflow++;
	WTIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER3A timeout
}
