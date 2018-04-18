#include <stdint.h>
#include <stdbool.h>

#define STACK_SIZE 128

typedef enum {ACTIVE, SLEEP, BLOCKED, FREE} threadStatus;

typedef struct PCB PCB;

/*! ******** TCB ************
 * TCB data structure definition
 */
typedef struct TCB {
	int32_t *sPtr; // thread stack pointer
	struct TCB *nextTCB; // pointer to next TCB
	struct TCB *prevTCB; // pointer to previous TCB
	uint16_t pid; // unique process ID
	uint32_t sleepCtr; // counter for sleep
	uint8_t priority;
	uint8_t status;
} TCB;

/*! ******** DLL ************
 * Defines a doubly linked list of TCBs
 */
typedef struct DLL {
	TCB *head;
	TCB *tail;
	TCB *nextThread;
	uint32_t size;
} DLL;

/*! ******** DLL_Add ************
 * 
 */
void DLL_Add(DLL *dll, TCB *tcb);

void DLL_Remove(DLL *dll, TCB *tcb);
