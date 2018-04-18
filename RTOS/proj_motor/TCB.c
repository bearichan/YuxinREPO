#include <stdint.h>
#include "TCB.h"
#include <stddef.h>

/**
 * Adds a tcb to a doubly linked list
 * Updates size of the DLL
 * @param TCB *tcb that will be added to a DLL *dll at the tail
 */
void DLL_Add(DLL *dll, TCB *tcb) {
	if(dll->head == NULL) {
		dll->head = tcb;
		dll->tail = tcb; 
		dll->nextThread = tcb;
	}
	
	dll->tail->nextTCB = tcb;
	tcb->prevTCB = dll->tail;
	tcb->nextTCB = dll->head;
	dll->tail = tcb;
	dll->head->prevTCB = dll->tail;
	if(dll->size == 1) {
		dll->nextThread = tcb;
	}
	dll->size++;
}

/**
 * Removes a specified tcb from a dll that is passed into the function
 * Updates size of the DLL
 * @param TCB *tcb that will be removed from a DLL *dll
 */
void DLL_Remove(DLL *dll, TCB *tcb) {
	dll->size--;
	tcb->prevTCB->nextTCB = tcb->nextTCB;
  tcb->nextTCB->prevTCB = tcb->prevTCB; 
	
	if(tcb == dll->head && tcb == dll->tail) {
		dll->head = NULL;
		dll->tail = NULL;
	}
	else if(tcb == dll->head) {
		dll->head = tcb->nextTCB;
  }
	else if(tcb == dll->tail) {
    dll->tail = tcb->prevTCB;
  }
}
