	
	AREA |.text|, CODE, READONLY, ALIGN=2
	THUMB
	REQUIRE8
	PRESERVE8

	IMPORT curThread
	IMPORT nextThread
	IMPORT OS_Scheduler
	IMPORT OS_Id
	IMPORT OS_Kill
	IMPORT OS_Sleep
	IMPORT OS_Time
	IMPORT OS_AddThread
	EXPORT PendSV_Handler
	EXPORT SVC_Handler
	EXPORT OS_DisableInterrupts
	EXPORT OS_EnableInterrupts
	EXPORT StartOS
	EXPORT CheckMode

CheckMode
	MRS		R0, IPSR
	; 0 = thread mode
	; !0 = handler
	BX		LR

OS_DisableInterrupts
	CPSID 	I
	BX 		LR
	
OS_EnableInterrupts
	CPSIE 	I
	BX 		LR

SVC_Handler
; taken directly from lecture slides
	LDR		R12, [SP, #24]	; Return address
	LDRH	R12, [R12, #-2]	; SVC instruction is 2 bytes
	BIC		R12, #0xFF00	; Extract ID in R12
	LDM		SP, {R0-R3}		; Get any parameters
	PUSH	{R0, LR}
; switch case statements for different SVC calls
; SVC #0 call	
	CMP		R12, #0
	BNE		next1
	BL		OS_Id
	B		return

next1 ; SVC #1 call
	CMP		R12, #1
	BNE		next2
	BL		OS_Kill
	B		return

next2 ; SVC #2 call
	CMP		R12, #2
	BNE		next3
	BL		OS_Sleep
	B		return

next3 ; SVC #3 call
	CMP		R12, #3
	BNE		next4
	BL		OS_Time
	B		return

next4 ; SVC #4 call
	CMP		R12, #4
	BNE		return
	BL		OS_AddThread
	B		return

return
	POP		{R0, LR}
	STR		R0, [SP]		; Store return value
	BX		LR				; Return from exception

; code directly from ValvanoWare and lecture slides
; slightly modified to fit with our scheduling approach 
PendSV_Handler
	PUSH	{LR, R0}
	BL		OS_Scheduler			; call scheduler to get next thread
	CPSID I
	POP 	{LR, R0}
	PUSH 	{R4-R11}					; push current registers onto stack
 		 	
	LDR 	R0, =curThread		; store current stack pointer in to curTCB's SPtr
	LDR 	R1, [R0] 
	STR 	SP, [R1]
 		 	
	LDR 	R1, =nextThread  	; store nextThread into curThread pointer
	LDR		R1, [R1]
	STR 	R1, [R0]
 		 	
	LDR 	SP, [R1]					; update with new thread's stack pointer and pop registers
	POP		{R4-R11}
	
	CPSIE I
	BX 		LR
	
; pulled directly from ValvanoWare
StartOS
  LDR     R0, =curThread     ; currently running thread
  LDR     R2, [R0]           ; R2 = value of RunPt
  LDR     SP, [R2]           ; new thread SP; SP = RunPt->stackPointer;
  POP     {R4-R11}           ; restore regs r4-11
  POP     {R0-R3}            ; restore regs r0-3
  POP     {R12}
  POP     {LR}               ; discard LR from initial stack
  POP     {LR}               ; start location
  POP     {R1}               ; discard PSR
  CPSIE   I                  ; Enable interrupts at processor level
  BX      LR                 ; start first thread

	ALIGN
	END
