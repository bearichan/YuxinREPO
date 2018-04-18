#include <stdint.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "UART.h"
#include "ST7735.h"
#include "ADC.h"
#include "OS.h"

/*! *************** Interpreter_OutCRLF ********************
 * Prints out CR and LF to UART screen
 */
void Interpreter_OutCRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}

/*! *************** cmdUnimplemented ********************
 * Private function
 * Placeholder function for unimplemented interpreter commands
 * Writes to UART
 */
void cmdUnimplemented(void) {
	UART_OutString("Functionality unimplemented. Please try another command.");
}


/*!
 * print introduction to UART
 * precondition UART initialized correctly
 * postcondition none
 */
void printIntro(void) {
	Interpreter_OutCRLF();
	UART_OutString("Greeting: Hello to you, master. I am referred to as HK-47."); Interpreter_OutCRLF();
	UART_OutString("Query: Would you be so kind as to input a command, master?");
}

/*! *************** cmdLCD ********************
 * Private function
 * Proccess user "LCD" command
 * Writes data to LCD
 */
void cmdLCD(void) {
	char string[20];
	uint32_t line, device;
	UART_OutString("Query: What device are you using, master? ");
	device = UART_InUDec(); Interpreter_OutCRLF();
	UART_OutString("Query: What line shall I write to, master? ");
	line = UART_InUDec(); Interpreter_OutCRLF();
	UART_OutString("Query: What would you have me say on the screen? ");
	UART_InString(string, 19); Interpreter_OutCRLF();
	ST7735_Message(device, line, string, 0);
}

/*! *************** cmdSTAT ********************
 * Private function
 * Proccess user "STAT" command
 * Writes statistics of ADC sampling, DAS, and PID to UART
 */
void cmdSTAT(void) {
	cmdUnimplemented();
}
/*! *************** cmdADC ********************
 * Private function
 * This function will do nothing for the purpose of this lab
 */
void cmdADC(void) {
	cmdUnimplemented();
}

/*! *************** cmdTIME ********************
 * Private function
 * Collects the time in seconds from OS
 * Writes time in seconds to LCD
 */
void cmdTIME(void) {
	char buf[21];
	uint32_t time = OS_MsTime();
	ST7735_Message(0, 0, buf, time);
}

/*! *************** cmdHELP ********************
 * Private function
 * Prints to UART capable functions
 */
void cmdHELP(void) {
	UART_OutString("List of functions:"); Interpreter_OutCRLF();
	UART_OutString("\t1)lcd"); Interpreter_OutCRLF();
	UART_OutString("\t2)os"); Interpreter_OutCRLF();
	UART_OutString("\t3)stat"); Interpreter_OutCRLF();
	UART_OutString("\t4)gpio"); Interpreter_OutCRLF();
	UART_OutString("\t5)file"); Interpreter_OutCRLF();
	UART_OutString("\t6)help"); Interpreter_OutCRLF();
}

/**
 * Private function
 * Prints to UART capable OS specific functions
 */
void cmdOSHELP(void) {
	UART_OutString("List of OS functions:"); Interpreter_OutCRLF();
	UART_OutString("\t1)time"); Interpreter_OutCRLF();
	UART_OutString("\t2)clear"); Interpreter_OutCRLF();
	UART_OutString("\t3)help"); Interpreter_OutCRLF();
}

/**
 * Private function
 * Prints to UART capable GPIO specific functions
 */
void cmdGPIOHELP(void) {
	UART_OutString("List of GPIO functions:"); Interpreter_OutCRLF();
	UART_OutString("\t1)arm"); Interpreter_OutCRLF();
	UART_OutString("\t2)disarm"); Interpreter_OutCRLF();
	UART_OutString("\t3)help"); Interpreter_OutCRLF();
}

/**
 * Private function
 * Prints to UART capable file system commands
 */
void cmdFILEHELP(void) {
	UART_OutString("Answer: Master, I am capable of the following aperiodic FILE functions:"); Interpreter_OutCRLF();
	UART_OutString("\t1)dir"); Interpreter_OutCRLF();
	UART_OutString("\t2)delete"); Interpreter_OutCRLF();
	UART_OutString("\t3)format"); Interpreter_OutCRLF();
	UART_OutString("\t4)print"); Interpreter_OutCRLF();
	UART_OutString("\t5)help"); Interpreter_OutCRLF();
}

/*! *************** cmdCTIME ********************
 * Private function
 * Clears timer
 * Calls OS_ClearPeriodicTimer()
 */
void cmdCTIME(void) {
	OS_ClearMsTime();
}

/**
 * private function
 * Proccesses potential user instruction for OS tasks.
 */
void cmdOS(char *ptr) {
	if(strcmp(ptr, "time") == 0) cmdTIME();
	else if(strcmp(ptr, "clear") == 0) cmdCTIME();
	else if(strcmp(ptr, "help") == 0) cmdOSHELP();
	else {
		UART_OutString("Command not recognized."); Interpreter_OutCRLF();
		UART_OutString("Type \"help\" for list of commands."); Interpreter_OutCRLF();
	}
}

/*! *************** Interpreter_String ********************
 * Processes user command from UART
 * Will call private function associated with command
 * If not a command, writes to UART that command does not exist
 * @param[in] ptr String that contains user command
 */
void Interpreter_String(char *ptr) {
	if(strcmp(ptr, "help") == 0) cmdHELP();
	else if(strcmp(ptr, "os") == 0) {
		UART_OutString("Input OS command."); Interpreter_OutCRLF();
		UART_OutString(">");
		UART_InString(ptr, 19); Interpreter_OutCRLF();
		cmdOS(ptr);
	}
	else if(strcmp(ptr, "lcd") == 0) cmdLCD();
	else if(strcmp(ptr, "stat") == 0) cmdSTAT();
	//else if(strcmp(ptr, "ADC") == 0) cmdADC();
	else {
		UART_OutString("Command not recognized."); Interpreter_OutCRLF();
		UART_OutString("Type \"help\" for list of commands."); Interpreter_OutCRLF();
	}
}

/*! *************** Interpreter ********************
 * Public function
 * Processes user instructions over the UART terminal
 * Is a function that will never terminate
 */
void Interpreter(void) {
	char command_in[20];
	UART_Init();
	printIntro();
	while(1) {
		Interpreter_OutCRLF();
		UART_OutString(">");
		UART_InString(command_in, 19); Interpreter_OutCRLF();
		Interpreter_String(command_in);
	}
}
