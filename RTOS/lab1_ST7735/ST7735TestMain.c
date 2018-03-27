// ST7735TestMain.c
// Runs on LM4F120/TM4C123
// Test the functions in ST7735.c by printing basic
// patterns to the LCD.
//    16-bit color, 128 wide by 160 high LCD
// Daniel Valvano
// March 30, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014

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

// hardware connections
// **********ST7735 TFT and SDC*******************
// ST7735
// Backlight (pin 10) connected to +3.3 V
// MISO (pin 9) unconnected
// SCK (pin 8) connected to PA2 (SSI0Clk)
// MOSI (pin 7) connected to PA5 (SSI0Tx)
// TFT_CS (pin 6) connected to PA3 (SSI0Fss)
// CARD_CS (pin 5) unconnected
// Data/Command (pin 4) connected to PA6 (GPIO), high for data, low for command
// RESET (pin 3) connected to PA7 (GPIO)
// VCC (pin 2) connected to +3.3 V
// Gnd (pin 1) connected to ground

// **********wide.hk ST7735R with ADXL345 accelerometer *******************
// Silkscreen Label (SDC side up; LCD side down) - Connection
// VCC  - +3.3 V
// GND  - Ground
// !SCL - PA2 Sclk SPI clock from microcontroller to TFT or SDC
// !SDA - PA5 MOSI SPI data from microcontroller to TFT or SDC
// DC   - PA6 TFT data/command
// RES  - PA7 TFT reset
// CS   - PA3 TFT_CS, active low to enable TFT
// *CS  - (NC) SDC_CS, active low to enable SDC
// MISO - (NC) MISO SPI data from SDC to microcontroller
// SDA  – (NC) I2C data for ADXL345 accelerometer
// SCL  – (NC) I2C clock for ADXL345 accelerometer
// SDO  – (NC) I2C alternate address for ADXL345 accelerometer
// Backlight + - Light, backlight connected to +3.3 V

// **********wide.hk ST7735R with ADXL335 accelerometer *******************
// Silkscreen Label (SDC side up; LCD side down) - Connection
// VCC  - +3.3 V
// GND  - Ground
// !SCL - PA2 Sclk SPI clock from microcontroller to TFT or SDC
// !SDA - PA5 MOSI SPI data from microcontroller to TFT or SDC
// DC   - PA6 TFT data/command
// RES  - PA7 TFT reset
// CS   - PA3 TFT_CS, active low to enable TFT
// *CS  - (NC) SDC_CS, active low to enable SDC
// MISO - (NC) MISO SPI data from SDC to microcontroller
// X– (NC) analog input X-axis from ADXL335 accelerometer
// Y– (NC) analog input Y-axis from ADXL335 accelerometer
// Z– (NC) analog input Z-axis from ADXL335 accelerometer
// Backlight + - Light, backlight connected to +3.3 V

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "ST7735.h"
#include "PLL.h"
#include "Timer2.h"
#include "OS.h"
#include "ADC.h"
#include "UART.h"
#include "../inc/tm4c123gh6pm.h"

#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
#define LEDS      (*((volatile uint32_t *)0x40025038))
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08
#define WHEELSIZE 8           // must be an integer multiple of 2
#define PARSE_MAX_SIZE 3
                              //    red, yellow,    green, light blue, blue, purple,   white,          dark
const uint32_t COLORWHEEL[WHEELSIZE] = {RED, RED+GREEN, GREEN, GREEN+BLUE, BLUE, BLUE+RED, RED+GREEN+BLUE, 0};

void DelayWait10ms(uint32_t n);
void DisableInterrupts(void);
void EnableInterrupts(void);

//---------------------OutCRLF---------------------
// Output a CR,LF to UART to go to a new line
// Input: none
// Output: none
void OutCRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}

void UserTask(void){
  static uint32_t i = 0;
  PF2 ^= 0x04;
//  LEDS = COLORWHEEL[i&(WHEELSIZE-1)];
  i = i + 1;
}

uint16_t buffer[100];
int main(void){
//  char i;
//  char string[20];  // global to assist in debugging

  PLL_Init(Bus80MHz);                  // set system clock to 80 MHz
  DisableInterrupts();

  SYSCTL_RCGCGPIO_R |= 0x20;  // activate port F
	//Delay1ms(1000);
  GPIO_PORTF_DIR_R |= 0x07;   // make PF2 output (PF2 built-in LED)
  GPIO_PORTF_AFSEL_R &= ~0x07;// disable alt funct on PF2
  GPIO_PORTF_DEN_R |= 0x07;   // enable digital I/O on PF2
						  // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;     // disable analog functionality on PF
	
  ST7735_InitR(INITR_REDTAB);
  UART_Init();

  ST7735_Message(0,0, "test this ", 0);
  ST7735_Message(1,0, "test this next ", 1);
	
	
  ADC_Start_Collect(2,10,150);
  OS_AddPeriodicThread(&UserTask,8000000,1);	//10Hz

  EnableInterrupts();


  while(1){

    UART_Parse_String();
		
  }
}



char parse_buf[PARSE_MAX_SIZE];
int UART_Parse_String(){

  char character;
  int i;
  character = UART_InChar();
  
  for(i = 0; i< PARSE_MAX_SIZE-1  ; i++){
    parse_buf[i] = parse_buf[i+1];
  }
  parse_buf[PARSE_MAX_SIZE-1] = character;
  
  
  if(0 == strcmp(parse_buf,"001")){
    memset(parse_buf,0, PARSE_MAX_SIZE);
    ST7735_Message(0,4, "mode ", 1);
  }
  else if(0 == strcmp(parse_buf,"002")){
    ST7735_Message(0,4, "mode ", 2);
  }
  else if(0 == strcmp(parse_buf,"003")){
    ST7735_Message(1,1, "mode ", 3);
  }

  return 0;
}
