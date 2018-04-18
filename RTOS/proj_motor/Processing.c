#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "Timer.h"
#include "OS.h"
#include "IR.h"
#include "string.h"
#include "Ping.h"
#include "FIFO.h"
#include "can0.h"
#include "Processing.h"
#include "ST7735.h"
#include "pwm.h"

AddIndexFifo(CAN, 32, SensorMSG, 1, 0);
Sema4Type CANLock;
int PutDataLost = 0;
extern int DataLost;

extern unsigned long Ping2_PulseWidth; 
long ADC2millimeter(long adcSample){
  if(adcSample<494) return 799; // maximum distance 80cm
  return (268130/(adcSample-159));  
}

long median(long u1,long u2,long u3){ 
long result;
  if(u1>u2)
    if(u2>u3)   result=u2;     // u1>u2,u2>u3       u1>u2>u3
      else
        if(u1>u3) result=u3;   // u1>u2,u3>u2,u1>u3 u1>u3>u2
        else      result=u1;   // u1>u2,u3>u2,u3>u1 u3>u1>u2
  else 
    if(u3>u2)   result=u2;     // u2>u1,u3>u2       u3>u2>u1
      else
        if(u1>u3) result=u1;   // u2>u1,u2>u3,u1>u3 u2>u1>u3
        else      result=u3;   // u2>u1,u2>u3,u3>u1 u2>u3>u1
  return(result);
}

void PutCANData(SensorMSG* message) {
	int rc = CANFifo_Put(*message);
	if(rc) {
		OS_Signal(&CANLock);
	}
	if(!rc) {
		PutDataLost++;
	}
}

void GetCANData(SensorMSG* data) {
	OS_Wait(&CANLock);
	CANFifo_Get(data);
}

void CANSender(void) {
	SensorMSG data;
	uint8_t xmit[8];
	while(1) {
		GetCANData(&data);
		uint32_t * startPtr = (uint32_t *) &data;
		memcpy(xmit, startPtr + 1, 8);
		CAN0_SendData(xmit);
	}
}

uint32_t Steering;     // 625 to 3125
uint32_t SteeringMode; // 0 for increase, 1 for decrease
uint32_t Power;
uint32_t recv_speed;
uint8_t recv_dir;
uint8_t pre_recv_dir;

 uint8_t rcv[8]; 
void CANReceiver(void) {
  
  unsigned long ir0, ir1, ir2, ir3, ping; 
  int16_t devia_speed;

  uint16_t  Left_PWM;
  uint16_t  Right_PWM;

  while(1) 
  {
    CAN0_GetMail(rcv);
//      rcv[0] direction 0~0xFF   7F:MID
//      rcv[1] f/b rcv[1]&0x80 speed 0~0x7F  
//    
    Steering =  (uint32_t)(SERVOMAX - (rcv[0] * 4.313));   //0~255 --> MIN to MAX, resolution is 4.313
    if(Steering > SERVOMAX)
    {
        Steering = SERVOMAX; // go to center and
    }
    else if(Steering < SERVOMIN)
    {
        Steering =  SERVOMIN;
    }
    
    Servo_Duty(Steering);

    recv_dir =  (rcv[1] & 0x80) >> 7;        //take the MSB for direction 1->forward, 0-> backward
//    recv_dir = 1;
//    recv_speed = 50;

//    devia_speed = rcv[0] - 0x7F;

    if(pre_recv_dir != recv_dir)
    {
       if(recv_dir == 0)
       {
          Left_Init(12500, 12500,1);          // initialize PWM0, 100 Hz
          Right_InitB(12500, 12500,1);   // initialize PWM0, 100 Hz
       }
       else
       {
          Left_InitB(12500, 12500,1);          // initialize PWM0, 100 Hz
          Right_Init(12500, 12500,1);   // initialize PWM0, 100 Hz
       }
    }

    if(recv_dir == 0)   //forward            
    {
      if(devia_speed <= 0)     //left turn     left wheel should be slower
      {
        Left_PWM = 12500 - 98 * (rcv[1] & 0x7F);
        Right_PWM = 12500 - 98 *(rcv[2] & 0x7F);

      }
      else
      {
        Left_PWM = 12500 - 98 * (rcv[1] & 0x7F);
        Right_PWM = 12500 - 98 *(rcv[2] & 0x7F);

      }
    }
    else         //backward         cpomment for temp use
    {
      Left_PWM = 12500 - 98 * (rcv[1] & 0x7F);
      Right_PWM = 12500 - 98 * (rcv[2] & 0x7F);
    }
    
    
    if(Left_PWM > POWERMAX)
    {
        Left_PWM = POWERMAX; // go to center and
    }
    else if(Left_PWM < POWERMIN)
    {
        Left_PWM =  POWERMIN;
    }
      if(Right_PWM > POWERMAX)
    {
        Right_PWM = POWERMAX; // go to center and
    }
    else if(Right_PWM < POWERMIN)
    {
        Right_PWM =  POWERMIN;
    }
    
    if(recv_dir == 0)   //forward            
    {
      Left_Duty(Left_PWM,1);
      Right_DutyB(Right_PWM,1);
    }
    else
    {
      Left_DutyB(Left_PWM,1);
      Right_Duty(Right_PWM,1);
    }
    
    pre_recv_dir = recv_dir;
  }
}

//void DAS(void){
//	unsigned long sensor[4][2];
//	unsigned long distance[4];
//	unsigned long data[4];
//	unsigned long lastPing;
//	unsigned long ping; 
//	
//	while(1) {
//		OS_MailBox_Recv(data);
//		distance[0] = ADC2millimeter(median(data[0], sensor[0][0], sensor[0][1]));
//		distance[1] = ADC2millimeter(median(data[1], sensor[1][0], sensor[1][1]));		
//		distance[2] = ADC2millimeter(median(data[2], sensor[2][0], sensor[2][1]));
//		distance[3] = ADC2millimeter(median(data[3], sensor[3][0], sensor[3][1]));
//		
//		
//		// mailbox distance values 
//		sensor[0][1]=sensor[0][0]; sensor[0][0]= data[0];  
//		sensor[1][1]=sensor[1][0]; sensor[1][0]= data[1];
//		sensor[2][1]=sensor[2][0]; sensor[2][0]= data[2];
//		sensor[3][1]=sensor[3][0]; sensor[3][0]= data[3];
//		
//		ST7735_Message(0, 0, "IR 0:", distance[0]); 
//		ST7735_Message(0, 1, "IR 1:", distance[1]); 
//		ST7735_Message(0, 2, "IR 2:", distance[2]); 
//		ST7735_Message(0, 3, "IR 3:", distance[3]); 

//		SensorMSG message = {XMT_IR01_ID, distance[0], distance[1]};
//		PutCANData(&message);

//		SensorMSG message1 = {XMT_IR23_ID, distance[2], distance[3]};
//		PutCANData(&message1);
//		ST7735_Message(1, 4, "ADC DLoss:", DataLost);
//		ST7735_Message(1, 5, "Put DLoss:", PutDataLost);

//		ping = Ping2_PulseWidth;
//		// do ping message
//		SensorMSG message2 = {XMT_PING_ID, ping, 0};
//		PutCANData(&message2);
//		//put into CAN FIFO
//		ST7735_Message(1, 0, "Ping:", ping);
//		lastPing = ping; 

//	}
//}

void Processing_Init() {
	OS_InitSemaphore(&CANLock, 0);
//	IR_Init();
//	Ping2_Init();
	CAN0_Open();
	OS_AddThread(&CANReceiver, 128, 1);
//	OS_AddThread(&DAS, 128, 1);
	// add processing thread
}
