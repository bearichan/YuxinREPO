#include <stdint.h>

void Timer0A_Init(void(*task)(void), uint32_t period);
void Timer1A_Init(void(*task)(void), uint32_t period);
void Timer2A_Init(void(*task)(void), uint32_t period);
void Timer3A_Init(uint32_t period);