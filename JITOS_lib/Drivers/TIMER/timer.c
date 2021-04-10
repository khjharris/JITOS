/*
 * JITOS TIMER INTERFACE
 * Noah Malhi   4/10/2021
*/

#include "timer.h"


void TIM6Config (void)
{
    //Enable Timer clock
	RCC->APB1ENR |= (1<<4);  // Enable the timer6 clock

    //Set the prescalar and the ARR
	TIM6->PSC = 90-1;  //90MHz/90 = 1 MHz ~~ 1 uS delay
	TIM6->ARR = 0xffff;  //MAX ARR value

    //Enable the Timer, and wait for the update Flag to set
	TIM6->CR1 |= (1<<0); //Enable the Counter
	while (!(TIM6->SR & (1<<0)));  //Update interrupt flag
}

void Delay_us (uint16_t us)
{
	TIM6->CNT = 0;
	while (TIM6->CNT < us);
}

/* Delay call to prevent immediatiate execution */
void Delay_ms (uint16_t ms)
{
	for (uint16_t i=0; i<ms; i++)
	{
		Delay_us (1000); // delay of 1 ms
	}
}
