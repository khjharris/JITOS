//This file contains the interfacing for the supported peripherals
//
//
//
//
//Noah Malhi  1/2/2021

//Includes
#include  "hw_Iface.h"

// Local variables
uint8_t timeout = 0;


static void general_timer_callback()
{
    if( TIM->SR & 0x1 )
    {
        timeout = 1;
        GEN_TIM->SR &= ~( UPDATE_INTERRUPT_FLAG );  //Clear SR register
        GEN_TIM->CR1 |= (~((U16)TIM_CR1_EN));  //Disable the timer
        GEN_TIM->CNT = 0x1; //Reset the timer
    }

}

static void pwm_timer_callback()
{

}

void start_general_timer( n )
{
    GEN_TIM->CR1 |= TIM_CR1_EN;  //Enable the timer
}

void start_pwm_timer()
{
    //TODO: This function will start tim1(advanced timer) for pwm mode
}

void read_general_timer()
{
    return GEN_TIM->CNT;
}

void stop_general_timer() 
{
  GEN_TIM->CR1 |= (~((U16) TIM_CR1_EN));
}
