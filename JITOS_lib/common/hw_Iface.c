//This file contains the interfacing for the supported peripherals
//
//
//Noah Malhi  1/2/2021

//Includes
#include  "hw_Iface.h"

// Static function defines
static void general_timer_callback();
static void pwm_timer_callback();

// Local variables
uint8_t timeout = 0;


/* PUBLIC FUNCTIONS *****************************************************************************************************/
void init_general_timer()
{

    /*Counter Register = CNT
     * Presscaler Register = PSC
     * AUTO_reload Register = ARR
    */

    //Enable timer
    RCC->APB1ENR |= (1<<8)      //Enamble tim14 clock --> This step may need to be added in the target folder config
    
    //Set the Prescaler
    TIM14->PSC = 90-1;          //TODO:: change to correct prescaler value when rcc config is commplete

    //Set ARR value
    TIM14->ARR = 0xffff;        //Giving it the maximum array value but may change, depending on what we want to count to

    //Enable the counter
    TIM14->CR1 != (1<<0);

    //Enable status register
    //TODO: May want to init timer for compare and capture mode CC1OF 
    while( !( TIM14->SR & (1<<0)));  //Update interupt flag

    
}

void init_pwm_timer()
{
    //Enable timer

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

/*PRIVATE FUNCTIONS ************************************************************************************************************/
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
