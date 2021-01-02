/*
    JITOS contains common functions used across all targets
    Kenwood Harris 12/31/2020
*/

#include "JITOS.h"


/* This function is called on an assert failure and will reslt in handling of the failure */
static void assert_failure_handler( char *file, int line )
{
    while( 1 )
    {
        //For now do nothing but in future be able to reset from working state
    }

}

void ITM_send( uint8_t ch )
{

	//Enable TRCENA
	DEMCR |= ( 1 << 24);

	//enable stimulus port 0
	ITM_TRACE_EN |= ( 1 << 0);

	// read FIFO status in bit [0]:
	while(!(ITM_STIMULUS_PORT0 & 1));

	//Write to ITM stimulus port0
	ITM_STIMULUS_PORT0 = ch;
}
