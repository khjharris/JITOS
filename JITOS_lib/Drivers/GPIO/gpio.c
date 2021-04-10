/*
    JITOS GPIO Drivers
    Kenwood Harris 12/31/2020
*/

#ifdef TARGET_STM32F446RE

void GPIOConfig (void)
{
  /* Configuration for on-board Green Led (PA5) */
  //Enable GPIOA clock
  RCC->AHB1ENR |= (1<<0);  // Enable the GPIOA clock
	
  //Set the PIN PA5 as output
  GPIOA->MODER |= (1<<10);  // pin PA5(bits 11:10) as Output (01)
	
  //Configure the output mode
  GPIOA->OTYPER &= ~(1<<5);  // bit 5=0 --> Output push pull
  GPIOA->OSPEEDR |= (1<<11);  // Pin PA5 (bits 11:10) as Fast Speed (1:0)
  GPIOA->PUPDR &= ~((1<<10) | (1<<11));  // Pin PA5 (bits 11:10) are 0:0 --> no pull up or pulldown
}

/* Set the GPIO pin output to 1 */
void setGpioOutput( uint32_t gpioPort, uint8_t gpioPinNumber )
{
  gpioPort->BSRR |= (1 << gpioPinNumber);
}

/* Clear the GPIO pin output and set to 0 */
void clearGpioOutput( uint32_t gpioPort, uint8_t gpioPinNumber )
{
  gpioPort->BSRR |= (1 << gpioPinNumber) <<16;
}

#endif
