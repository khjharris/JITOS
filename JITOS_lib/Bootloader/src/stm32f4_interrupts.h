

//  This file contains the headers of the interrupt handlers.
 

#ifndef STM32F4_INTERRUPTS_H
#define STM32F4_INTERRUPTS_H

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

#endif /* STM32F4_INTERRUPTS_H */
