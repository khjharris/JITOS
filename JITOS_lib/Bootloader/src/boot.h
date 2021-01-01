/* 
    JITOS Bootloader Header
    Kenwood Harris 12/31/2020
*/

/*
    Nucleo-F446RE
    The built in bootloader is located at 0x1FFF76DE
    The ID is 0x90
    !!! Appliction note on bootloader can be found here: 
    https://www.st.com/resource/en/application_note/cd00167594-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf
*/

#include <stdint.h>
#include "JITOS.h"

// Target defines will be set in makefile
#ifdef TARGET_STM32F446RE
#include "stm32f4_boot.h"
#endif

void set_bootloader_indicator(void);
JITOS_STATUS run_bootloader_initialization(void);
void kernel_jump(void);
JITOS_STATUS halt_boot(void);
