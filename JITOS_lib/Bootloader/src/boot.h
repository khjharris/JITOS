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

// Target defines will be set in makefile
#ifdef TARGET_STM32F446RE
// LD2 is the designated LED at PA5 active high
#define BOOTLOADER_LED_PIN "PA5"
// 512kb of flash
#define FLASH_SIZE 0x80000
#endif

void set_bootloader_indicator(void);
uint8_t run_bootloader_initialization(void);
void kernel_jump(void);
uint8_t halt_boot(void);
