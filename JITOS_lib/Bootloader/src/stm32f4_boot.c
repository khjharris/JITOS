/*
    JITOS Bootloader target stm32f4
    Kenwood Harris 12/31/2020
*/

#include "stm32f4_boot.h"

JITOS_STATUS run_bootloader_initialization_stm32f4(void){

    if(!enable_flash_instruction_cache_stm32f4()){
        // Some error
    }

    if(!enable_data_cache_stm32f4()){
        // Some error
    }

    if(!enable_flash_prefetch_buffer()){
        // Some error
    }

    // Perhaps set some priority?

    // Init systick

    // Enable rcc sysconfig clk

    // rcc pwr clk enable

}

JITOS_STATUS configure_clocks_stm32f4(void){
    
}

JITOS_STATUS enable_gpio_stm32f4(void){

}

JITOS_STATUS enable_usbfs_stm32f4(void){

}

JITOS_STATUS enable_flash_instruction_cache_stm32f4(void){
    SET_BIT(FLASH_ACCESS_CONTROL_REGISTER,FLASH_INSTRUCTION_CACHE_EN);
}

JITOS_STATUS enable_flash_data_cache_stm32f4(void){
    SET_BIT(FLASH_ACCESS_CONTROL_REGISTER,FLASH_DATA_CACHE_EN);
}

JITOS_STATUS enable_flash_prefetch_buffer(void){
    SET_BIT(FLASH_ACCESS_CONTROL_REGISTER,FLASH_PREFETCH_EN);
}

