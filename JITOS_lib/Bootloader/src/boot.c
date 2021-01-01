/*
    JITOS Bootloader
    Kenwood Harris 12/31/2020
*/

/* Functions of the bootloader
    1. Setup for boot into the main OS
    2. Allow for upgrade of the OS without reflashing
    Keys: Bootloader and Main Application are seperate binaries/applications
*/
#include "boot.h"

void __JITOS_BOOT_GPIO_INIT(void){

}

void set_bootloader_indicator(void){

}

JITOS_STATUS run_bootloader_initialization(void){
    #ifdef TARGET_STM32F446RE
        return run_bootloader_initialization_stm32f4();
    #endif
}

JITOS_STATUS halt_boot(void){
    // Read FW flash pin for new sw to boot
}

void main(){

    if(!run_bootloader_initialization()){
        // Error during intialization
    }

    set_bootloader_indicator();

    //Check if new FW image needs to be flashed/loaded
    if(halt_boot()){
        
    }

    kernel_jump();
}