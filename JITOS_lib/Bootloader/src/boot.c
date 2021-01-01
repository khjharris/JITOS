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

uint8_t run_bootloader_initialization(void){


}

uint8_t halt_boot(void){

}

void main(){
    set_bootloader_indicator();

    //Check if new FW image needs to be flashed/loaded
    if(halt_boot()){
        
    }

    if(run_bootloader_initialization()){

        kernel_jump();

    } else{

        //Set some error
    }

}