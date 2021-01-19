/*
    JITOS Bootloader target stm32f4
    Kenwood Harris 12/31/2020
*/

#include "stm32f4_boot.h"

JITOS_STATUS run_bootloader_initialization_stm32f4(void){

    // Enable flash instruction cache if set
    if(FLASH_INSTRUCTION_CACHE){
        if(!enable_flash_instruction_cache_stm32f4()){
            // Some error
        }
    }

    // Enable data cache if set
    if(FLASH_DATA_CACHE){
        if(!enable_flash_data_cache_stm32f4()){
            // Some error
        }
    }

    // Enable prefetch if set
    if(FLASH_PREFETCH){
        if(!enable_flash_prefetch_buffer()){
            // Some error
        }
    }

    // Configure priority grouping
    if(!configure_NVIC_priority_grouping(NVIC_PRIORITYGROUP_4)){
        // Some error
    }

    // Init systick
    if(!configure_SYSTICK(SYSTICK_INTERRUPT_PRIORITY, sysclock_tick_freq_1khz)){
        // Some error
    }

    // RCC SYSCFGCLK Enable in APB2
    // Create rcc_apb2_config structure
    RCC_APB2_config syscgcclk_enable_config;
    syscgcclk_enable_config.bit14_SYSCFGEN = 0x1U;
    RCC_APB2_ENABLE(syscgcclk_enable_config);





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

    // Do read to verify write
}

JITOS_STATUS enable_flash_data_cache_stm32f4(void){
    SET_BIT(FLASH_ACCESS_CONTROL_REGISTER,FLASH_DATA_CACHE_EN);

    // Do read to verify write
}

JITOS_STATUS enable_flash_prefetch_buffer_stm32f4(void){
    SET_BIT(FLASH_ACCESS_CONTROL_REGISTER,FLASH_PREFETCH_EN);

    // Do read to verify write
}

JITOS_STATUS configure_NVIC_priority_grouping(uint32_t priority_grouping){
    // Ensure priority is valid
    if(!IS_NVIC_PRIORITY_GROUP(priority_grouping)){
        // Input some debug info here
        return STATUS_FAIL;
    }

    // Currently set no priority subgroups
    NVIC_SetPriorityGrouping(priority_grouping);

    return STATUS_OK;
}

JITOS_STATUS set_NVIC_priority(IRQn_Type interrupt_number, uint32_t preemption_priority, uint32_t priority_subgroup){
    // Ensure that the priorities passed are valid
    if(!IS_NVIC_PREEMPTION_PRIORITY(preemption_priority)){
        // Input debug info here
        return STATUS_FAIL;
    }

    if(!IS_NVIC_SUB_PRIORITY(priority_subgroup)){
        // Input debug info here
        return STATUS_FAIL;
    }

    // Retrieve the current NVIC priority grouping
    uint32_t current_nvic_priority_grouping = NVIC_GetPriorityGrouping();

    // Set the priority using the current priority grouping and wants
    NVIC_SetPriority(interrupt_number, NVIC_EncodePriority(current_nvic_priority_grouping, preemption_priority, priority_subgroup));

    return STATUS_OK;
}

JITOS_STATUS configure_SYSTICK(uint32_t tick_priority, sysclock_tick_scaler_t clock_scaler){
    // Ensure tha the priority sent is a valid priority
    if(!IS_NVIC_PREEMPTION_PRIORITY(tick_priority)){
        // Inupt some debug info here
        return STATUS_FAIL;
    }

    // Configure systick with CMSIS function SysTick_Config()
    if(SysTick_Config(system_core_clock_stm32f4 / (1000U / clock_scaler))){
        // Input some debug info here
        return STATUS_FAIL;
    }

    // Ensure that systick has the highest priority grouping, despite our default wanting no sub priority (no need to assume)
    uint32_t systick_priority_grouping = NVIC_GetPriorityGrouping();

    // Set the tick interrupt priority, SysTick_IRQn is defined in the CMSIS files, no subpriority
    if(set_NVIC_priority(SysTick_IRQn, tick_priority, 0) == STATUS_FAIL){
        // Input some debug info here
        return STATUS_FAIL;
    }

    return STATUS_OK;
}

JITOS_STATUS RCC_APB2_ENABLE(rcc_apb2_register rcc_apb2_bit_to_set){
    volatile uint32_t read_value = 0U;

    // Set the rcc_apb2 bit passed
    SET_BIT(RCC->APB2ENR, rcc_apb2_bit_to_set);

    //Confirm that the bit is set
    if(READ_BIT(read_value, rcc_apb2_bit_to_set)){
        return STATUS_OK;
    }else{
        // input some debug info here
        return STATUS_FAIL;
    }

}

JITOS_STATUS RCC_APB2_DISABLE(rcc_apb2_register rcc_apb2_bit_to_clear){
    volatile uint32_t clear_value = 0U;

    CLEAR_BIT(RCC->APB2ENR, rcc_apb2_bit_to_clear);

    if(READ_BIT(clear_value, rcc_apb2_bit_to_clear)){
        return STATUS_OK;
    }else{
        // input some debug info here
        return STATUS_FAIL;
    }

}

JITOS_STATUS RCC_APB2_READ(rcc_apb2_register rcc_apb2_bit_to_read, volatile uint32_t read_output){

}

JITOS_STATUS RCC_APB1_ENABLE(RCC_APB1_config rcc_apb1_config_struct){

}
