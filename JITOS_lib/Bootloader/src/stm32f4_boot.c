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
    rcc_apb2_register_t enable_syscfgclk = bit14_SYSCFGEN;
    if(!RCC_APB2_ENABLE(enable_syscfgclk)){
        // some error
    }

    // Configure Clocks
    if(!configure_clocks_stm32f4()){
        // some error
    }
}

// Configure clks
JITOS_STATUS configure_clocks_stm32f4(void){

    // RCC PWR Enable in APB1
    rcc_apb1_register_t enable_rccpwr = bit28_PWREN;
    if(!RCC_APB1_DISABLE(enable_rccpwr)){
        // some error
    }

    // Configure voltage scaling for main internal regulator
    if(!configure_voltage_scaling_stm32f4(power_regulator_voltage_scale3)){
        // some error
    }

    // Initialize CPU AHB and APB clocks
    /* Oscillator Configuration information:
            Using the highspeed internal
            RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
            Change HSI state to on
            RCC_OscInitStruct.HSIState = RCC_HSI_ON;
            Set the calibration value to the default calibration value
            RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
            Set the PLL state to on
            RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
            Set the pll source to the high speed 
            RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
            RCC_OscInitStruct.PLL.PLLM = 15;
            RCC_OscInitStruct.PLL.PLLN = 144;
            RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
            RCC_OscInitStruct.PLL.PLLQ = 5;
            RCC_OscInitStruct.PLL.PLLR = 2;
    */

    RCC_oscilator_init_t rcc_initialization = {0};

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

JITOS_STATUS RCC_APB2_ENABLE(rcc_apb2_register_t rcc_apb2_bit_to_set){
    volatile uint32_t read_value = 0U;

    // Set the rcc_apb2 bit passed
    SET_BIT(RCC->APB2ENR, rcc_apb2_bit_to_set);

    //Confirm that the bit is set
    read_value = READ_BIT(RCC->APB2ENR, rcc_apb2_bit_to_set);
    if(read_value){
        return STATUS_OK;
    }else{
        // input some debug info here
        return STATUS_FAIL;
    }

}

JITOS_STATUS RCC_APB2_DISABLE(rcc_apb2_register_t rcc_apb2_bit_to_clear){
    volatile uint32_t clear_value = 0U;

    // Clear the bit passed
    CLEAR_BIT(RCC->APB2ENR, rcc_apb2_bit_to_clear);

    // Confirm the bit is set
    clear_value = READ_BIT(clear_value, rcc_apb2_bit_to_clear);
    if(!clear_value){
        return STATUS_OK;
    }else{
        // input some debug info here
        return STATUS_FAIL;
    }

}

JITOS_STATUS RCC_APB2_READ(rcc_apb2_register_t rcc_apb2_bit_to_read, volatile uint32_t * read_output){
    *read_output = READ_BIT(RCC->APB2ENR, rcc_apb2_bit_to_read);

    return STATUS_OK;
}

JITOS_STATUS RCC_APB1_ENABLE(rcc_apb1_register_t rcc_apb1_bit_to_set){
    volatile uint32_t read_value = 0U;

    // Set the rcc_apb2 bit passed
    SET_BIT(RCC->APB1ENR, rcc_apb1_bit_to_set);

    //Confirm that the bit is set
    read_value = READ_BIT(RCC->APB1ENR, rcc_apb1_bit_to_set);
    if(read_value){
        return STATUS_OK;
    }else{
        // input some debug info here
        return STATUS_FAIL;
    }
}

JITOS_STATUS RCC_APB1_DISABLE(rcc_apb1_register_t rcc_apb1_bit_to_clear){
    volatile uint32_t clear_value = 0U;

    // Clear the bit passed
    CLEAR_BIT(RCC->APB1ENR, rcc_apb1_bit_to_clear);

    // Confirm the bit is set
    clear_value = READ_BIT(clear_value, rcc_apb1_bit_to_clear);
    if(!clear_value){
        return STATUS_OK;
    }else{
        // input some debug info here
        return STATUS_FAIL;
    }
}

JITOS_STATUS RCC_APB1_READ(rcc_apb1_register_t rcc_apb1_bit_to_read, volatile uint32_t * read_output){
    *read_output = READ_BIT(RCC->APB1ENR, rcc_apb1_bit_to_read);

    return STATUS_OK;
}

JITOS_STATUS configure_voltage_scaling_stm32f4(pwr_regulator_t voltage_scale){
    volatile uint32_t modify_value = 0U;

    //Modify the register using the masks provided in stm32f4xx.h
    MODIFY_REG(PWR->CR, PWR_CR_VOS, voltage_scale);
    
    //Read the bit to confirm change and insert delay
    modify_value = READ_BIT(PWR->CR,PWR_CR_VOS);

    // Compare the read value to the passed modify mask
    if(modify_value != voltage_scale){
        return STATUS_FAIL;
    }else {
        return STATUS_OK;
    }

}
