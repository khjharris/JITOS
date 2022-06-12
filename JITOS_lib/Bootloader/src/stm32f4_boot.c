/*
    JITOS Bootloader target stm32f4
    Kenwood Harris 12/31/2020
*/

#include "stm32f4_boot.h"

/* */
JITOS_STATUS run_bootloader_initialization_stm32f4(void){

    // Enable flash instruction cache if set
    if(FLASH_INSTRUCTION_CACHE){
        if(!enable_flash_instruction_cache_stm32f4()){
            // Some error
            return STATUS_FAIL;
        }
    }

    // Enable data cache if set
    if(FLASH_DATA_CACHE){
        if(!enable_flash_data_cache_stm32f4()){
            // Some error
            return STATUS_FAIL;
        }
    }

    // Enable prefetch if set
    if(FLASH_PREFETCH){
        if(!enable_flash_prefetch_buffer()){
            // Some error
            return STATUS_FAIL;
        }
    }

    // Configure priority grouping
    if(!configure_NVIC_priority_grouping(NVIC_PRIORITYGROUP_4)){
        // Some error
        return STATUS_FAIL;
    }

    // Init systick
    if(!configure_SYSTICK(SYSTICK_INTERRUPT_PRIORITY, sysclock_tick_freq_1khz)){
        // Some error
        return STATUS_FAIL;
    }

    // RCC SYSCFGCLK Enable in APB2
    rcc_apb2_register_t enable_syscfgclk = bit14_SYSCFGEN;
    if(!RCC_APB2_ENABLE(enable_syscfgclk)){
        // some error
        return STATUS_FAIL;
    }

    // Configure Clocks
    if(!configure_clocks_stm32f4()){
        // some error
        return STATUS_FAIL;
    }
}

// Configure clks
/* */
JITOS_STATUS configure_clocks_stm32f4(void){
    JITOS_STATUS clock_config_result, oscillator_config_result;
    RCC_oscilator_init_t rcc_osc_init = {0};
    RCC_clock_init_t rcc_clk_init = {0};

    // RCC PWR Enable in APB1
    rcc_apb1_register_t enable_rccpwr = bit28_PWREN;
    if(!RCC_APB1_DISABLE(enable_rccpwr)){
        // some error
        return STATUS_FAIL;
    }

    // Configure voltage scaling for main internal regulator
    if(!configure_voltage_scaling_stm32f4(power_regulator_voltage_scale3)){
        // some error
        return STATUS_FAIL;
    }

    rcc_osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    rcc_osc_init.HSIState = RCC_HSI_ON;
    rcc_osc_init.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    rcc_osc_init.PLL.PLLState = RCC_PLL_ON;
    rcc_osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    // M Scale Factor
    rcc_osc_init.PLL.PLLM = 16;
    // N Sacle Factor
    rcc_osc_init.PLL.PLLN = 336;
    // P Scale Factor
    rcc_osc_init.PLL.PLLP = RCC_PLLP_DIV4;
    // Q Scale Factor
    rcc_osc_init.PLL.PLLQ = 2;
    // R Scale Factor
    rcc_osc_init.PLL.PLLR = 2;

    // Initialze RCC Oscillators
    oscillator_config_result = RCC_CONFIGURE_OSCILLATOR(&rcc_osc_init);
    if(oscillator_config_result == STATUS_TIMEOUT){
        panic("Timeout occured on oscillator configuration");
    } else if(oscillator_config_result == STATUS_FAIL){
        panic("Error occured during osicllator configuration");
    }

    rcc_clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    rcc_clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
    rcc_clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
    rcc_clk_init.APB2CLKDivider = RCC_HCLK_DIV1;

    // Initialize CPU, AHB, and APB bus clocks
    clock_config_result = RCC_CONFIGURE_CLOCK(rcc_clk_init);
    if(clock_config_result == STATUS_TIMEOUT){
        panic("Timeout occured on clock configuration");
    } else if(clock_config_result == STATUS_FAIL){
        panic("Error occured during clock configuration");
    }


}

/* */
JITOS_STATUS enable_gpio_stm32f4(void){

}

/* */
JITOS_STATUS enable_usbfs_stm32f4(void){

}

/* */
JITOS_STATUS enable_flash_instruction_cache_stm32f4(void){
    SET_BIT(FLASH_ACCESS_CONTROL_REGISTER,FLASH_INSTRUCTION_CACHE_EN);

    // Do read to verify write
}

/* */
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

// Keep short called in systick handler
JITOS_STATUS increment_systick(void){

    systick_counter = systick_counter + 1;
    return STATUS_OK;
}

// Returns the current systick counter
uint32_t get_systick(void){

    return systick_counter;
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

JITOS_STATUS RCC_HSI_ENABLE(void){
    // The registers used to enable HSI are mapped in the bit-band region and we'll use them avoid read modify write's and possible errors resulting from that process (bit-band reads and writes are atomic)

    *(volatile uint32_t *) RCC_CR_HSION_BIT_BAND = ENABLE;

    return STATUS_OK;
}

JITOS_STATUS RCC_HSI_DISABLE(void){

    *(volatile uint32_t *) RCC_CR_HSION_BIT_BAND = DISABLE;

    return STATUS_OK;
}

JITOS_STATUS RCC_PLL_ENABLE(void){
    // The registers used to enable PLL are mapped in the bit-band region and we'll use them avoid read modify write's and possible errors resulting from that process (bit-band reads and writes are atomic)

    *(volatile uint32_t *) RCC_CR_PLLON_BIT_BAND = ENABLE;

    return STATUS_OK;
}

JITOS_STATUS RCC_PLL_DISABLE(void){

    *(volatile uint32_t *) RCC_CR_PLLON_BIT_BAND = DISABLE;

    return STATUS_OK;
}

uint32_t GET_FLASH_LATENCY(void){

    return (READ_BIT((FLASH->ACR), FLASH_ACR_LATENCY));
}

// Will only configure HSI Oscilator types for now
JITOS_STATUS RCC_CONFIGURE_OSCILLATOR(RCC_oscilator_init_t *oscillator_init_struct){
    uint32_t tickstart, pll_config;

    if(oscillator_init_struct == NULL){
        // Some debug info
        return STATUS_FAIL;
    }

    // HSE Configuration
    if(oscillator_init_struct->OscillatorType == RCC_OSCILLATORTYPE_HSE){
        // Not currently enabled panic here
        return STATUS_FAIL;
    }

    // HSI Configuration
    if(oscillator_init_struct->OscillatorType == RCC_OSCILLATORTYPE_HSI){
        
        // Verify needed paramaters are not null (In the future could check for valid entries)
        assert(oscillator_init_struct->HSIState);
        assert(oscillator_init_struct->HSICalibrationValue);

        // See if HSI is currently used as system clock or as a PLL source (if PLL is selected as the system clock)
        if( (RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_HSI) ||\                                                                                // If HSI is the current Source
            ((RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSI)) ||\        // Bit 22 of PLLCFG reg holds PLLSRC (HSI or HSE)
            ((RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLRCLK) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSI))){         // Same for PLLR
            
            // IF HSI is being used as a system clock it will not be disabled
            if((RCC_GET_FLAG(RCC_FLAG_HSIRDY) != RESET) && (oscillator_init_struct->HSIState != RCC_HSI_ON)){
                return STATUS_FAIL;
            } else {
                // Only calibration is allowed for these settings
                RCC_HSI_CALIBRATIONVALUE_ADJUST(oscillator_init_struct->HSICalibrationValue);
            }
        } else{

            // Check the requested HSI State
            if(oscillator_init_struct->HSIState != RCC_HSI_OFF){

                // Enable the high speed internal oscillator
                RCC_HSI_ENABLE();

                // Get tick for timeout
                tickstart = get_systick();

                // Wait for HSI to be ready
                while(RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET) {

                    if (get_systick() - tickstart > HSI_CLOCK_CONFIG_TIMEOUT){
                        return STATUS_TIMEOUT;
                    }
                }

                // Adjust the the calibration value
                RCC_HSI_CALIBRATIONVALUE_ADJUST(oscillator_init_struct->HSICalibrationValue);

            } else {
                // Disable HSI
                RCC_HSI_DISABLE();

                // Get tick for timeout
                tickstart = get_systick();
                    
                // Wait for HSI to be ready
                while(RCC_GET_FLAG(RCC_FLAG_HSIRDY) != RESET){
                    if(get_systick() - tickstart > HSI_CLOCK_CONFIG_TIMEOUT) {
                        return STATUS_TIMEOUT;
                    }
                }
            }
        }
    }

    // LSE Configuration
    if(oscillator_init_struct->OscillatorType == RCC_OSCILLATORTYPE_LSE){
        // Not currently enabled panic here
        return STATUS_FAIL;
    }

    // LSI Configuration
    if(oscillator_init_struct->OscillatorType == RCC_OSCILLATORTYPE_LSI){
        // Not currently enabled panic here
        return STATUS_FAIL;
    }

    // PLL Configuration
    if(oscillator_init_struct->PLL.PLLState != RCC_PLL_NONE){

        // Check if PLL is used as the system clock
        if(RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_PLL){

            // If request is to turn PLL ON
            if(oscillator_init_struct->PLL.PLLState == RCC_PLL_ON){
                
                // Disable the main PLL
                RCC_PLL_DISABLE();

                // Get tick for timeout
                tickstart = get_systick();

                while(RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET){
                    if(get_systick() - tickstart > HSI_PLL_CONFIG_TIMEOUT){
                        return STATUS_TIMEOUT;
                    }
                }

                // Configure Division factors
                WRITE_REG(RCC->PLLCFGR, (oscillator_init_struct->PLL.PLLSource                                    | \
                                 oscillator_init_struct->PLL.PLLM                                                 | \
                                 (oscillator_init_struct->PLL.PLLN << RCC_PLLCFGR_PLLN_Pos)                       | \
                                 (((oscillator_init_struct->PLL.PLLP >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos)        | \
                                 (oscillator_init_struct->PLL.PLLQ << RCC_PLLCFGR_PLLQ_Pos)                       | \
                                 (oscillator_init_struct->PLL.PLLR << RCC_PLLCFGR_PLLR_Pos)));

                // Enable PLL
                RCC_PLL_ENABLE();

                // Wait untill PLL is stable
                tickstart = get_systick();
                while(RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET){
                    if(get_systick() - tickstart > HSI_PLL_CONFIG_TIMEOUT){
                        return STATUS_TIMEOUT;
                    }
                
                }
            
            // If the request is to turn PLL OFF
            } else{

                RCC_PLL_DISABLE();
                tickstart = get_systick();

                // Wait until PLL Ready
                while(RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET){
                    if(get_systick() - tickstart > HSI_PLL_CONFIG_TIMEOUT){
                        return STATUS_TIMEOUT;
                    }
                }
            }

        } else{

            // If the request is to disable the PLL and it's curerntly used as the system clock this is an invalid request
            if(oscillator_init_struct->PLL.PLLState == RCC_PLL_OFF){
                return STATUS_FAIL;

            } else{
                // Check if the passed config is any different than the current configuration, if so return error
                pll_config = oscillator_init_struct->PLLCFGR;
                if (((oscillator_init_struct->PLL.PLLState) == RCC_PLL_OFF) ||
                    (READ_BIT(pll_config, RCC_PLLCFGR_PLLSRC) != oscillator_init_struct->PLL.PLLSource) ||
                    (READ_BIT(pll_config, RCC_PLLCFGR_PLLM) != (oscillator_init_struct->PLL.PLLM) << RCC_PLLCFGR_PLLM_Pos) ||
                    (READ_BIT(pll_config, RCC_PLLCFGR_PLLN) != (oscillator_init_struct->PLL.PLLN) << RCC_PLLCFGR_PLLN_Pos) ||
                    (READ_BIT(pll_config, RCC_PLLCFGR_PLLP) != (((oscillator_init_struct->PLL.PLLP >> 1U) - 1U)) << RCC_PLLCFGR_PLLP_Pos) ||
                    (READ_BIT(pll_config, RCC_PLLCFGR_PLLQ) != (oscillator_init_struct->PLL.PLLQ << RCC_PLLCFGR_PLLQ_Pos)) ||
                    (READ_BIT(pll_config, RCC_PLLCFGR_PLLR) != (oscillator_init_struct->PLL.PLLR << RCC_PLLCFGR_PLLR_Pos))) {
                        
                    return STATUS_FAIL;
                }

            }
            
        } 
    }

    return STATUS_OK;
}

JITOS_STATUS RCC_CONFIGURE_CLOCK(RCC_clock_init_t *clock_init_struct, uint32_t flash_latency){

    uint32_t tickstart;

    if(clock_init_struct == NULL){
        return STATUS_FAIL;
    }

    // Note from ST regarding flash memory latency 
    /* To correctly read data from FLASH memory, the number of wait states (LATENCY)
    must be correctly programmed according to the frequency of the CPU clock
    (HCLK) and the supply voltage of the device. */

    if(flash_latency > GET_FLASH_LATENCY()){
        
    }
}