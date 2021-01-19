/*
    JITOS Bootloader target stm32f4
    Kenwood Harris 12/31/2020
*/

#include <stdint.h>
// !!!! NEED TO INCLUDE CORRECT CMSIS FILES
#include "JITOS.h"

/* Flash Defines */
/* !!! SEE MEMORY MAP FOR DETAILS */
#define FLASH_INSTRUCTION_CACHE 1
#define FLASH_DATA_CACHE 1
#define FLASH_PREFETCH 1

// 512kb of flash
#define FLASH_SIZE 0x80000

// Start address for perepherals
#define PERIPHERAL_BASE_ADDR 0x40000000UL

/* Note: AHB stands for Advanced High Performance Bus, a bus protocol by arm */
// AHB1
#define AHB1_PERIPHERAL_BASE_ADDR (PERIPHERAL_BASE_ADDR + 0x00020000UL)

// Flash is located on the AHB1 bus
#define FLASH_REGISTER_BASE_ADDR (AHB1_PERIPHERAL_BASE_ADDR + 0x3C00UL)

// Flash control registers are offset from the FLASH_REGISTER_BASE_ADDR
#define FLASH_ACCESS_CONTROL_REGISTER       (FLASH_REGISTER_BASE_ADDR + 0x00UL)     /*!< FLASH access control register,   Address offset: 0x00 */
#define FLASH_KEY_REGISTER                  (FLASH_REGISTER_BASE_ADDR + 0x04UL)     /*!< FLASH key register,              Address offset: 0x04 */
#define FLASH_OPTION_KEY_REGISTER           (FLASH_REGISTER_BASE_ADDR + 0x08UL)     /*!< FLASH option key register,       Address offset: 0x08 */
#define FLASH_STATUS_REGISTER               (FLASH_REGISTER_BASE_ADDR + 0x0CUL)     /*!< FLASH status register,           Address offset: 0x0C */
#define FLASH_CONTROL_REGISTER              (FLASH_REGISTER_BASE_ADDR + 0x10UL)     /*!< FLASH control register,          Address offset: 0x10 */
#define FLASH_OPTION_CONTROL_REGISTER       (FLASH_REGISTER_BASE_ADDR + 0x14UL)     /*!< FLASH option control register ,  Address offset: 0x14 */
#define FLASH_OPTION_CONTROL_REGISTER1      (FLASH_REGISTER_BASE_ADDR + 0x18UL)     /*!< FLASH option control register 1, Address offset: 0x18 */

/*
    From ST Datasheet:

    Instruction Cache:
    To limit the time lost due to jumps, it is possible to retain 64 lines of 128 bits in an instruction
    cache memory. This feature can be enabled by setting the instruction cache enable (ICEN)
    bit in the FLASH_ACR register. Each time a miss occurs (requested data not present in the
    currently used instruction line, in the prefetched instruction line or in the instruction cache
    memory), the line read is copied into the instruction cache memory. If some data contained
    in the instruction cache memory are requested by the CPU, they are provided without
    inserting any delay. Once all the instruction cache memory lines have been filled, the LRU
    (least recently used) policy is used to determine the line to replace in the instruction memory
    cache. This feature is particularly useful in case of code containing loops.

    Data Cache:
    Literal pools are fetched from Flash memory through the D-Code bus during the execution
    stage of the CPU pipeline. The CPU pipeline is consequently stalled until the requested
    literal pool is provided. To limit the time lost due to literal pools, accesses through the AHB
    databus D-Code have priority over accesses through the AHB instruction bus I-Code.
    If some literal pools are frequently used, the data cache memory can be enabled by setting
    the data cache enable (DCEN) bit in the FLASH_ACR register. This feature works like the
    instruction cache memory, but the retained data size is limited to 8 rows of 128 bits.

    Prefetch:
    Each Flash memory read operation provides 128 bits from either four instructions of 32 bits
    or 8 instructions of 16 bits according to the program launched. So, in case of sequential
    code, at least four CPU cycles are needed to execute the previous read instruction line.
    Prefetch on the I-Code bus can be used to read the next sequential instruction line from the
    Flash memory while the current instruction line is being requested by the CPU. Prefetch is
    enabled by setting the PRFTEN bit in the FLASH_ACR register. This feature is useful if at
    least one wait state is needed to access the Flash memory. 
*/

/*
                                                    FLASH ACCESS CONTROL REGISTER
    +-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+
    |   15  |   14  |   13  |   12  |   11  |   10  |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
    +-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+
       RES     RES     RES    DCRST   ICRST   DCEN    ICEN    PRFTEN   RES     RES     RES     RES  |----------- LATENCY -----------|
*/
#define FLASH_INSTRUCTION_CACHE_EN (0x1UL << 9U)
#define FLASH_DATA_CACHE_EN (0x1UL << 10U)
#define FLASH_PREFETCH_EN (0x1UL << 8U)

JITOS_STATUS enable_flash_instruction_cache_stm32f4(void);
JITOS_STATUS enable_flash_data_cache_stm32f4(void);
JITOS_STATUS enable_flash_prefetch_buffer_stm32f4(void);

/*  NVIC PROIRITY Defines 
    We wil setup the priority to take 4 bits and have no subpriority implemented.
    This setting is written in the PRIGROUP Field in the AIRCR (Application interrupt and reset control register) within the SCB(System Control Block).

    From Datasheet: 
    The PRIGROUP field indicates the position of the binary point that splits the PRI_n fields in
    the Interrupt Priority Registers into separate group priority and subpriority fields. 

    +-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+
    |   15  |   14  |   13  |   12  |   11  |   10  |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
    +-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+
    ENDIANESS  RES     RES     RES     RES  |------ PRIGROUP -------|  RES     RES     RES     RES     RES  SYSRESREQ 

    Bit 0 and 1 are for VECTRESET and VECTCLRACTIVE respectively, and are reserved for debug. They must be written as zeros when writing
    to this register or undefined behavior may occur. Fortunately there is a CMSIS command that handles writing to this register:
    NVIC_SetPriorityGrouping()

    You can get the current priority grouping by utilizing the CMSIS function: NVIC_GetPriorityGrouping()
*/

// Macros that allow us to quickly assert the values passed are valid (Sanity Check) From stm32f4xx_hal_cortex.h
#define IS_NVIC_PREEMPTION_PRIORITY(PRIORITY)  ((PRIORITY) < 0x10U)
#define IS_NVIC_SUB_PRIORITY(PRIORITY)         ((PRIORITY) < 0x10U)

#define NVIC_PRIORITYGROUP_0         0x00000007U /* 0 bits for pre-emption priority and 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         0x00000006U /* 1 bits for pre-emption priority and 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         0x00000005U /* 2 bits for pre-emption priority and 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         0x00000004U /* 3 bits for pre-emption priority and 1 bits for subpriority */
#define NVIC_PRIORITYGROUP_4         0x00000003U /* 4 bits for pre-emption priority and 0 bits for subpriority */

#define IS_NVIC_PRIORITY_GROUP(GROUP) (((GROUP) == NVIC_PRIORITYGROUP_0) || \
                                       ((GROUP) == NVIC_PRIORITYGROUP_1) || \
                                       ((GROUP) == NVIC_PRIORITYGROUP_2) || \
                                       ((GROUP) == NVIC_PRIORITYGROUP_3) || \
                                       ((GROUP) == NVIC_PRIORITYGROUP_4))

//  Set the NVIC priority grouping
JITOS_STATUS configure_NVIC_priority_grouping(uint32_t priority_grouping);

// Set the NVIC priority of an interrupt
JITOS_STATUS set_NVIC_priority(IRQn_Type interrupt_number, uint32_t preemption_priority, uint32_t priority_subgroup);

/* SYSTICK Defines
    We will setup the systick interface to allow us to determine timing for various applications such as scheduling and system up time.

    From Datasheet:
    The processor has a 24-bit system timer, SysTick, that counts down from the reload value to
    zero, reloads (wraps to) the value in the STK_LOAD register on the next clock edge, then
    counts down on subsequent clocks.
    
    Note: SYSTICK does not count when the processer is in sleep mode!

    Register map is not needed as CMSIS provides simple functionality for enabling and disabling and reading.
    CMSIS Function SysTick_Config() allows us to setup the interrupt period
    MATH: SYSCORECLK / X = 
*/

// Systick Interrupt priority: Should be the highest
#define SYSTICK_INTERRUPT_PRIORITY 0U;

// This is the core clock frequency of the stm32f4 target
uint32_t system_core_clock_stm32f4 = 16000000;

// Tick scalars to divide the sysclock default is 1khz
#define sysclock_tick_freq_1khz = 1U;
#define sysclock_tick_freq_100hz = 10U;
#define sysclock_tick_freq_10hz = 100U;

// sysclock scalar type to clarify in function prototype
typedef uint32_t sysclock_tick_scaler_t;

JITOS_STATUS configure_SYSTICK(uint32_t tick_priority, sysclock_tick_scaler_t clock_scaler);

/*  RCC Defines 

    RCC stands for Reset and Control Clocks

    From ST Datasheet:
    "There are three types of reset, defined as system Reset, power Reset and backup domain
    Reset."

    System Reset:
    A system reset sets all registers to their reset values except the reset flags in the clock
    controller CSR register and the registers in the Backup domain.
    A system reset is generated when one of the following events occurs:
    1. A low level on the NRST pin (external reset)
    2. Window watchdog end of count condition (WWDG reset)
    3. Independent watchdog end of count condition (IWDG reset)
    4. A software reset (SW reset) (see Software reset)
    5. Low-power management reset (see Low-power management reset)

        Software reset:
        The reset source can be identified by checking the reset flags in the RCC clock control &
        status register (RCC_CSR).
        The SYSRESETREQ bit in Cortex®-M4 with FPU Application Interrupt and Reset Control
        Register must be set to force a software reset on the device. Refer to the Cortex®-M4 with
        FPU technical reference manual for more details.

        Low-power management reset
        There are two ways of generating a low-power management reset:
        1. Reset generated when entering the Standby mode:
        This type of reset is enabled by resetting the nRST_STDBY bit in the user option bytes.
        In this case, whenever a Standby mode entry sequence is successfully executed, the
        device is reset instead of entering the Standby mode.
        2. Reset when entering the Stop mode:
        This type of reset is enabled by resetting the nRST_STOP bit in the user option bytes.
        In this case, whenever a Stop mode entry sequence

    Power Reset:
    A power reset is generated when one of the following events occurs:
    1. Power-on/power-down reset (POR/PDR reset) or brownout (BOR) reset
    2. When exiting the Standby mode
    A power reset sets all registers to their reset values except the Backup domain

    These sources act on the NRST pin and it is always kept low during the delay phase. The
    RESET service routine vector is fixed at address 0x0000_0004 in the memory map.
    The system reset signal provided to the device is output on the NRST pin. The pulse
    generator guarantees a minimum reset pulse duration of 20 µs for each internal reset
    source. In case of an external reset, the reset pulse is generated while the NRST pin is
    asserted low

    Backup Domain Reset:
    The backup domain reset sets all RTC registers and the RCC_BDCR register to their reset
    values. The BKPSRAM is not affected by this reset. The only way of resetting the
    BKPSRAM is through the Flash interface by requesting a protection level change from 1 to
    0.
    A backup domain reset is generated when one of the following events occurs:
    1. Software reset, triggered by setting the BDRST bit in the RCC Backup domain control
    register (RCC_BDCR).
    2. VDD or VBAT power on, if both supplies have previously been powered off.

    *** We will set our clock to be based on the High Speed Internal (HSI) Oscillator ***
    *** Our secondary clock source will be an external Low Speed Oscillator (LSE) ***

    AHB Speeds:
    Several prescalers are used to configure the AHB frequency, the high-speed APB (APB2)
    and the low-speed APB (APB1) domains. The maximum frequency of the AHB domain is
    180 MHz. The maximum allowed frequency of the high-speed APB2 domain is 90 MHz. The
    maximum allowed frequency of the low-speed APB1 domain is 45 MHz

    All peripheral clocks are derived from the system clock (SYSCLK) except for:
    -   The USB OTG FS clock (48 MHz), which is coming from a specific output of the PLL
        (PLLP) or PLLSAI (PLLSAIP)
    
    -   The SDIO clock (48 MHz) which is coming from a specific output of the PLL48CLK
        (PLLQ, PLLSAIP), or System Clock

    -   I2S1/2 clock for higher audio quality can be derived from four sources: specific main PLL output, a specific
        PLLI2S output, from an external clock mapped on the I2S_CKIN pin or from HSI/HSE

    -   SAIs clock, The SAI1/SAI2 clocks are generated from a specific PLL (Main PLL, PLLSAI, or
        PLLI2S), from an external clock mapped on the I2S_CKIN pin or from HSI/HSE clock

    -   The USB OTG HS (60 MHz) clock which is provided from the external PHY

    -   SPDIF-Rx clock, is generated from a specific output of PLLI2S or from a specific
        output of main PLL

    -   HDMI-CEC clock which is generated from LSE or HSI divided by 488

    -   FMPI2C1 clock which can also be generated from HSI, SYSCLK or APB1 clock

    Timer Clock Frequencies:
    The timer clock frequencies are automatically set by hardware. There are two cases
    depending on the value of TIMPRE bit in RCC_CFGR register

    1. If TIMPRE bit in RCC_DKCFGR register is reset: 
    If the APB prescaler is configured to a division factor of 1, the timer clock frequencies
    (TIMxCLK) are set to PCLKx. Otherwise, the timer clock frequencies are twice the
    frequency of the APB domain to which the timers are connected: TIMxCLK = 2xPCLKx

    2. If TIMPRE bit in RCC_DKCFGR register is set:
    If the APB prescaler is configured to a division factor of 1, 2 or 4, the timer clock
    frequencies (TIMxCLK) are set to HCLK. Otherwise, the timer clock frequencies is four
    times the frequency of the APB domain to which the timers are connected: TIMxCLK =
    4xPCLKx.

    SYSTICK:
    The RCC feeds the external clock of the Cortex System Timer (SysTick) with the AHB clock
    (HCLK) divided by 8. The SysTick can work either with this clock or with the Cortex clock
    (HCLK), configurable in the SysTick control and status register.

    PLL Configuration:
    A main PLL (PLL) clocked by the HSE or HSI oscillator and featuring three different
    output clocks:
    -   The first output is used to generate the high speed system clock (up to 180 MHz)
    -   The second output can be used to generate the clock for the USB OTG FS
        (48 MHz) or the SDIO (≤ 48 MHz)
    -   The third output can be used to generate the clock for I2S1 and I2S2 clocks,
        SPDIF-Rx clock or the high speed system clock

    Since the main-PLL configuration parameters cannot be changed once PLL is enabled, it is
    recommended to configure PLL before enabling it (selection of the HSI or HSE oscillator as
    PLL clock source, and configuration of division factors M, N, P, R and Q)

    The PLLI2S and PLLSAI use the same input clock as PLL (PLLSRC bit is common to both
    PLLs). However, the PLLI2S and PLLSAI have dedicated enable/disable and division
    factors (M, N, P, R and R) configuration bits. Once the PLLI2S and PLLSAI are enabled, the
    configuration parameters cannot be changed.

    The three PLLs are disabled by hardware when entering Stop and Standby modes, or when
    an HSE failure occurs when HSE or PLL (clocked by HSE) are used as system clock. RCC
    PLL configuration register (RCC_PLLCFGR),RCC clock configuration register
    (RCC_CFGR), and RCC dedicated clock configuration register (RCC_DCKCFGR) can be
    used to configure PLL, PLLI2S, and PLLSAI.

    After a system reset, the HSI oscillator is selected as the system clock. When a clock source
    is used directly or through PLL as the system clock, it is not possible to stop it.

    *** We will be using the internal oscillator as our only souce so we will not need to worry about
        an external PLL to lock as described in later sections ***
*/

#define RCC_BASE              (AHB1_PERIPHERAL_BASE_ADDR + 0x3800UL)

// PLL struct to store configuration information
typedef struct
{
  uint32_t PLLState;   /* The new state of the PLL. */
  uint32_t PLLSource;  /* RCC_PLLSource: PLL entry clock source. */
  uint32_t PLLM;       /* PLLM: Division factor for PLL VCO input clock. This parameter must be a number between Min_Data = 0 and Max_Data = 63 */
  uint32_t PLLN;       /* PLLN: Multiplication factor for PLL VCO output clock. This parameter must be a number between Min_Data = 50 and Max_Data = 432 */
  uint32_t PLLP;       /* PLLP: Division factor for main system clock (SYSCLK). */
  uint32_t PLLQ;       /* PLLQ: Division factor for OTG FS, SDIO and RNG clocks. This parameter must be a number between Min_Data = 2 and Max_Data = 15 */
  uint32_t PLLR;       /* PLLR: PLL division factor for I2S, SAI, SYSTEM, SPDIFRX clocks. */
} RCC_PLL_init;

// Oscillator struct to store configuration information

typedef struct
{
    uint32_t OscillatorType; /* The oscillators to be configured */
    uint32_t HSEState; /* The new state of the HSE */
    uint32_t LSEState; /* The new state of the LSE */
    uint32_t HSIState; /* The new state of the HSI */
    uint32_t HSICalibrationValue; /* The HSI calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT). This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F*/
    uint32_t LSIState; /* The new state of the LSI */
    RCC_PLL_init PLL; /* PLL structure parameters */

}RCC_oscilator_init;

// Step 1: Enable APB2 clocks
// RCC APB2 peripheral clock enable register (RCC_APB2ENR) follows the following structure
typedef enum rcc_apb2_register_enum{
    bit0_TIM1EN = 0,
    bit1_TIM8EN = 1,
    // Bits two and three are reserved
    bit4_USART1EN = 4,
    bit5_USART6EN = 5,
    // Bits six and seven are reserved
    bit8_ADC1EN = 8,
    bit9_ADC2EN = 9,
    bit10_ADC3EN = 10,
    bit11_SDIOEN = 11,
    bit12_SPI1EN = 12,
    bit13_SPI4EN = 13,
    bit14_SYSCFGEN = 14,
    // Bit fifteen is reserved
    bit16_TIM9EN = 16,
    bit17_TIM10EN = 17,
    bit18_TIM11EN = 18,
    // Bits 19-21 are reserved
    bit22_SAI1EN = 22,
    bit23_SAI2EN = 23,
} rcc_apb2_register;

#define __HAL_RCC_SYSCFG_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)

JITOS_STATUS RCC_APB2_ENABLE(rcc_apb2_register rcc_apb2_bit_to_set);
JITOS_STATUS RCC_APB2_DISABLE(rcc_apb2_register rcc_apb2_bit_to_clear);
JITOS_STATUS RCC_APB2_READ(rcc_apb2_register rcc_apb2_bit_to_read, volatile uint32_t read_output);

// Step 2: Enable APB1 Clocks
// RCC APB1 peripheral clock enable register (RCC_APB1ENR) follows the following structure
typedef enum rcc_apb1_register_enum{
    bit0_TIM2EN = 0,
    bit1_TIM3EN = 1,
    bit2_TIM4EN = 2,
    bit3_TIM5EN = 3,
    bit4_TIM6EN = 4,
    bit5_TIM7EN = 5,
    bit6_TIM12EN = 6,
    bit7_TIM13EN = 7,
    bit8_TIM14EN = 8,
    // Bits 9 and 10 are reserved 
    bit11_WWDGEN = 11,
    // Bits 12 and 13 are reserved
    bit14_SPI2EN = 14,
    bit15_SPI3EN = 15,
    bit16_SPDIFRXEN = 16,
    bit17_USART2EN = 17,
    bit18_USART3EN = 18,
    bit19_UART4EN = 19,
    bit20_UART5EN = 20,
    bit21_I2C1EN = 21,
    bit22_I2C2EN = 22,
    bit23_I2C3EN = 23,
    bit24_FMPI2C1EN = 24,
    bit25_CAN1EN = 25,
    bit26_CAN2EN = 26,
    bit27CECEN = 27,
    bit28_PWREN = 28,
    bit29_DACEN = 29,
}rcc_apb1_register;

#define NUM_RCC_APB1_CONFIG_BITS 26

JITOS_STATUS RCC_APB1_ENABLE(rcc_apb1_register rcc_apb1_bit_to_set);
JITOS_STATUS RCC_APB1_DISABLE(rcc_apb1_register rcc_apb1_bit_to_clear);
JITOS_STATUS RCC_APB1_READ(rcc_apb1_register rcc_apb1_bit_to_read, uint32_t read_output);

#define __HAL_RCC_PWR_CLK_ENABLE()     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);\
                                        UNUSED(tmpreg); \
                                          } while(0U)



// LD2 is the designated LED at PA5 active high
#define BOOTLOADER_LED_PIN "PA5"


// Prototypes
JITOS_STATUS run_bootloader_initialization_stm32f4(void);
JITOS_STATUS configure_clocks_stm32f4(void);
JITOS_STATUS enable_gpio_stm32f4(void);
JITOS_STATUS enable_usbfs_stm32f4(void);
