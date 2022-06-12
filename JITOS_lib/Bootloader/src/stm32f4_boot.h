/*
    JITOS Bootloader target stm32f4
    Kenwood Harris 12/31/2020
*/

#include <stdint.h>
#include 
// !!!! NEED TO INCLUDE CORRECT CMSIS FILES
#include "JITOS.h"

// HW Definition files provided by Silicon producer
#include "stm32f4xx.h"
#include "stm32f446xx.h"

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

// Variable that stores the incremented systick value from the systick interrupt
volatile uint32_t systick_counter;

// Tick scalars to divide the sysclock default is 1khz
#define sysclock_tick_freq_1khz = 1U;
#define sysclock_tick_freq_100hz = 10U;
#define sysclock_tick_freq_10hz = 100U;

// sysclock scalar type to clarify in function prototype
typedef uint32_t sysclock_tick_scaler_t;

JITOS_STATUS configure_SYSTICK(uint32_t tick_priority, sysclock_tick_scaler_t clock_scaler);


// Used to get the RCC OFFSET from the Peripheral base address (Useful for bitband operations which are offset from Peripheral BB Base)
#define RCC_OFFSET                 (RCC_BASE - PERIPH_BASE)

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


// RCC_BASE is defined as ->  RCC_BASE              (AHB1_PERIPHERAL_BASE_ADDR + 0x3800UL)

// The following describes the Reset and Clock Control Registers (RCC):

/* RCC Clock Control Register (RCC_CR)
    +-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+
    |   31  |   30  |   29  |   28  |   27  |   26  |   25  |   24  |   23  |   22  |   21  |   20  |   19  |   18  |   17  |   16  |
    +-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+
       RES     RES   PLLSAI  PLLSAI  PLLI2S  PLLI2S   PLL     PLL     RES      RES     RES      RES    CSS     HSE     HSE     HSE
                       RDY     ON     RDY      ON     RDY     ON                                       ON      BPY     RDY     ON

    +-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+
    |   15  |   14  |   13  |   12  |   11  |   10  |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
    +-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+
    |------------------------------ HSICAL -------------------------|---------------- HISTRIM --------------|  RES     HSI     HSI
                                                                                                                       RDY     ON

    Bits 31:28 Reserved, must be kept at reset value.

    Bit 29 PLLSAIRDY: PLLSAI clock ready flag
        Set by hardware to indicate that the PLLSAI is locked.
        0: PLLSAI unlocked
        1: PLLSAI locked

    Bit 28 PLLSAION: PLLSAI enable
        Set and cleared by software to enable PLLSAI.
        Cleared by hardware when entering Stop or Standby mode.
        0: PLLSAI OFF
        1: PLLSAI ON

    Bit 27 PLLI2SRDY: PLLI2S clock ready flag
        Set by hardware to indicate that the PLLI2S is locked.
        0: PLLI2S unlocked
        1: PLLI2S locked

    Bit 26 PLLI2SON: PLLI2S enable
        Set and cleared by software to enable PLLI2S.
        Cleared by hardware when entering Stop or Standby mode.
        0: PLLI2S OFF
        1: PLLI2S ON

    Bit 25 PLLRDY: Main PLL (PLL) clock ready flag
        Set by hardware to indicate that PLL is locked.
        0: PLL unlocked
        1: PLL locked

    Bit 24 PLLON: Main PLL (PLL) enable
        Set and cleared by software to enable PLL.
        Cleared by hardware when entering Stop or Standby mode. This bit cannot be reset if PLL
        clock is used as the system clock.
        0: PLL OFF
        1: PLL ON

    Bits 23:20 Reserved, must be kept at reset value

    Bit 19 CSSON: Clock security system enable
        Set and cleared by software to enable the clock security system. When CSSON is set, the
        clock detector is enabled by hardware when the HSE oscillator is ready, and disabled by
        hardware if an oscillator failure is detected.
        0: Clock security system OFF (Clock detector OFF)
        1: Clock security system ON (Clock detector ON if HSE oscillator is stable, OFF if not)

    Bit 18 HSEBYP: HSE clock bypass
        Set and cleared by software to bypass the oscillator with an external clock. The external
        clock must be enabled with the HSEON bit, to be used by the device.
        The HSEBYP bit can be written only if the HSE oscillator is disabled.
        0: HSE oscillator not bypassed
        1: HSE oscillator bypassed with an external clock

    Bit 17 HSERDY: HSE clock ready flag
        Set by hardware to indicate that the HSE oscillator is stable. After the HSEON bit is cleared,
        HSERDY goes low after 6 HSE oscillator clock cycles.
        0: HSE oscillator not ready
        1: HSE oscillator ready

    Bit 16 HSEON: HSE clock enable
        Set and cleared by software.
        Cleared by hardware to stop the HSE oscillator when entering Stop or Standby mode. This
        bit cannot be reset if the HSE oscillator is used directly or indirectly as the system clock.
        0: HSE oscillator OFF
        1: HSE oscillator ON

    Bits 15:8 HSICAL[7:0]: Internal high-speed clock calibration
        These bits are initialized automatically at startup.

    Bits 7:3 HSITRIM[4:0]: Internal high-speed clock trimming
        These bits provide an additional user-programmable trimming value that is added to the
        HSICAL[7:0] bits. It can be programmed to adjust to variations in voltage and temperature
        that influence the frequency of the internal HSI RC.

    Bit 2 Reserved, must be kept at reset value.

    Bit 1 HSIRDY: Internal high-speed clock ready flag
        Set by hardware to indicate that the HSI oscillator is stable. After the HSION bit is cleared,
        HSIRDY goes low after 6 HSI clock cycles.
        0: HSI oscillator not ready
        1: HSI oscillator ready

    Bit 0 HSION: Internal high-speed clock enable
        Set and cleared by software.
        Set by hardware to force the HSI oscillator ON when leaving the Stop or Standby mode or in
        case of a failure of the HSE oscillator used directly or indirectly as the system clock. This bit
        cannot be cleared if the HSI is used directly or indirectly as the system clock.
        0: HSI oscillator OFF
        1: HSI oscillator ON
*/

/* Flags in the CR register */
#define RCC_FLAG_HSIRDY                  ((uint8_t)0x21)
#define RCC_FLAG_HSERDY                  ((uint8_t)0x31)
#define RCC_FLAG_PLLRDY                  ((uint8_t)0x39)
#define RCC_FLAG_PLLI2SRDY               ((uint8_t)0x3B)

// The HSI Calibration value must be a number between 0 and 0x1F
#define RCC_HSI_CALIBRATIONVALUE_ADJUST(__HSICalibrationValue__) (MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, (uint32_t)(__HSICalibrationValue__) << RCC_CR_HSITRIM_Pos))

// Bit-band address' of the above RCC register bits
// CR is the first 32bits from RCC base so no need to offset
#define RCC_CR_HSION_BIT_BAND   (PERIPH_BB_BASE + (RCC_OFFSET * 32U))
// PLLON is bit 32 from RCC_CR(RCC_CR is 0 offset from RCC_OFFSET)
#define RCC_CR_PLLON_BIT_BAND   (PERIPH_BB_BASE + (RCC_OFFSET * 32U) + (0x18 * 4U))

#define HSI_CLOCK_CONFIG_TIMEOUT 10U
#define HSI_PLL_CONFIG_TIMEOUT 10U

JITOS_STATUS RCC_HSI_ENABLE(void);
JITOS_STATUS RCC_HSI_DISABLE(void);
JITOS_STATUS RCC_PLL_ENABLE(void);
JITOS_STATUS RCC_PLL_DISABLE(void);


/*  
    RCC PLL configuration register (RCC_PLLCFGR):

   (Definition: VCO - Voltage Controlled Oscillator)
    Address offset: 0x04
    Reset value: 0x2400 3010
    Access: no wait state, word, half-word and byte access.
    This register is used to configure the PLL clock outputs according to the formulas:
    - f(VCO clock) = f(PLL clock input) × (PLLN / PLLM)
    - f(PLL general clock output) = f(VCO clock) / PLLP
    - f(USB OTG FS, SDIO) = f(VCO clock) / PLLQ
    +-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+
    |   31  |   30  |   29  |   28  |   27  |   26  |   25  |   24  |   23  |   22  |   21  |   20  |   19  |   18  |   17  |   16  |
    +-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+
       RES  |------ PLLR[2:0] ------|---------- PLLQ[3:0] ----------|  RES    PLLSRC   RES     RES     RES     RES  |-- PLLP[1:0] --|

    +-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+
    |   15  |   14  |   13  |   12  |   11  |   10  |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
    +-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+
       RES  |------------------------------- PLLN[8:0] -----------------------------|----------------- PLLM[5:0] -------------------|
    
    PLLM[5:0]: Division factor for the main PLL (PLL) input clock
    Set and cleared by software to divide the PLL input clock before the VCO. These bits can be
    written only when the PLL is disabled.
    Caution: The software has to set these bits correctly to ensure that the VCO input frequency
    ranges from 1 to 2 MHz. It is recommended to select a frequency of 2 MHz to limit
    PLL jitter.
    VCO input frequency = PLL input clock frequency / PLLM with 2 ≤ PLLM ≤ 63
    000000: PLLM = 0, wrong configuration
    000001: PLLM = 1, wrong configuration
    000010: PLLM = 2    
    000011: PLLM = 3
    000100: PLLM = 4
    ...
    111110: PLLM = 62
    111111: PLLM = 63

    PLLN[8:0]: Main PLL (PLL) multiplication factor for VCO
    Set and cleared by software to control the multiplication factor of the VCO. These bits can
    be written only when PLL is disabled. Only half-word and word accesses are allowed to
    write these bits.
    Caution: The software has to set these bits correctly to ensure that the VCO output
    frequency is between 100 and 432 MHz.
    VCO output frequency = VCO input frequency × PLLN with 50 ≤ PLLN ≤ 432
    000000000: PLLN = 0, wrong configuration
    000000001: PLLN = 1, wrong configuration ...
    000110010: PLLN = 50
    ...
    001100011: PLLN = 99
    ...
    110110000: PLLN = 432
    110110001: PLLN = 433, wrong configuration ...
    111111111: PLLN = 511, wrong configuration
    Note: Between 50 and 99 multiplication factors are possible for VCO input frequency higher
    than 1 MHz. However care must be taken to fulfill the minimum VCO output frequency
    as specified above.

    PLLP[1:0]: Main PLL (PLL) division factor for main system clock
    Set and cleared by software to control the frequency of the general PLL output clock. These
    bits can be written only if PLL is disabled.
    Caution: The software has to set these bits correctly not to exceed 180 MHz on this domain.
    PLL output clock frequency = VCO frequency / PLLP with PLLP = 2, 4, 6, or 8
    00: PLLP = 2
    01: PLLP = 4
    10: PLLP = 6
    11: PLLP = 8

    PLLSRC: Main PLL(PLL) and audio PLL (PLLI2S) entry clock source
    Set and cleared by software to select PLL and PLLI2S clock source. This bit can be written
    only when PLL and PLLI2S are disabled.
    0: HSI clock selected as PLL and PLLI2S clock entry
    1: HSE oscillator clock selected as PLL and PLLI2S clock entry

    PLLQ[3:0]: Main PLL (PLL) division factor for USB OTG FS, SDIOclocks
    Set and cleared by software to control the frequency of USB OTG FS clock and the
    SDIOclock. These bits should be written only if PLL is disabled.
    Caution: The USB OTG FS requires a 48 MHz clock to work correctly. The SDIOneeds a
    frequency lower than or equal to 48 MHz to work correctly.
    USB OTG FS clock frequency = VCO frequency / PLLQ with 2 ≤ PLLQ ≤ 15
    0000: PLLQ = 0, wrong configuration
    0001: PLLQ = 1, wrong configuration
    0010: PLLQ = 2
    0011: PLLQ = 3
    0100: PLLQ = 4
    ...
    1111: PLLQ = 15

    PLLR[2:0]: Main PLL division factor for I2Ss, SAIs, SYSTEM and SPDIF-Rx clocks
    Set and cleared by software to control the frequency of the clock. These bits should be
    written only if PLL is disabled.
    Clock frequency = VCO frequency / PLLR with 2 ≤ PLLR ≤ 7
    000: PLLR = 0, wrong configuration
    001: PLLR = 1, wrong configuration
    010: PLLR = 2
    011: PLLR = 3
    ...
    111: PLLR = 7
*/

/* RCC Clock Configuration Register (RCC_CFGR)
   (See datasheet if needed)

*/

#define RCC_FLAG_MASK  ((uint8_t)0x1FU)
#define RCC_GET_FLAG(__FLAG__) (((((((__FLAG__) >> 5U) == 1U)? RCC->CR :((((__FLAG__) >> 5U) == 2U) ? RCC->BDCR :((((__FLAG__) >> 5U) == 3U)? RCC->CSR :RCC->CIR))) & (1U << ((__FLAG__) & RCC_FLAG_MASK)))!= 0U)? 1U : 0U)

// Defines for OscillatorType
#define RCC_OSCILLATORTYPE_NONE            0x00000000U
#define RCC_OSCILLATORTYPE_HSE             0x00000001U
#define RCC_OSCILLATORTYPE_HSI             0x00000002U
#define RCC_OSCILLATORTYPE_LSE             0x00000004U
#define RCC_OSCILLATORTYPE_LSI             0x00000008U

// RCC HSI Config
#define RCC_HSI_OFF                      ((uint8_t)0x00)
#define RCC_HSI_ON                       ((uint8_t)0x01)

#define RCC_HSICALIBRATION_DEFAULT       0x10U  

#define RCC_PLL_NONE                      ((uint8_t)0x00)
#define RCC_PLL_OFF                       ((uint8_t)0x01)
#define RCC_PLL_ON                        ((uint8_t)0x02)

#define RCC_PLLSOURCE_HSI                RCC_PLLCFGR_PLLSRC_HSI
#define RCC_PLLSOURCE_HSE                RCC_PLLCFGR_PLLSRC_HSE

#define RCC_PLLP_DIV2                  0x00000002U
#define RCC_PLLP_DIV4                  0x00000004U
#define RCC_PLLP_DIV6                  0x00000006U
#define RCC_PLLP_DIV8                  0x00000008U

// Used to determine System Clock Source
#define RCC_SYSCLKSOURCE_STATUS_HSI     RCC_CFGR_SWS_HSI   /*!< HSI used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_HSE     RCC_CFGR_SWS_HSE   /*!< HSE used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_PLLCLK  RCC_CFGR_SWS_PLL   /*!< PLL used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_PLLRCLK ((uint32_t)(RCC_CFGR_SWS_0 | RCC_CFGR_SWS_1))   /*!< PLLR used as system clock */

// Returns from the above of RCC_SYSCLKSOURCE_STATUS_HSI, RCC_SYSCLKSOURCE_STATUS_HSE, RCC_SYSCLKSOURCE_STATUS_PLLCLK, RCC_SYSCLKSOURCE_STATUS_PLLRCLK
#define RCC_GET_SYSCLK_SOURCE() (RCC->CFGR & RCC_CFGR_SWS)

/** @defgroup RCC_Flag Flags
  *        Elements values convention: 0XXYYYYYb
  *           - YYYYY  : Flag position in the register
  *           - 0XX  : Register index
  *                 - 01: CR register
  *                 - 10: BDCR register
  *                 - 11: CSR register
  * @{
  */

/* Flags in the BDCR register */
#define RCC_FLAG_LSERDY                  ((uint8_t)0x41)

/* Flags in the CSR register */
#define RCC_FLAG_LSIRDY                  ((uint8_t)0x61)
#define RCC_FLAG_BORRST                  ((uint8_t)0x79)
#define RCC_FLAG_PINRST                  ((uint8_t)0x7A)
#define RCC_FLAG_PORRST                  ((uint8_t)0x7B)
#define RCC_FLAG_SFTRST                  ((uint8_t)0x7C)
#define RCC_FLAG_IWDGRST                 ((uint8_t)0x7D)
#define RCC_FLAG_WWDGRST                 ((uint8_t)0x7E)
#define RCC_FLAG_LPWRRST                 ((uint8_t)0x7F)

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
}RCC_PLL_init_t;

// Oscillator struct to store configuration information

typedef struct
{
    uint32_t OscillatorType; /* The oscillators to be configured */
    uint32_t HSEState; /* The new state of the HSE */
    uint32_t LSEState; /* The new state of the LSE */
    uint32_t HSIState; /* The new state of the HSI */
    uint32_t HSICalibrationValue; /* The HSI calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT). This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F*/
    uint32_t LSIState; /* The new state of the LSI */
    RCC_PLL_init_t PLL; /* PLL structure parameters */

} RCC_oscilator_init_t;

JITOS_STATUS RCC_CONFIGURE_OSCILLATOR(RCC_oscilator_init_t *oscillator_init_struct);

// CLK struct to store configuration for AHB and APB
typedef struct
{
  uint32_t ClockType;             /*!< The clock to be configured. This parameter can be a value of @ref RCC_System_Clock_Type      */
  uint32_t SYSCLKSource;          /*!< The clock source (SYSCLKS) used as system clock. This parameter can be a value of @ref RCC_System_Clock_Source    */
  uint32_t AHBCLKDivider;         /*!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK). This parameter can be a value of @ref RCC_AHB_Clock_Source       */
  uint32_t APB1CLKDivider;        /*!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK). This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */
  uint32_t APB2CLKDivider;        /*!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK). This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */

}RCC_clock_init_t;

// Clocktypes
#define RCC_CLOCKTYPE_SYSCLK             0x00000001U
#define RCC_CLOCKTYPE_HCLK               0x00000002U
#define RCC_CLOCKTYPE_PCLK1              0x00000004U
#define RCC_CLOCKTYPE_PCLK2              0x00000008U

// SYSCLK Source types
#define RCC_SYSCLKSOURCE_HSI             RCC_CFGR_SW_HSI
#define RCC_SYSCLKSOURCE_HSE             RCC_CFGR_SW_HSE
#define RCC_SYSCLKSOURCE_PLLCLK          RCC_CFGR_SW_PLL
#define RCC_SYSCLKSOURCE_PLLRCLK         ((uint32_t)(RCC_CFGR_SW_0 | RCC_CFGR_SW_1))

// SYSCLOCK Dividers
#define RCC_SYSCLK_DIV1                  RCC_CFGR_HPRE_DIV1
#define RCC_SYSCLK_DIV2                  RCC_CFGR_HPRE_DIV2
#define RCC_SYSCLK_DIV4                  RCC_CFGR_HPRE_DIV4
#define RCC_SYSCLK_DIV8                  RCC_CFGR_HPRE_DIV8
#define RCC_SYSCLK_DIV16                 RCC_CFGR_HPRE_DIV16
#define RCC_SYSCLK_DIV64                 RCC_CFGR_HPRE_DIV64
#define RCC_SYSCLK_DIV128                RCC_CFGR_HPRE_DIV128
#define RCC_SYSCLK_DIV256                RCC_CFGR_HPRE_DIV256
#define RCC_SYSCLK_DIV512                RCC_CFGR_HPRE_DIV512

// APB divider
#define RCC_HCLK_DIV1                    RCC_CFGR_PPRE1_DIV1
#define RCC_HCLK_DIV2                    RCC_CFGR_PPRE1_DIV2
#define RCC_HCLK_DIV4                    RCC_CFGR_PPRE1_DIV4
#define RCC_HCLK_DIV8                    RCC_CFGR_PPRE1_DIV8
#define RCC_HCLK_DIV16                   RCC_CFGR_PPRE1_DIV16

JITOS_STATUS RCC_CONFIGURE_CLOCK(RCC_clock_init_t *clock_init_struct, uint32_t flash_latency);

// Step 1: Enable APB2 clocks
// RCC APB2 peripheral clock enable register (RCC_APB2ENR) follows the following structure
typedef enum rcc_apb2_register_t_enum{
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
} rcc_apb2_register_t;

JITOS_STATUS RCC_APB2_ENABLE(rcc_apb2_register_t rcc_apb2_bit_to_set);
JITOS_STATUS RCC_APB2_DISABLE(rcc_apb2_register_t rcc_apb2_bit_to_clear);
JITOS_STATUS RCC_APB2_READ(rcc_apb2_register_t rcc_apb2_bit_to_read, volatile uint32_t * read_output);

// Step 2: Enable APB1 Clocks
// RCC APB1 peripheral clock enable register (RCC_APB1ENR) follows the following structure
typedef enum rcc_apb1_register_t_enum{
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
}rcc_apb1_register_t;

JITOS_STATUS RCC_APB1_ENABLE(rcc_apb1_register_t rcc_apb1_bit_to_set);
JITOS_STATUS RCC_APB1_DISABLE(rcc_apb1_register_t rcc_apb1_bit_to_clear);
JITOS_STATUS RCC_APB1_READ(rcc_apb1_register_t rcc_apb1_bit_to_read, volatile uint32_t * read_output);

/*
    Voltage Scaling -- Voltage regulator

    From Datasheet:
    An embedded linear voltage regulator supplies all the digital circuitries except for the backup
    domain and the Standby circuitry. The regulator output voltage is around 1.2 V.
    This voltage regulator requires two external capacitors to be connected to two dedicated
    pins, VCAP_1 and VCAP_2 available in all packages. Specific pins must be connected either to
    VSS or VDD to activate or deactivate the voltage regulator. These pins depend on the
    package.
    When activated by software, the voltage regulator is always enabled after Reset. It works in
    three different modes depending on the application modes (Run, Stop, or Standby mode).

    **** RUN MODE ****
    In Run mode, the main regulator supplies full power to the 1.2 V domain (core,
    memories and digital peripherals). In this mode, the regulator output voltage (around
    1.2 V) can be scaled by software to different voltage values (scale 1, scale 2, and scale
    3 can be configured through VOS[1:0] bits of the PWR_CR register). The scale can be
    modified only when the PLL is OFF and the HSI or HSE clock source is selected as
    system clock source. The new value programmed is active only when the PLL is ON.
    When the PLL is OFF, the voltage scale 3 is automatically selected.
    The voltage scaling allows optimizing the power consumption when the device is
    clocked below the maximum system frequency. After exit from Stop mode, the voltage scale 3 is automatically selected.(see Section 5.4.1: PWR power control register
    (PWR_CR).
    2 operating modes are available:
    – Normal mode: The CPU and core logic operate at maximum frequency at a given
    voltage scaling (scale 1, scale 2 or scale 3)
    – Over-drive mode: This mode allows the CPU and the core logic to operate at a
    higher frequency than the normal mode for the voltage scaling scale 1 and scale
    2.

    **** STOP MODE ****
    In Stop mode: the main regulator or low-power regulator supplies a low-power voltage
    to the 1.2V domain, thus preserving the content of registers and internal SRAM.
    The voltage regulator can be put either in main regulator mode (MR) or in low-power
    mode (LPR). Both modes can be configured by software as follows:
    – Normal mode: the 1.2 V domain is preserved in nominal leakage mode. It is the
    default mode when the main regulator (MR) or the low-power regulator (LPR) is
    enabled.
    – Low voltage mode.
    – Under-drive mode: the 1.2 V domain is preserved in reduced leakage mode. This
    mode is only available when the main regulator or the low-power regulator is in
    low voltage mode (see Table 14).

    **** STANDBY MODE ****
    In Standby mode: the regulator is powered down. The content of the registers and
    SRAM are lost except for the Standby circuitry and the backup domain.
    Note: Over-drive and under-drive mode are not available when the regulator is bypassed.
    For more details, refer to the voltage regulator section in the STM32F446xx datasheet.

    PWR power control register (PWR_CR) - contains bits to control voltage scaling

    Bits 15:14 VOS[1:0]: Regulator voltage scaling output selection
    These bits control the main internal voltage regulator output voltage to achieve a trade-off
    between performance and power consumption when the device does not operate at the
    maximum frequency (refer to the STM32F446xx datasheet for more details).
    These bits can be modified only when the PLL is OFF. The new value programmed is active
    only when the PLL is ON. When the PLL is OFF, the voltage scale 3 is automatically
    selected.
    00: Reserved (Scale 3 mode selected)    
    01: Scale 3 mode
    10: Scale 2 mode
    11: Scale 1 mode (reset value)
*/

// Contains the voltage scaling presets for this target the values
typedef enum voltage_scale_enum{
    /* Scale 1 mode(default value at reset): the maximum value of fHCLK is 168 MHz. It can be extended to 180 MHz by activating the over-drive mode. */
    power_regulator_voltage_scale1 = PWR_CR_VOS,
    /* Scale 2 mode: the maximum value of fHCLK is 144 MHz. It can be extended to 168 MHz by activating the over-drive mode. */
    power_regulator_voltage_scale2 = PWR_CR_VOS_1,
    /* Scale 3 mode: the maximum value of fHCLK is 120 MHz. */
    power_regulator_voltage_scale3 = PWR_CR_VOS_0,
}pwr_regulator_t;

JITOS_STATUS configure_voltage_scaling_stm32f4(pwr_regulator_t voltage_scale);
JITOS_STATUS configure_clocks_stm32f4(void);

// LD2 is the designated LED at PA5 active high
#define BOOTLOADER_LED_PIN "PA5"


// Prototypes
JITOS_STATUS run_bootloader_initialization_stm32f4(void);
JITOS_STATUS configure_clocks_stm32f4(void);
JITOS_STATUS enable_gpio_stm32f4(void);
JITOS_STATUS enable_usbfs_stm32f4(void);
