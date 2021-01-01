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
JITOS_STATUS enable_flash_prefetch_buffer(void);

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
*/

#define NVIC_PRIORITYGROUP_0         0x00000007U /* 0 bits for pre-emption priority and 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         0x00000006U /* 1 bits for pre-emption priority and 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         0x00000005U /* 2 bits for pre-emption priority and 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         0x00000004U /* 3 bits for pre-emption priority and 1 bits for subpriority */
#define NVIC_PRIORITYGROUP_4         0x00000003U /* 4 bits for pre-emption priority and 0 bits for subpriority */

JITOS_STATUS configure_NVIC_priority_grouping(uint32_t priority);

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

JITOS_STATUS configure_SYSTICK(uint32_t number_of_ticks);



// LD2 is the designated LED at PA5 active high
#define BOOTLOADER_LED_PIN "PA5"


// Prototypes
JITOS_STATUS run_bootloader_initialization_stm32f4(void);
JITOS_STATUS configure_clocks_stm32f4(void);
JITOS_STATUS enable_gpio_stm32f4(void);
JITOS_STATUS enable_usbfs_stm32f4(void);
