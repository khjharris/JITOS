/*
    JITOS Bootloader target stm32f4
    Kenwood Harris 12/31/2020
*/

#include <stdint.h>
#include "JITOS.h"

// Defines
#define INSTRUCTION_CACHE_ENABLE 1

/* !!! SEE MEMORY MAP FOR DETAILS */

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

// Flash Instruction Cache Mask 
/*
    From ST Datasheet:
    To limit the time lost due to jumps, it is possible to retain 64 lines of 128 bits in an instruction
    cache memory. This feature can be enabled by setting the instruction cache enable (ICEN)
    bit in the FLASH_ACR register. Each time a miss occurs (requested data not present in the
    currently used instruction line, in the prefetched instruction line or in the instruction cache
    memory), the line read is copied into the instruction cache memory. If some data contained
    in the instruction cache memory are requested by the CPU, they are provided without
    inserting any delay. Once all the instruction cache memory lines have been filled, the LRU
    (least recently used) policy is used to determine the line to replace in the instruction memory
    cache. This feature is particularly useful in case of code containing loops.
*/

/*
                                                    FLASH ACCESS CONTROL REGISTER
    +-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+
    |   15  |   14  |   13  |   12  |   11  |   10  |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
    +-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+
       RES     RES     RES    DCRST   ICRST   DCEN    ICEN    PRFTEN   RES     RES     RES     RES  |----------- LATENCY -----------|
*/
#define FLASH_INSTRUCTION_CACHE_EN (0x1UL << 0x9U)
#define FLASH_DATA_CACHE_EN (0x1UL << 0x10U)
#define FLASH_PREFETCH_EN (0x1UL << 0x8U)

// Prototypes
JITOS_STATUS run_bootloader_initialization_stm32f4(void);
JITOS_STATUS configure_clocks_stm32f4(void);
JITOS_STATUS enable_gpio_stm32f4(void);
JITOS_STATUS enable_usbfs_stm32f4(void);