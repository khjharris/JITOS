/*
    JITOS contains common functions used across all targets
    Kenwood Harris 12/31/2020
*/

#include <stdint.h>

// Macros
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))

// Typedefs

typedef enum {
    STATUS_OK,
    STATUS_FAIL,
    STATUS_BUSY,
    STATUS_TIMEOUT,
} JITOS_STATUS;




