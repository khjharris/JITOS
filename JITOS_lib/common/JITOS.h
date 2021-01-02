/*
    JITOS contains common functions used across all targets
    Kenwood Harris 12/31/2020
    Noah Malhi  1/2/2021
*/

#include <stdint.h>

// Macros
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))



// Function Definitions
static void assert_failure_handler( char *file, int line );

//Defining the assert macro
#ifdef NDEBUG
#define assert( condition ) (( void ) 0 )
#else
#define assert( condition ) \
    if( !( condtion ) \
        assert_failure_handler( __FILE__, __LINE__ )
#endif


// Typedefs

typedef enum {
    STATUS_OK,
    STATUS_FAIL,
    STATUS_BUSY,
    STATUS_TIMEOUT,
} JITOS_STATUS;

