#include <stdlib.h>
#include "JITOS.h"

//Write function will be used for printf
__attribute__((weak)) int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_send( *ptr++ );
	}
	return len;
}


