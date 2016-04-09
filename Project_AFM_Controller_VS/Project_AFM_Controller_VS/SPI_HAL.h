#ifndef __CSPI_HAL__
#define  __CSPI_HAL__

#include "stm32fxxx_hal.h"



#include "defines.h"

#include "tm_stm32_spi.h"
#include "string.h"


class CSPI_HAL
{
public:
	CSPI_HAL(void);
	~CSPI_HAL(void);
};


#endif // !__CSPI_HAL__
