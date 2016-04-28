/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MBED_DIGITAL_IO_API_H
#define MBED_DIGITAL_IO_API_H

#include "PortNames.h"
#include "PinNames.h"

#define MBED_OPERATORS


//#ifdef __cplusplus
//extern "C" {
//#endif
//
class DigitalIO_Base
{
protected:
	int mIO_Num;
	GPIO_TypeDef * pPort_Num;
	GPIO_PinState mValue;

public:
	DigitalIO_Base()
	{
	}
	;
	
void 	GOIO_Initial(PinName pin, bool InTrue_OutFalse)
	{
		//	HAL_Init();


		GPIO_TypeDef* m [] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOH, GPIOI};
		int mPort = pin >> 4;
		pPort_Num = m[mPort];



		mIO_Num = pin & 0x0f;
		mIO_Num = 1 << mIO_Num;

		__GPIOA_CLK_ENABLE();
		__GPIOB_CLK_ENABLE();
		__GPIOC_CLK_ENABLE();
		__GPIOD_CLK_ENABLE();
		__GPIOE_CLK_ENABLE();
		__GPIOF_CLK_ENABLE();
		__GPIOG_CLK_ENABLE();
		__GPIOH_CLK_ENABLE();
		__GPIOI_CLK_ENABLE();

		
		GPIO_InitTypeDef GPIO_InitStructure;

		GPIO_InitStructure.Pin = mIO_Num;//GPIO_PIN_8;
		if (InTrue_OutFalse == true)
			GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
		else
			GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;

		GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;//850kHz		//
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(pPort_Num, &GPIO_InitStructure);
		//HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	};

};

//#ifdef __cplusplus
//}
//#endif

#endif
