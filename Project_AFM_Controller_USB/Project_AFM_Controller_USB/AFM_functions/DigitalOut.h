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
#ifndef MBED_DIGITALOUT_H
#define MBED_DIGITALOUT_H

#include "DigitalIO_API.h"
//namespace mbed {

	/** A digital output, used for setting the state of a pin
	*
	* Example:
	* @code
	* // Toggle a LED
	* #include "mbed.h"
	*
	* DigitalOut led(LED1);
	*
	* int main() {
	*     while(1) {
	*         led = !led;
	*         wait(0.2);
	*     }
	* }
	* @endcode
	*/
	class DigitalOut : DigitalIO_Base
	{

	public:
		
		/** Create a DigitalOut connected to the specified pin
		*
		*  @param pin DigitalOut pin to connect to
		*/
		DigitalOut(PinName pin) 
		{
			GOIO_Initial(pin, false);
		};

		/** Create a DigitalOut connected to the specified pin
		*
		*  @param pin DigitalOut pin to connect to
		*  @param value the initial pin value
		*/
		//DigitalOut(PinName pin, int value) : gpio() {
		//    gpio_init_out_ex(&gpio, pin, value);
		//}

		/** Set the output, specified as 0 or 1 (int)
		*
		*  @param value An integer specifying the pin output value,
		*      0 for logical 0, 1 (or any other non-zero value) for logical 1
		*/
		void write(int value) // speed=427.3 kHz*2
		{
			mValue=(GPIO_PinState)value;
			HAL_GPIO_WritePin(pPort_Num, mIO_Num, mValue);//GPIO_PIN_SET
		};
		void write_hal(int value) // speed=427.3 kHz*2
		{
			mValue = (GPIO_PinState)value;
			HAL_GPIO_WritePin(pPort_Num, mIO_Num, mValue);//GPIO_PIN_SET
		}
		;

		/** Return the output setting, represented as 0 or 1 (int)
		*
		*  @returns
		*    an integer representing the output setting of the pin,
		*    0 for logical 0, 1 for logical 1
		*/
		int read() {
			return mValue;// gpio_read(&gpio);
		};

		void toggle(bool mock=false)
		{
			if (mock == false)
			{
				mValue = (GPIO_PinState) !(bool)mValue;
				write(mValue);
			}
			else
				HAL_GPIO_TogglePin(pPort_Num, mIO_Num);
		};// 88*2 kHz
		/** Return the output setting, represented as 0 or 1 (int)
		*
		*  @returns
		*    Non zero value if pin is connected to uc GPIO
		*    0 if gpio object was initialized with NC
		*/
		//int is_connected() {
		//	return gpio_is_connected(&gpio);
		//}

#ifdef MBED_OPERATORS
		/** A shorthand for write()
		*/
		DigitalOut& operator= (int value) {
			write(value);
			return *this;
		}

		DigitalOut& operator= (DigitalOut& rhs) {
			write(rhs.read());
			return *this;
		}

		/** A shorthand for read()
		*/
		operator int() {
			return read();
		}
#endif

	//protected:
	//	gpio_t gpio;
	};

//} // namespace mbed

#endif
