/* SWSPI, Software SPI library
 * Copyright (c) 2012-2014, David R. Van Wagner, http://techwithdave.blogspot.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef SWSPI_H
#define SWSPI_H

/** A software implemented SPI that can use any digital pins
 *
 * Useful when don't want to share a single SPI hardware among attached devices
 * or when pinout doesn't match exactly to the target's SPI pins
 *
 * @code
 * #include "mbed.h"
 * #include "SWSPI.h"
 * 
 * SWSPI spi(p5, p6, p7); // mosi, miso, sclk
 * 
 * int main() 
 * {
 *     DigitalOut cs(p8);
 *     spi.format(8, 0);
 *     spi.frequency(10000000);
 *     cs.write(0);
 *     spi.write(0x9f);
 *     int jedecid = (spi.write(0) << 16) | (spi.write(0) << 8) | spi.write(0);
 *     cs.write(1);
 * }
 * @endcode
 */


// arduino define
//#define SPI_MODE0 0x02
//#define SPI_MODE1 0x00
//#define SPI_MODE2 0x03
//#define SPI_MODE3 0x01

//TM_SPI_Mode_0 = 0x00, /*!< Clock polarity low, clock phase 1st edge */
//TM_SPI_Mode_1,        /*!< Clock polarity low, clock phase 2nd edge */
//TM_SPI_Mode_2,        /*!< Clock polarity high, clock phase 1st edge */
//TM_SPI_Mode_3         /*!< Clock polarity high, clock phase 2nd edge */
//
//#include "TickTimer.h"


#include "PortNames.h"
#include "PinNames.h"
#include "DigitalOut.h"
#include "DigitalIn.h"

//#define SPI_WAIT

class SWSPI
{
private:

    int port;
    int bits;
    int mode;
    int polarity; // idle clock value
    int phase; // 0=sample on leading (first) clock edge, 1=trailing (second)
    int freq;
	float period_us;// time to wait for each clock
    
public:

	DigitalOut* mosi;
    DigitalIn* miso;
    DigitalOut* sclk;
    /** Create SWSPI object
     *
     *  @param mosi_pin
     *  @param miso_pin
     *  @param sclk_pin
     */
	SWSPI(PinName mosi_pin, PinName miso_pin, PinName sclk_pin, int frequencyHz=2000000);
    
    /** Destructor */
    ~SWSPI();
    
    /** Specify SPI format
     *
     *  @param bits  8 or 16 are typical values
     *  @param mode  0, 1, 2, or 3 phase (bit1) and idle clock (bit0)
     */
    void format(int bits, int mode_0123 = 0);
    void SetMode(int mode_0123);
	void SetDataLength(int length);
    /** Specify SPI clock frequency
     *
     *  @param hz  frequency (optional, defaults to 10000000)
     */
	void SetFrequency(int frequencyHz);
    
    /** Write data and read result
     *
     *  @param value  data to write (see format for bit size)
     *  returns value read from device
     */
    int write(int value);
	void write_bool_array(bool* value_array_in,bool* value_array_out,int length);

	//template <class T>  T transfer(T value, int data_bit_length);
	uint32_t transfer(uint32_t value, int data_bit_length);
	void transfer_write(uint32_t value, int data_bit_length);
	uint32_t transfer_read(int data_bit_length);
//	unsigned long transfer(unsigned long value, int data_bit_length);
//	unsigned long long transfer(unsigned long long  value, int data_bit_length);
};

#endif // SWSPI_H
