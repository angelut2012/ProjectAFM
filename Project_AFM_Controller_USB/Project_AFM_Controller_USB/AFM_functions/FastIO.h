// performance
// max speed write=4.61MHz*2, regular IO=1.33MHz*2
// fast read write together=2.12*2MHz

////		FastIn *p = new FastIn<Tdio1>;// not working
////		FastIn<Tdio1> *p = new FastIn<Tdio1>;  // pointer must new
////		DigitalIn * p = new DigitalIn(Tdio1);
//FastOut<Tdio2> *p0;
//p0->write(1);
//while (1)
//{
////////		p.write(0);
////////		p.write(1);// 4.61MHz*2
////////	    p = 1;// 2.58MHz*2
////////	    *p = 1;// 2.60MHz*2
//////	    p->write(1);// 4.50*2 MHz
//////		//p.output();
////////	    *p = 0;
////////	    *p = 1;// 2.58MHz*2
//////	    p->write(0);
//////	    p->write(1);
//////	   p0->write(1);
//////	    int x = p->read();
//////	    p0->write(0);
//	int x = p->read();
//	//	    
//	MY_Debug_LN(x);
//	wait(0.5);
//}	
//		



#ifndef __FAST_IO_H
#define __FAST_IO_H

#include "pinmap.h"

typedef struct {
    uint32_t mask;
} fastio_vars;
// define target here
#define PINMASK             (1 << STM_PIN(pin))
#define PINMASK_CLR         ((1<<16) << STM_PIN(pin))
#define PORT                ((GPIO_TypeDef *)(GPIOA_BASE + 0x0400 * STM_PORT(pin)))

#define INIT_PIN            RCC->AHB1ENR |= (1 << STM_PORT(pin)); (PORT->MODER &= ~(GPIO_MODER_MODER0_1 << (STM_PIN(pin) * 2))); container.mask = PINMASK
#define DESTROY_PIN     

#define SET_DIR_INPUT       (PORT->MODER &= ~(GPIO_MODER_MODER0_0 << (STM_PIN(pin) * 2)))
#define SET_DIR_OUTPUT      (PORT->MODER |= (GPIO_MODER_MODER0_0 << (STM_PIN(pin) * 2)))
#define SET_MODE(pull)      //pin_mode(pin, pull);

#define WRITE_PIN_SET       (PORT->BSRR = PINMASK)
#define WRITE_PIN_CLR       (PORT->BSRR = PINMASK_CLR)

#define READ_PIN            ((PORT->IDR & container.mask) ? 1 : 0)
// define target end


//#ifndef INIT_PIN
//#warning Target is not supported by FastIO
//#warning Reverting to regular DigitalInOut
//#include "FastIO_Unsupported.h"
//#endif

//#include "mbed.h"

/**
 * Faster alternative compared to regular DigitalInOut
 *
 * Except the constructor it is compatible with regular DigitalInOut.
 * Code is based on Igor Skochinsky's code (http://mbed.org/users/igorsk/code/FastIO/)
 */
template <PinName pin> class FastInOut
{
public:
    /**
     * Construct new FastInOut object
     *
     * @code
     * FastInOut<LED1> led1;
     * @endcode
     *
     * No initialization is done regarding input/output mode,
     * FastIn/FastOut can be used if that is required
     *
     * @param pin pin the FastOut object should be used for
     */
    FastInOut() {
        INIT_PIN;
    }
    
    ~FastInOut() {
        DESTROY_PIN;
    }

    void write(int value) {
        if ( value )
            WRITE_PIN_SET;
        else
            WRITE_PIN_CLR;
    }
    int read() {
        return READ_PIN;
    }

    void mode(PinMode pull) {
        SET_MODE(pull);
    }

    void output() {
        SET_DIR_OUTPUT;
    }

    void input() {
        SET_DIR_INPUT;
    }

    FastInOut& operator= (int value) {
        write(value);
        return *this;
    };
    FastInOut& operator= (FastInOut& rhs) {
        return write(rhs.read());
    };
    operator int() {
        return read();
    };
    
    protected:
    fastio_vars container;
};



/**
 * Faster alternative compared to regular DigitalOut
 *
 * Except the constructor it is compatible with regular DigitalOut. Aditionally all
 * functions from DigitalInOut are also available (only initialization is different)
 * Code is based on Igor Skochinsky's code (http://mbed.org/users/igorsk/code/FastIO/)
 */
template <PinName pin, int initial = 0> class FastOut : public FastInOut<pin>
{
public:
    /**
     * Construct new FastOut object
     *
     * @code
     * FastOut<LED1> led1;
     * @endcode
     *
     * @param pin pin the FastOut object should be used for
     * @param initial (optional) initial state of the pin after construction: default is 0 (low)
     */
    FastOut() : FastInOut<pin>::FastInOut() {
       // write(initial);
        SET_DIR_OUTPUT;
    }

    FastOut& operator= (int value) {
        this->write(value);
        return *this;
    };
    FastOut& operator= (FastOut& rhs) {
        return this->write(rhs.read());
    };
    operator int() {
        return this->read();
    };
};

/**
 * Faster alternative compared to regular DigitalIn
 *
 * Except the constructor it is compatible with regular DigitalIn. Aditionally all
 * functions from DigitalInOut are also available (only initialization is different)
 * Code is based on Igor Skochinsky's code (http://mbed.org/users/igorsk/code/FastIO/)
 */
template <PinName pin, PinMode pinmode = PullDefault> class FastIn : public FastInOut<pin>
{
public:
    /**
     * Construct new FastIn object
     *
     * @code
     * FastIn<LED1> led1;
     * @endcode
     *
     * @param pin pin the FastIn object should be used for
     * @param pinmode (optional) initial mode of the pin after construction: default is PullDefault
     */
    FastIn() : FastInOut<pin>::FastInOut() {
        SET_MODE(pinmode);
        SET_DIR_INPUT;
    }

    FastIn& operator= (int value) {
        this->write(value);
        return *this;
    };
    FastIn& operator= (FastIn& rhs) {
        return this->write(rhs.read());
    };
    operator int() {
        return this->read();
    };
};

#endif