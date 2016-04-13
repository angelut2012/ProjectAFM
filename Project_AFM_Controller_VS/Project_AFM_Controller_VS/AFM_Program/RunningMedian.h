﻿//
//    FILE: RunningMedian.h
//  AUTHOR: Rob dot Tillaart at gmail dot com
// PURPOSE: RunningMedian library for Arduino
// VERSION: 0.1.11 
//     URL: http://arduino.cc/playground/Main/RunningMedian
// HISTORY: See RunningMedian.cpp
//
// Released to the public domain
//

#ifndef RunningMedian_h
#define RunningMedian_h
#include "mbed.h"
#include "constant_define.h"
//#if defined(ARDUINO) && ARDUINO >= 100
//#include "Arduino.h"
//#else
//#include "WProgram.h"
//#endif
//
//#include <inttypes.h>
//
//#define RUNNING_MEDIAN_VERSION "0.1.11"
//
// prepare for dynamic version
// not tested use at own risk :)
// #define RUNNING_MEDIAN_USE_MALLOC

// conditional compile to minimize lib
// by removeing a lot of functions.
#define RUNNING_MEDIAN_ALL


// should at least be 5 to be practical
// odd size results in a 'real' middle element.
// even size takes the lower of the two middle elements (TODO)



#define LENGTH_SortFilter_Buffer (100)
#define MEDIAN_MIN_SIZE     (1)
#define MEDIAN_MAX_SIZE     LENGTH_SortFilter_Buffer          // adjust if needed



class RunningMedian
{
public:
    RunningMedian(uint8_t size);        // # elements in the internal buffer
    ~RunningMedian();                   // destructor

    void clear();                       // resets internal buffer and var
    void add(double value);              // adds a new value to internal buffer, optionally replacing the oldest element.
    double getMedian();                  // returns the median == middle element

#ifdef RUNNING_MEDIAN_ALL
    double getAverage();                 // returns average of the values in the internal buffer
    double getAverage(uint8_t nMedian);  // returns average of the middle nMedian values, removes noise from outliers
    double getHighest();                 // returns highest element
    double getLowest();                  // return lowest element

    double getElement(uint8_t n);        // get n'th element from the values in time order
    double getSortedElement(uint8_t n);  // get n'th element from the values in size order
    double predict(uint8_t n);           // predict the Math_Max change of median after n additions

    uint8_t getSize()  { return _size; };  // returns size of internal buffer
    uint8_t getCount() { return _cnt; };   // returns current used elements, getCount() <= getSize()
#endif

protected:
    bool _sorted;
    uint8_t _size;
    uint8_t _cnt;
    uint8_t _idx;

#ifdef RUNNING_MEDIAN_USE_MALLOC// do not use this
    double * _ar;
    uint8_t * _p;
#else
    double _ar[LENGTH_SortFilter_Buffer];
    uint8_t _p[LENGTH_SortFilter_Buffer];
#endif
    void sort();
};

#endif
// END OF FILE