
#ifndef _CONSTATN_DEFINE_
#define _CONSTATN_DEFINE_
//bit operations:  & | ^ ~
//logical operations: && || !
#ifndef BIT_N1
#define BIT_N1(x) (1<<x)
#endif // !BIT_N1

//#define BIT_N1(x) (1<<(x))
#ifndef BIT_N0
#define BIT_N0(x) (~BIT_N1(x))
#endif // !BIT_N0

#define SET_BIT1(x,n) ((x)|BIT_N1(n))
#define SET_BIT0(x,n) ((x)&(BIT_N0(n)))

#define FOR_REPEAT(N,fun) {for(int k=0;k<N;k++){fun;}};
//-------------test code
//while (1)
//{
//	mUSerial.printB(BIT_N0(0), 35);
//	mUSerial.println();
//	int x = BIT_N1(10);
//	x = SET_BIT0(x, 10);
//	mUSerial.printB(x, 11);
//	mUSerial.println();
//	x = SET_BIT1(x, 9);
//	mUSerial.printB(x, 11);
//	mUSerial.println();
//	mUSerial.process();
//	wait(0.3);
//	
//}
//-----------------------


#define Math_Abs(a)              (((a) <  0 ) ? -(a) : (a))

/*! \brief Takes the minimal value of \a a and \a b.
 *
 * \param a Input value.
 * \param b Input value.
 *
 * \return Minimal value of \a a and \a b.
 *
 * \note More optimized if only used with values known at compile time.
 */
#define Min(a, b)           (((a) < (b)) ?  (a) : (b))

/*! \brief Takes the maximal value of \a a and \a b.
 *
 * \param a Input value.
 * \param b Input value.
 *
 * \return Maximal value of \a a and \a b.
 *
 * \note More optimized if only used with values known at compile time.
 */
#define Max(a, b)           (((a) > (b)) ?  (a) : (b))

// Math_Abs() is already defined by stdlib.h

/*! \brief Takes the minimal value of \a a and \a b.
 *
 * \param a Input value.
 * \param b Input value.
 *
 * \return Minimal value of \a a and \a b.
 *
 * \note More optimized if only used with values unknown at compile time.
 */
#define Math_Min(a, b)   Min(a, b)

/*! \brief Takes the maximal value of \a a and \a b.
 *
 * \param a Input value.
 * \param b Input value.
 *
 * \return Maximal value of \a a and \a b.
 *
 * \note More optimized if only used with values unknown at compile time.
 */
#define Math_Max(a, b)   Max(a, b)

#define SIGN(x) ((x>0)-(x<0)) 

//-----------------------------------



//the 6th ADC data is first got in MCU
//priority : 
//ADC6 = Z,
//ADC5 = PRC, 
//ADC4 = X, 
//ADC3 = Y, 
//ADC2 = Temperature, 
//ADC1 = PRC_calibration.



// select z sensor
#define ADC_PORT_ZlOOP_SENSOR ADC_CHANNEL_PRC //ADC_PORT_3x2//
//#define ADC_PORT_ZlOOP_SENSOR ADC_PORT_TUNING_FORK


//-----------------------------------

#define NUM_OF_PIEZO_MODEL (3)
#define NUM_OF_SCANNER (NUM_OF_PIEZO_MODEL)
#define PIEZO_Z (0)
//#define PIEZO_X (2)
//#define PIEZO_Y (1)
#define PIEZO_X (1)
#define PIEZO_Y (2)
//#define PIEZO_T (3)
#define PIEZO_ALL (3)

//#define SCANNER_Z_ONLY (0)
//#define SCANNER_Z_LPF (1)
//#define SCANNER_T_ONLY (2)
//#define SCANNER_ZT (3)
//"Zonly",
//"Z_LPF",
//"Tonly",
//"ZT"});



// calibrated by afm grid 5um on optical microscope
//#define SCANNER_RANGE_Z_NM (21.387973775678940*1000.0)
//#define SCANNER_RANGE_X_NM (27.067266247186911*1000.0)
//#define SCANNER_RANGE_Y_NM (27.209844045785250*1000.0)
// devin's new piezo actuator
//#define SCANNER_RANGE_Z_NM ( 7.299788679537674*1000.0)
//#define SCANNER_RANGE_X_NM (27.067266247186911*1000.0)
//#define SCANNER_RANGE_Y_NM (27.209844045785250*1000.0)
//#define SCANNER_RANGE_Z_NM ( 14*1000.0)
//#define SCANNER_RANGE_X_NM (50*1000.0)
//#define SCANNER_RANGE_Y_NM (50*1000.0)

//// by optical microscope
//#define SCANNER_RANGE_Z_NM ( 21.04*1000.0)
//#define SCANNER_RANGE_X_NM (71.72*1000.0)
//#define SCANNER_RANGE_Y_NM (95.18*1000.0)
// by SEM
#define SCANNER_RANGE_Z_NM (21.514*1000.0)
#define SCANNER_RANGE_X_NM (92.509*1000.0)
#define SCANNER_RANGE_Y_NM (71.816*1000.0)



//#define SCANNER_RANGE_Z_NM (25000)
//#define SCANNER_RANGE_X_NM (22000)
//#define SCANNER_RANGE_Y_NM (21000)

#define MAX_RANGE_Z_PM (SCANNER_RANGE_Z_NM*1000)
#define MAX_RANGE_X_PM (SCANNER_RANGE_X_NM*1000)
#define MAX_RANGE_Y_PM (SCANNER_RANGE_Y_NM*1000)
#define CONV_PM2DAC(x)  ((x)*((float)BIT18MAX/(float)MAX_RANGE_Z_PM)) //*0.01048576


//#define MAX_STEP_NUMBER (1000000.0/85.0)
#define EPS (0.000001)
#define MAX_STEP_SIZE_PIEZO_MODEL_NM (50)//50.0 20// tuning fork(2.0)//(4.0)
#define MAX_STEP_SIZE_PIEZO_MODEL_01 (MAX_STEP_SIZE_PIEZO_MODEL_NM/SCANNER_RANGE_Z_NM)//(0.001)//step size=20~27 nm (0.05)




#define LENGTH_I2C_DATA_SG (11)
#define MCU_AD0 (54)// PIN 54 for analog adc0 on MCU


//CPID mPID_ZLOOP(&DInput_01, &DOutput_01, &DSet_01,2,5,0, DIRECT);
#define BIT18MAX (262143.0)
#define BIT18MAX_RECIPROCAL  (3.814711817595740e-06)
#define BIT18MAX_HALF (BIT18MAX/2.0)
#define BIT18MAX_0D75 (BIT18MAX*3.0/4.0)
#define BIT18MAX_0D9 (BIT18MAX*0.9)
#define BIT32MAX (4294967295.0)
#define BIT24MAX (16777215.0)





# define DAC_PER_NM_Z (BIT18MAX/SCANNER_RANGE_Z_NM) //10.48572

# define DAC_PER_NM_X (BIT18MAX/SCANNER_RANGE_X_NM) //
# define DAC_PER_NM_Y (BIT18MAX/SCANNER_RANGE_Y_NM) //

#define LIMIT_MAX_MIN(x,up,down) (Max(Min(x,up),down))

//#define PERIOD_TIME_CHECK_EXIT(dt) {do{	static unsigned long time_store =0;	unsigned long time_now=millis();	if((time_now - time_store)<(dt))return;	else time_store=time_now;}while(0);}
//#define PERIOD_TIME_CHECK_EXIT(dt) {do{	static unsigned long time_store =0;	unsigned long time_now=millis();	if((time_now - time_store)<(dt)) 	{		time_store=time_now;		return;	}}while(0);}
//#define PERIOD_TIME_CHECK_EXIT_WITH_VALUE(dt,value) {do{	static unsigned long time_store =0;	unsigned long time_now=millis();	if((time_now - time_store)<(dt)) 	{	return (value);	}		time_store=time_now;	}while(0);}
//#define DIGITAL_PIN_TOGGLE(pin)	{do{static bool x=true; x=!x;fastDigitalWrite(pin,x);} while(0);}
//#define MOD(value,range) {do{while(value>=range)value-=range;while(value<0)value+=range;} while(0);}
#define REGION_LOCK()  static bool mlock = false;if (mlock == true) return;mlock = true; 
#define REGION_LOCK_RETURN0()  static bool mlock = false;if (mlock == true) return 0;mlock = true; 
#define REGION_UNLOCK()    mlock = false;

#endif