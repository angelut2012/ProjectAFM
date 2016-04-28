#ifndef __DEFINE_AFM_HARDWARE__
#define __DEFINE_AFM_HARDWARE__

//#include "mbed.h"
//http://stm32f4-discovery.com/2015/07/new-stm32f7-discovery-board/
//Pins pack 1	Pins pack 2	Pins pack 3	Pins pack 4
//SPIx	MOSI	MISO	SCK	MOSI	MISO	SCK	MOSI	MISO	SCK	MOSI	MISO	SCK
//SPI1	PA7	PA6	PA5	PB5	PB4	PB3						
//SPI2	PC3	PC2	PB10	PB15	PB14	PB13	PI3	PI2	PI0	PB15	PB14	PI1
//SPI3	PB5	PB4	PB3	PC12	PC11	PC10						
//SPI4	PE6	PE5	PE2	PE14	PE13	PE12						
//SPI5	PF9	PF8	PF7	PF11	PH7	PH6						
//SPI6	PG14	PG12	PG13							
//-----------SEM_SPI_ports---------------------
#define Sspi_cs2	(PB_11)
#define Sspi_cs		(PB_12)// inverse for ADC, non-inverting for DAC

#define Sspi_clk	(PB_13)
#define Sspi_miso	(PB_14)
#define Sspi_mosi	(PB_15)
//-----------end of SEM_SPI_ports---------------------

//-----------DAC_SPI_ports---------------------
#define Dspi_cs1	(PD_0)
#define Dspi_cs2	(PD_1)
#define Dspi_cs3	(PD_2)
#define Dspi_cs4	(PD_3)

#define Dspi_clk	(PC_10)
#define Dspi_miso	(PC_11)
#define Dspi_mosi	(PC_12)
//-----------end of DAC_SPI_ports---------------------

//-----------RAM_SPI_ports---------------------
#define Rspi_cs		(PA_3)
#define Rspi_cs2	(PA_4)//use j1
//#define Rspi_cs2	(PB_0)//use j3

#define Rspi_clk	(PA_5)
#define Rspi_miso	(PA_6)
#define Rspi_mosi	(PA_7)

#define Rspi_hold1	(PC_5)
#define Rspi_hold2	(PC_6)

//-----------end of RAM_SPI_ports---------------------


//-----------TuningFork_Board_SPI_ports---------------------


#define Tdio4		(PB_8)
#define Tdio5		(PB_9)

#define Tdio1		(PE_0)
#define Tdio2		(PE_1)
#define Tspi_clk	(PE_2)
#define Tdio3		(PE_3)

#define Tspi_cs		(PE_4)
#define Tspi_miso	(PE_5)
#define Tspi_mosi	(PE_6)
//-----------end of TuningFork_Board_SPI_ports---------------------



//--------------------------------
#define D_ZLiftUp	(PD_4)// digital output, Z axis piezo liftup quickly for protection
//for tuningfork
#define A_Tao_TF	(PA_4)// analog input use j2
#define A_Tai_TF	(PB_1)// analog input

#define D_CD_RST	(PC_13)// digital output for charge control reset initial state

#define D_PS_HV		(PA_0)// digital output for power switch HV power source
#define D_PS_SEM	(PA_1)// digital output for power switch SEM power source
#define D_PS_TF		(PA_2)// digital output for power switch TuningFork power source


//-----------end of ---------------------

//uint32_t multiplier;
// 
//void TM_Delay_Init(void) {
//    RCC_ClocksTypeDef RCC_Clocks;
//    
//    /* Get system clocks */
//    RCC_GetClocksFreq(&RCC_Clocks);
//    
//    /* While loop takes 4 cycles */
//    /* For 1 us delay, we need to divide with 4M */
//    multiplier = RCC_Clocks.HCLK_Frequency / 4000000;
//}
// 
//void TM_DelayMicros(uint32_t micros) {
//    /* Multiply micros with multipler */
//    /* Substract 10 */
//    micros = micros * multiplier - 10;
//    /* 4 cycles for one loop */
//    while (micros--);
//}
// 
//void TM_DelayMillis(uint32_t millis) {
//    /* Multiply millis with multipler */
//    /* Substract 10 */
//    millis = 1000 * millis * multiplier - 10;
//    /* 4 cycles for one loop */
//    while (millis--);
//}

//void wait(float s) {
//    wait_us(s * 1000000.0f);
//}
//
//void wait_ms(int ms) {
//    wait_us(ms * 1000);
//}
//
//void wait_us(int us) {
//    uint32_t x=0xffff;
//    while (x--);
//}

#endif // __DEFINE_AFM_HARDWARE__


