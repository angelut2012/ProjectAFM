/**
  ******************************************************************************
  * @file    system_stm32f7xx.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    25-June-2015
  * @brief   CMSIS Cortex-M7 Device Peripheral Access Layer System Source File.
  *
  *   This file provides two functions and one global variable to be called from 
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and 
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32f7xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick 
  *                                  timer or configure other parameters.
  *                                     
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  * This file configures the system clock as follows:
  *-----------------------------------------------------------------------------
  * System clock source       | [1] PLL_HSE_XTAL      | [2] PLL_HSI if [1] fails
  *                           | (external 25MHz xtal) | (internal 16MHz clock)
  *-----------------------------------------------------------------------------
  * SYSCLK(MHz)               | 216                   | 216
  *-----------------------------------------------------------------------------
  * AHBCLK (MHz)              | 216                   | 216
  *-----------------------------------------------------------------------------
  * APB1CLK (MHz)             |  54                   |  54
  *-----------------------------------------------------------------------------
  * APB2CLK (MHz)             | 108                   | 108
  *-----------------------------------------------------------------------------
  * USB capable               | YES                   |  NO
  * with 48 MHz precise clock |                       |
  *-----------------------------------------------------------------------------  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f7xx_system
  * @{
  */  
  
/** @addtogroup STM32F7xx_System_Private_Includes
  * @{
  */
#ifndef __AFM_CLOCK_DEFINE__
#define __AFM_CLOCK_DEFINE__

#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_rcc.h"
#include "stm32f7xx_hal_flash_ex.h"
#include "stm32f7xx_hal_pwr_ex.h"

#include "AFM_clock.h"
//HAL_StatusTypeDef HAL_Init(void);
//
class CAFM_ClockDefine 
{
	
public:
	CAFM_ClockDefine()
	{
	}
	;
	~CAFM_ClockDefine()
	{
	}
	;
	
private:
#if !defined  (HSE_VALUE) 
#define HSE_VALUE    ((uint32_t)25000000) /*!< Default value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
#define HSI_VALUE    ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @}
  */

/** @addtogroup STM32F7xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F7xx_System_Private_Defines
  * @{
  */

/************************* Miscellaneous Configuration ************************/
/*!< Uncomment the following line if you need to use external SRAM or SDRAM mounted
     on EVAL board as data memory  */
/* #define DATA_IN_ExtSRAM */ 
/* #define DATA_IN_ExtSDRAM */

#if defined(DATA_IN_ExtSRAM) && defined(DATA_IN_ExtSDRAM)
#error "Please select DATA_IN_ExtSRAM or DATA_IN_ExtSDRAM " 
#endif /* DATA_IN_ExtSRAM && DATA_IN_ExtSDRAM */

/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x00 /*!< Vector Table base offset field. 
                                   This value must be a multiple of 0x200. */
/******************************************************************************/

/**
  * @}
  */

/** @addtogroup STM32F7xx_System_Private_Macros
  * @{
  */

/* Select the clock sources (other than HSI) to start with (0=OFF, 1=ON) */
//#define USE_PLL_HSE_EXTC (0) /* Use external clock --> NOT USED ON THIS BOARD */ 
//#define USE_PLL_HSE_XTAL (1) /* Use external xtal */
#define USE_PLL_HSE_EXTC (1) /* Use external clock --> NOT USED ON THIS BOARD */ 
#define USE_PLL_HSE_XTAL (0) /* Use external xtal */
/**
  * @}
  */

/** @addtogroup STM32F7xx_System_Private_Variables
  * @{
  */

  /* This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency 
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
	uint32_t SystemCoreClock = HSI_VALUE;
	const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

	
	__weak HAL_StatusTypeDef AFM_HAL_InitTick(uint32_t TickPriority)
	{
	  /*Configure the SysTick to have interrupt in 1ms time basis*/
	  //  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
		HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / __TICK_FREQUENCY_HZ__);// change system default tick frequency=31.250k,HAL_RCC_GetHCLKFreq())=216M Hz
	  /*Configure the SysTick IRQ priority */
		HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority, 0);

		  /* Return function status */
		return HAL_OK;
	}
	HAL_StatusTypeDef AFM_HAL_Init(void)
	{
	  /* Configure Flash prefetch and Instruction cache through ART accelerator */ 
#if (ART_ACCLERATOR_ENABLE != 0)
		__HAL_FLASH_ART_ENABLE();
#endif /* ART_ACCLERATOR_ENABLE */

		  /* Set Interrupt Group Priority */
		HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

		  /* Use systick as time base source and configure 1ms tick (default clock after Reset is HSI) */
		AFM_HAL_InitTick(TICK_INT_PRIORITY);
  
		/* Init the low level hardware */
		HAL_MspInit();
  
		/* Return function status */
		return HAL_OK;
	}

	void AFM_SystemClock_Config(void)
	{
		RCC_ClkInitTypeDef RCC_ClkInitStruct;
		RCC_OscInitTypeDef RCC_OscInitStruct;
		RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
  
		  /* Enable HSE Oscillator and activate PLL with HSE as source */
		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
		RCC_OscInitStruct.HSEState = RCC_HSE_ON;
		RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
		RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
		RCC_OscInitStruct.PLL.PLLM = 25;
		RCC_OscInitStruct.PLL.PLLN = 432;  
		RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
		RCC_OscInitStruct.PLL.PLLQ = 9;
		if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		{
			asm("bkpt 255");
		}
  
		  /* Activate the OverDrive to reach the 216 Mhz Frequency */
		if (HAL_PWREx_EnableOverDrive() != HAL_OK)
		{
			asm("bkpt 255");
		}
  
		  /* Select PLLSAI output as USB clock source */
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
		PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
		PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
		PeriphClkInitStruct.PLLSAI.PLLSAIQ = 7; 
		PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct)  != HAL_OK)
		{
			asm("bkpt 255");
		}
  
		  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
		     clocks dividers */
		RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
		RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
		RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
		{
			asm("bkpt 255");
		}
	}

	void AFM_SystemCoreClockUpdate(void)
	{
		uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;
  
		/* Get SYSCLK source -------------------------------------------------------*/
		tmp = RCC->CFGR & RCC_CFGR_SWS;

		switch (tmp)
		{
		case 0x00:  /* HSI used as system clock source */
			SystemCoreClock = HSI_VALUE;
			break;
		case 0x04:  /* HSE used as system clock source */
			SystemCoreClock = HSE_VALUE;
			break;
		case 0x08:  /* PLL used as system clock source */

		      /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
		         SYSCLK = PLL_VCO / PLL_P
		         */    
			pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
			pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
      
			if (pllsource != 0)
			{
			  /* HSE used as PLL clock source */
				pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
			}
			else
			{
			  /* HSI used as PLL clock source */
				pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);      
			}

			pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1) * 2;
			SystemCoreClock = pllvco / pllp;
			break;
		default:
			SystemCoreClock = HSI_VALUE;
			break;
		}
		/* Compute HCLK frequency --------------------------------------------------*/
		/* Get HCLK prescaler */
		tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
		/* HCLK frequency */
		SystemCoreClock >>= tmp;
	}





	uint8_t AFM_SetSysClock_PLL_HSE(uint8_t bypass)
	{
		RCC_ClkInitTypeDef RCC_ClkInitStruct;
		RCC_OscInitTypeDef RCC_OscInitStruct;

		  // Enable power clock  
		__PWR_CLK_ENABLE();
  
		// Enable HSE oscillator and activate PLL with HSE as source
		RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
		if (bypass == 0)
		{
			RCC_OscInitStruct.HSEState          = RCC_HSE_ON; /* External xtal on OSC_IN/OSC_OUT */
		}
		else
		{
			RCC_OscInitStruct.HSEState          = RCC_HSE_BYPASS; /* External clock on OSC_IN */
		}
		// Warning: this configuration is for a 25 MHz xtal clock only
		RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
		RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
		RCC_OscInitStruct.PLL.PLLM            = 25;            // VCO input clock = 1 MHz (25 MHz / 25)
		RCC_OscInitStruct.PLL.PLLN            = 432;           // VCO output clock = 432 MHz (1 MHz * 432)
		RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2; // PLLCLK = 216 MHz (432 MHz / 2)
		RCC_OscInitStruct.PLL.PLLQ            = 9;             // USB clock = 48 MHz (432 MHz / 9) --> OK for USB
  
		if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		{
			return 0; // FAIL
		}

		  // Activate the OverDrive to reach the 216 MHz Frequency
		if (HAL_PWREx_EnableOverDrive() != HAL_OK)
		{
			return 0; // FAIL
		}
  
		// Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers
		RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
		RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK; // 216 MHz
		RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;         // 216 MHz
		RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;           //  54 MHz
		RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;           // 108 MHz
  
		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
		{
			return 0; // FAIL
		}

		return 1; // OK
	}

	void AFM_SetSysClock(void)
	{

		AFM_SetSysClock_PLL_HSE(1);

	}
public:
	void AFM_SystemInit_Original()
	{
		AFM_HAL_Init();
		AFM_SystemClock_Config();	
	}
	void AFM_SystemInit(void)
	{		AFM_SystemInit_Original();
	  /* FPU settings ------------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));  /* set CP10 and CP11 Full Access */
#endif
	  /* Reset the RCC clock configuration to the default reset state ------------*/
	  /* Set HSION bit */
		RCC->CR |= (uint32_t)0x00000001;

		  /* Reset CFGR register */
		RCC->CFGR = 0x00000000;

		  /* Reset HSEON, CSSON and PLLON bits */
		RCC->CR &= (uint32_t)0xFEF6FFFF;

		  /* Reset PLLCFGR register */
		RCC->PLLCFGR = 0x24003010;

		  /* Reset HSEBYP bit */
		RCC->CR &= (uint32_t)0xFFFBFFFF;

		  /* Disable all interrupts */
		RCC->CIR = 0x00000000;

#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
		SystemInit_ExtMemCtl(); 
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

		  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
		SCB->VTOR = SRAM1_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
		SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif

		  /* Configure the Cube driver */
		SystemCoreClock = HSI_VALUE; // At this stage the HSI is used as system clock
//		  HAL_Init();

		  // Enable CPU L1-Cache
		SCB_EnableICache();
		SCB_EnableDCache();

		  /* Configure the System clock source, PLL Multiplier and Divider factors,
		     AHB/APBx prescalers and Flash settings */
		AFM_SetSysClock();
  
	  /* Reset the timer to avoid issues after the RAM initialization */

//	   TIM_MST_RESET_ON;
	  //  TIM_MST_RESET_OFF;  
		
		AFM_SystemCoreClockUpdate();// this will not affect the system.
		
	}

	
};

#endif // !__AFM_CLOCK_DEFINE__
