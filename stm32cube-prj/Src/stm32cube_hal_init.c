/**
******************************************************************************
* @file    stm32cube_hal_init.c
* @author  System LAB
* @version V1.0.0
* @date    17-June-2015
* @brief   STM32 Cube HAL Init Source file (formerly main.c as per STM32Cube)
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
/* Includes ------------------------------------------------------------------*/

#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stm32cube_hal_init.h"
#include "stm32l1xx_hal_pwr.h"
#include "stm32l1xx_hal_gpio.h"
#include "stm32l1xx_nucleo.h"
#include "radio_shield_config.h"
#include "spirit1_appli.h"
#include "rtc.h"

extern UART_HandleTypeDef UartHandle;
FlagStatus TamperStatus = RESET;
volatile uint8_t scheduler_started=0;
int dec_precision = 2;
volatile float UVI_Value;
#define OLD_IMPL 0
#if OLD_IMPL
extern RTC_HandleTypeDef RtcHandle;
static void RTC_Config(void);
static void RTC_TimeStampConfig(void);
#endif
void SystemPower_Config(void);
static void MX_GPIO_Init(void);

void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);
void BSP_LED_Init(Led_TypeDef Led);
void BSP_LED_Off(Led_TypeDef Led);
void BSP_LED_On(Led_TypeDef Led);
void BSP_LED_Toggle(Led_TypeDef Led);
void USARTConfig(void);
void Stack_6LoWPAN_Init(void);

static void Error_Handler();

I2C_HandleTypeDef I2cHandle;
#define I2C_ADDRESS        0x30F

/* I2C SPEEDCLOCK define to max value: 400 KHz on STM32L1xx*/
#define I2C_SPEEDCLOCK   400000
#define I2C_DUTYCYCLE    I2C_DUTYCYCLE_2


/**
  * @brief  stm32cube_hal_init()
  * @param  None
  * @retval None
  */
void stm32cube_hal_init()
{
    HAL_Init();
    /* Configure the system clock */
    SystemClock_Config();

    /*HAL_EnableDBGStopMode();*/
    
    MX_GPIO_Init();

    HAL_Spirit1_Init();

    SPIRIT1_Init();
    SystemPower_Config();

    USARTConfig();
   /* Initialize RTC */
    RTC_Config();

   /*clear flags if return from standby*/
   if(__HAL_PWR_GET_FLAG(PWR_FLAG_WU)){
   __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
   }
   if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB)){
	   __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
   }


   /*to set RTC Time, use in the application RTC_Time_Regulate()*/


}


#if OLD_IMPL
/**
  * @brief  Configure the RTC peripheral by selecting the clock source.
  * @param  None
  * @retval None
  */
void RTC_Config(void)
{
  /*##-1- Configure the RTC peripheral #######################################*/
  RtcHandle.Instance = RTC;

  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follow:
  - Hour Format    = Format 24
  - Asynch Prediv  = Value according to source clock
  - Synch Prediv   = Value according to source clock
  - OutPut         = Output Disable
  - OutPutPolarity = High Polarity
  - OutPutType     = Open Drain */ 
  RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
  RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  
  if(HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  } 
}


/**
 * @brief  Configures the current time and date.
 * @param  None
 * @retval None
 */
static void RTC_TimeStampConfig(void)
{
    RTC_DateTypeDef sdatestructure;
    RTC_TimeTypeDef stimestructure;

    /*##-3- Configure the Date #################################################*/
    /* Set Date: Tuesday February 18th 2014 */
    sdatestructure.Year = 0x14;
    sdatestructure.Month = RTC_MONTH_FEBRUARY;
    sdatestructure.Date = 0x18;
    sdatestructure.WeekDay = RTC_WEEKDAY_TUESDAY;

    if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,FORMAT_BCD) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /*##-4- Configure the Time #################################################*/
    /* Set Time: 08:10:00 */
    stimestructure.Hours = 0x08;
    stimestructure.Minutes = 0x10;
    stimestructure.Seconds = 0x00;
    stimestructure.TimeFormat = RTC_HOURFORMAT_24;
    stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
    stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

    if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,FORMAT_BCD) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
}
#endif /*OLD_IMPL*/
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_MUL                        = 4
  *            PLL_DIV                        = 2
  *            Flash Latency(WS)              = 1
  *            Main regulator output voltage  = Scale1 mode
  * @param  None
  * @retval None
  */
#ifdef SYSCLOCKCONFIGMSI

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = (MSI)
  *            MSI Range                      = 0
  *            SYSCLK(Hz)                     = 32000
  *            HCLK(Hz)                       = 32000
  *            AHB Prescaler                  = 2
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            Main regulator output voltage  = Scale2 mode
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /* Enable MSI Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.MSICalibrationValue = 0x00;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Set MSI range to 0 */
  __HAL_RCC_MSI_RANGE_CONFIG(RCC_MSIRANGE_5);

}

#else


void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable HSI Oscillator and Activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    
  /* Set Voltage scale1 as MCU will run at 32MHz */
  __PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {};

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif /*SYSCLOCKCONFIGMSI*/

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
      while(1)
      {
      }
}

/**
  * @brief RTC MSP Initialization 
  *        This function configures the hardware resources used in this example
  * @param hrtc: RTC handle pointer
  * 
  * @note  Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to select 
  *        the RTC clock source; in this case the Backup domain will be reset in  
  *        order to modify the RTC Clock source, as consequence RTC registers (including 
  *        the backup registers) and RCC_BDCR register are set to their reset values.
  *             
  * @retval None
  */
void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc)
{
  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
  
  /*##-1- Configue LSE as RTC clock soucre ###################################*/ 
#ifdef RTC_CLOCK_SOURCE_LSE
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  { 
    /* Initialization Error */
    Error_Handler(); 
  }
  
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  { 
    /* Initialization Error */
    Error_Handler(); 
  }
#elif defined (RTC_CLOCK_SOURCE_LSI)  
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
  /*if returned from standby, do not reset RTC registers*/
  if(!__HAL_PWR_GET_FLAG(PWR_FLAG_SB)){

	  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	  {
		  /* Initialization Error */
		  Error_Handler();
	  }
  }
#else
#error Please select the RTC Clock source inside the stm32cube_hal_init.h file
#endif /*RTC_CLOCK_SOURCE_LSE*/
  
  /*##-2- Enable RTC peripheral Clocks #######################################*/ 
  /* Enable RTC Clock */ 
  __HAL_RCC_RTC_ENABLE(); 

  /*##-3- Configure the NVIC for RTC RTC Wakeup IRQn ###################################*/
  HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0x0, 0);
  HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
  /*##-3- Configure the NVIC for RTC RTC Alarm IRQn ###################################*/
  HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0x0, 0);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
}


/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
}
void SystemPower_Config(){
   /* Configure unused GPIO port pins in Analog Input mode (floating input trigger OFF) */
   GPIO_InitTypeDef GPIO_InitStructure = {0};

   GPIO_InitStructure.Pin = GPIO_PIN_All;
   GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
   GPIO_InitStructure.Pull = GPIO_NOPULL;
     HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

     /* Disable GPIOs clock */
     __GPIOH_CLK_DISABLE();

}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
