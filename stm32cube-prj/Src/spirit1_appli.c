/**
******************************************************************************
* @file    spirit1_appli.c
* @author  System Lab - NOIDA
* @version V1.1.0
* @date    14-Aug-2014
* @brief   user file to configure Spirit1 transceiver.
*         
@verbatim
===============================================================================
##### How to use this driver #####
===============================================================================
[..]
This file is generated automatically by STM32CubeMX and eventually modified 
by the user

@endverbatim
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
#include "stm32l152xe.h"
#include "spirit1_appli.h"
#include "stm32cube_hal_init.h"
#include "MCU_Interface.h"
#include "SPIRIT1_Util.h"
#include <stdio.h>

#include "lib/sensors.h"
extern const struct sensors_sensor button_sensor;

#define OLD_IMPL 0

/** @addtogroup USER
* @{
*/

/** @defgroup SPIRIT1_APPLI
* @brief User file to configure spirit1 tranceiver for desired frequency and 
* @feature.
* @{
*/

/* Private typedef -----------------------------------------------------------*/

/**
* @brief RadioDriver_t structure fitting
*/
RadioDriver_t spirit_cb =
{
  .Init = Spirit1InterfaceInit, 
  .GpioIrq = Spirit1GpioIrqInit,
  .RadioInit = Spirit1RadioInit,
  .SetRadioPower = Spirit1SetPower,
  .PacketConfig = Spirit1PacketConfig,
  .SetPayloadLen = Spirit1SetPayloadlength,
  .SetDestinationAddress = Spirit1SetDestinationAddress,
  .EnableTxIrq = Spirit1EnableTxIrq,
  .EnableRxIrq = Spirit1EnableRxIrq,
  .DisableIrq = Spirit1DisableIrq,
  .SetRxTimeout = Spirit1SetRxTimeout,
  .EnableSQI = Spirit1EnableSQI,
  .SetRssiThreshold = Spirit1SetRssiTH,
  .ClearIrqStatus = Spirit1ClearIRQ,
  .StartRx = Spirit1StartRx,
  .StartTx = Spirit1StartTx,
  .GetRxPacket = Spirit1GetRxPacket
};

#if OLD_IMPL

/**
* @brief MCULowPowerMode_t structure fitting
*/
MCULowPowerMode_t MCU_LPM_cb =
{
  .McuStopMode = MCU_Enter_StopMode,
  .McuStandbyMode = MCU_Enter_StandbyMode,
  .McuSleepMode = MCU_Enter_SleepMode
}; 

/**
* @brief RadioLowPowerMode_t structure fitting
*/
RadioLowPowerMode_t Radio_LPM_cb =
{
  .RadioShutDown = RadioPowerOFF,
  .RadioStandBy = RadioStandBy,
  .RadioSleep = RadioSleep,
  .RadioPowerON = RadioPowerON
};
#endif /*OLD_IMPL*/

/**
* @brief GPIO structure fitting
*/
SGpioInit xGpioIRQ={
  SPIRIT_GPIO_IRQ,
  SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
  SPIRIT_GPIO_DIG_OUT_IRQ
};

/**
* @brief Radio structure fitting
*/
SRadioInit xRadioInit = {
  XTAL_OFFSET_PPM,
  BASE_FREQUENCY,
  CHANNEL_SPACE,
  CHANNEL_NUMBER,
  MODULATION_SELECT,
  DATARATE,
  FREQ_DEVIATION,
  BANDWIDTH
};


#if defined(USE_STack_PROTOCOL)
/**
* @brief Packet Basic structure fitting
*/
PktStackInit xStackInit={
  PREAMBLE_LENGTH,
  SYNC_LENGTH,
  SYNC_WORD,
  LENGTH_TYPE,
  LENGTH_WIDTH,
  CRC_MODE,
  CONTROL_LENGTH,
  EN_FEC,
  EN_WHITENING
};

/* LLP structure fitting */
PktStackLlpInit xStackLLPInit ={
  EN_AUTOACK,
  EN_PIGGYBACKING,
  MAX_RETRANSMISSIONS
};

/**
* @brief Address structure fitting
*/
PktStackAddressesInit xAddressInit={
  EN_FILT_MY_ADDRESS,
  MY_ADDRESS,
  EN_FILT_MULTICAST_ADDRESS,
  MULTICAST_ADDRESS,
  EN_FILT_BROADCAST_ADDRESS,
  BROADCAST_ADDRESS
};

#elif defined(USE_BASIC_PROTOCOL)

/**
* @brief Packet Basic structure fitting
*/
PktBasicInit xBasicInit={
  PREAMBLE_LENGTH,
  SYNC_LENGTH,
  SYNC_WORD,
  LENGTH_TYPE,
  LENGTH_WIDTH,
  CRC_MODE,
  CONTROL_LENGTH,
  EN_ADDRESS,
  EN_FEC,
  EN_WHITENING
};


/**
* @brief Address structure fitting
*/
PktBasicAddressesInit xAddressInit={
  EN_FILT_MY_ADDRESS,
  MY_ADDRESS,
  EN_FILT_MULTICAST_ADDRESS,
  MULTICAST_ADDRESS,
  EN_FILT_BROADCAST_ADDRESS,
  BROADCAST_ADDRESS
};
#endif


/* Private define ------------------------------------------------------------*/
#define TIME_UP                                         0x01

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
RadioDriver_t *pRadioDriver;
MCULowPowerMode_t *pMCU_LPM_Comm;
RadioLowPowerMode_t  *pRadio_LPM_Comm;
/*Flags declarations*/
volatile FlagStatus xRxDoneFlag = RESET, xTxDoneFlag=RESET, cmdFlag=RESET;
volatile FlagStatus xStartRx=RESET, rx_timeout=RESET, exitTime=RESET;
volatile FlagStatus datasendFlag=RESET, wakeupFlag=RESET;
volatile FlagStatus PushButtonStatusWakeup=RESET;
volatile FlagStatus PushButtonStatusData=RESET;

static __IO uint32_t KEYStatusData = 0x00;
AppliFrame_t xTxFrame, xRxFrame;
uint8_t TxFrameBuff[MAX_BUFFER_LEN] = {0x00};
uint16_t exitCounter = 0;
uint16_t txCounter = 0;
uint16_t wakeupCounter = 0;
uint16_t dataSendCounter = 0x00;

/* Private function prototypes -----------------------------------------------*/
static void LowPower_Config(void);
static void SystemClockConfig_STOP(void);
void HAL_Spirit1_Init(void);
void SPIRIT1_Init(void);
void STackProtocolInit(void);
void BasicProtocolInit(void);
void RadioPowerON(void);
void RadioPowerOFF(void);
void RadioStandBy(void);
void RadioSleep(void);

#if OLD_IMPL
void Enter_LP_mode(MCU_Status_t mcu_status, Radio_Status_t radio_status);
void Exit_LP_mode(void);
void MCU_Enter_StopMode(void);
void MCU_Enter_StandbyMode(void);
void MCU_Enter_SleepMode(void);
void FLASH_WriteArray(uint8_t* array, uint32_t offset, uint32_t length);
Time_Typedef_t RTC_GetTime();
HAL_StatusTypeDef RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss);
void Set_WakeupTimer(uint32_t milliseconds);
void Disable_WakeupTimer();
int Set_Alarm(uint8_t hour, uint8_t minutes, uint8_t seconds);
Alarm_Typedef_t GetAlarm();
void Disable_Alarm();
int Compare_Alarm(Alarm_Typedef_t alarm);
int Compare_Alarms(Alarm_Typedef_t alarm1, Alarm_Typedef_t alarm2);
void BKUPWrite(uint32_t BackupRegister, uint32_t Data);
uint32_t BKUPRegRead(uint32_t BackupRegister);
#endif /*OLD_IMPL*/
void HAL_SYSTICK_Callback(void);
/* Private functions ---------------------------------------------------------*/

/** @defgroup SPIRIT1_APPLI_Private_Functions
* @{
*/

/**
* @brief  Initializes RF Transceiver's HAL.
* @param  None
* @retval None.
*/
void HAL_Spirit1_Init(void)
{
  pRadioDriver = &spirit_cb;
  pRadioDriver->Init( ); 
}



/**
* @brief  This function initializes the protocol for point-to-point 
* communication
* @param  None
* @retval None
*/
void SPIRIT1_Init(void)
{
  pRadioDriver = &spirit_cb;


     /* Spirit IRQ config */
  pRadioDriver->GpioIrq(&xGpioIRQ);

  /* Spirit Radio config */    
  pRadioDriver->RadioInit(&xRadioInit);
  
  /* Spirit Radio set power */
  pRadioDriver->SetRadioPower(POWER_INDEX, POWER_DBM);  
  
  /* Spirit Packet config */  
  pRadioDriver->PacketConfig();
  
  pRadioDriver->EnableSQI();
  
  pRadioDriver->SetRssiThreshold(RSSI_THRESHOLD);
}

/**
* @brief  This function initializes the BASIC Packet handler of spirit1
* @param  None
* @retval None
*/
void BasicProtocolInit(void)
{ 
#if defined(USE_BASIC_PROTOCOL)
  /* Spirit Packet config */
  SpiritPktBasicInit(&xBasicInit);
  SpiritPktBasicAddressesInit(&xAddressInit);
#endif
}


#if OLD_IMPL
/**
* @brief  This routine will put the radio and mcu in LPM
* @param  mcu_status The status in which the MCU is entering
* @param  radio_status The status in which the MCU is entering
* @retval None
*/
void Enter_LP_mode(MCU_Status_t mcu_status, Radio_Status_t radio_status)
{

  pMCU_LPM_Comm = &MCU_LPM_cb;
  pRadio_LPM_Comm = &Radio_LPM_cb;
  
if (radio_status==RF_SHUTDOWN)
  {
    pRadio_LPM_Comm->RadioShutDown();
  }
if (radio_status==RF_STANDBY)
  {
    pRadio_LPM_Comm->RadioStandBy();
  }  
if (radio_status==RF_SLEEP)
  {
    pRadio_LPM_Comm->RadioSleep();
  }

if (mcu_status==MCU_STANDBY_MODE)
  {
    pMCU_LPM_Comm->McuStandbyMode();
  } 
if (mcu_status==MCU_STOP_MODE)
  {
    pMCU_LPM_Comm->McuStopMode();
  }
if (mcu_status==MCU_SLEEP_MODE)
  {
    pMCU_LPM_Comm->McuSleepMode();
  }
}

/**
* @brief  This routine wake-up the mcu and radio from LPM
* @param  None
* @retval None
*/
void Exit_LP_mode(void)
{
  pRadio_LPM_Comm = &Radio_LPM_cb;      
  pRadio_LPM_Comm->RadioPowerON();  
}

/**
* @brief  This routine puts the MCU in stop mode
* @param  None
* @retval None
*/
void MCU_Enter_StopMode(void)
{
  LowPower_Config();
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);  /* Infinite loop */
}

/**
* @brief  This routine puts the MCU in standby mode
* @param  None
* @retval None
*/
void MCU_Enter_StandbyMode(void)
{
	/*in case of bad return to stanby*/
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_WU)){
	   __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	   }
	  LowPower_Config();
	HAL_PWR_EnterSTANDBYMode();  /* Infinite loop */
}

/**
* @brief  This routine puts the MCU in sleep mode
* @param  None
* @retval None
*/
void MCU_Enter_SleepMode(void)
{
  /*Suspend Tick increment to prevent wakeup by Systick interrupt. 
  Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base)*/
  HAL_SuspendTick();

  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  /*when returning from sleep mode resume systick*/
  HAL_ResumeTick();
}

/**
* @brief  This function will turn on the radio and waits till it enters the Ready state.
* @param  Param:None. 
* @retval None
*                       
*/
void RadioPowerON(void)
{
  SpiritCmdStrobeReady();   
  do{
    /* Delay for state transition */
    for(volatile uint8_t i=0; i!=0xFF; i++);
    
    /* Reads the MC_STATUS register */
    SpiritRefreshStatus();
  }
  while(g_xStatus.MC_STATE!=MC_STATE_READY);
}


/**
* @brief  This function will Shut Down the radio.
* @param  Param:None. 
* @retval None
*                       
*/
void RadioPowerOFF(void)
{
  SpiritEnterShutdown();
}


/**
* @brief  This function will put the radio in standby state.
* @param  None. 
* @retval None
*                       
*/
void RadioStandBy(void)
{
  SpiritCmdStrobeStandby();  
#if 0  
  do{
    /* Delay for state transition */
    for(volatile uint8_t i=0; i!=0xFF; i++);
    
    /* Reads the MC_STATUS register */
    SpiritRefreshStatus();
  }
  while(g_xStatus.MC_STATE!=MC_STATE_STANDBY);
#endif
}

/**
* @brief  This function will put the radio in sleep state.
* @param  None. 
* @retval None
*                       
*/
void RadioSleep(void)
{
  SpiritCmdStrobeSleep(); 
#if 0
  do{
    /* Delay for state transition */
    for(volatile uint8_t i=0; i!=0xFF; i++);
    
    /* Reads the MC_STATUS register */
    SpiritRefreshStatus();
  }
  while(g_xStatus.MC_STATE!=MC_STATE_SLEEP);
#endif
}

/**
* @brief  This function writes an array of uint8_t of length 'length'.
* @param  array: poiter to the array to write.
* @param  length: the length of the array passed
* @retval None
*
*/
void FLASH_WriteArray(uint8_t* array, uint32_t offset,  uint32_t length){

	int i;
	HAL_FLASHEx_DATAEEPROM_Unlock();
	  for(i=0;i< length;i++){
	  HAL_FLASHEx_DATAEEPROM_Program(TYPEPROGRAMDATA_FASTBYTE, FLASH_EEPROM_BASE+i, *((uint8_t*)(array+i)));
	  }


	  HAL_FLASHEx_DATAEEPROM_Lock();

}

Time_Typedef_t RTC_GetTime(){
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	Time_Typedef_t ret_time;
	HAL_RTC_GetTime(&RtcHandle, &sTime,FORMAT_BIN );
	HAL_RTC_GetDate(&RtcHandle, &sDate, FORMAT_BIN);
	ret_time.hour=sTime.Hours;
	ret_time.minute=sTime.Minutes;
	ret_time.second=sTime.Seconds;
	return ret_time;
}

HAL_StatusTypeDef RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss){
    RTC_TimeTypeDef stimestructure;

	stimestructure.Hours = hh;
	    stimestructure.Minutes = mm;
	    stimestructure.Seconds = ss;
	    stimestructure.TimeFormat = RTC_HOURFORMAT_24;
	    stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
	    stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;


	return HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BIN);
}

/**
* @brief  This routine sets wakeup timer.
* @param  None
* @retval None
*/
void Set_WakeupTimer(uint32_t milliseconds){
	/*Wakeup Time Base = 16 /(~39.000KHz) = ~0,410 ms
	      Wakeup Time = seconds = 0,410ms  * WakeUpCounter
	       ==> WakeUpCounter = seconds /0,410ms = 9750 = 0x2616 */
    HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandle, milliseconds/0.410, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

}
/*@Brief Set alarm, ignores weekday.
 *@param hours The hours of the alarm
 *@param minutes The minutes of the alarm
 *@param seconds The seconds of the alarm
 *@retval status value of the function
 * */
int Set_Alarm(uint8_t hours, uint8_t minutes, uint8_t seconds){

	  RTC_AlarmTypeDef sAlarm;

	  if(hours>24 ||  minutes>60 || seconds>60){
		  return HAL_ERROR;
	  }

	  sAlarm.AlarmTime.Hours = hours;
	  sAlarm.AlarmTime.Minutes = minutes;
	  sAlarm.AlarmTime.Seconds= seconds;
	  sAlarm.AlarmDateWeekDay = 0x0;
	  sAlarm.AlarmTime.TimeFormat = RTC_HOURFORMAT_24;
	  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
	  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	  sAlarm.Alarm = RTC_ALARM_A;
	  return HAL_RTC_SetAlarm_IT(&RtcHandle, &sAlarm, FORMAT_BIN);
}

Alarm_Typedef_t GetAlarm(){
	RTC_AlarmTypeDef alarm;
	Alarm_Typedef_t alarm_ret;
	HAL_RTC_GetAlarm(&RtcHandle, &alarm, RTC_ALARM_A, FORMAT_BIN);
	alarm_ret.hour= alarm.AlarmTime.Hours;
	alarm_ret.minute=alarm.AlarmTime.Minutes;
	alarm_ret.second=alarm.AlarmTime.Seconds;
	return alarm_ret;
}
void Disable_WakeupTimer(){
    HAL_RTCEx_DeactivateWakeUpTimer(&RtcHandle);

}

void Disable_Alarm(){
	HAL_RTC_DeactivateAlarm(&RtcHandle,RTC_ALARM_A);
}
/*gives 1 if alarm greater than time*/
int Compare_Alarm(Alarm_Typedef_t alarm){
	Time_Typedef_t time;
	time=RTC_GetTime();
	if(time.hour<alarm.hour){
		return 1;
	}
	else if(time.hour==alarm.hour && time.minute<alarm.minute){
		return 1;
	}
		else if(time.hour==alarm.hour && time.minute==alarm.minute && time.second < alarm.second){
			return 1;
		}
	return 0;
}

/*return 1 if alarm1 > alarm2 else 0*/
int Compare_Alarms(Alarm_Typedef_t alarm1, Alarm_Typedef_t alarm2){

	if(alarm1.hour>alarm2.hour){
		return 1;
	}
	else if(alarm1.hour==alarm2.hour && alarm1.minute>alarm1.minute){
		return 1;
	}
	else if(alarm1.hour==alarm2.hour && alarm1.minute==alarm2.minute &&alarm1.second> alarm2.second){
		return 1;
	}
	return 0;
}
/**
* @}
*/

void BKUPWrite(uint32_t BackupRegister, uint32_t Data){
	HAL_RTCEx_BKUPWrite(&RtcHandle, BackupRegister, Data);
}
uint32_t BKUPRegRead(uint32_t BackupRegister){
	return HAL_RTCEx_BKUPRead(&RtcHandle, BackupRegister);
}


void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	printf("exit stop mode\r\n");
	  HAL_PWREx_DisableUltraLowPower();
	  HAL_PWREx_DisableFastWakeUp();
  /* Clear Wake Up Flag */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

  /* Configures system clock after wake-up from STOP: enable HSE, PLL and select
    PLL as system clock source (HSE and PLL are disabled in STOP mode) */
    SystemClockConfig_STOP();

    
    /* Initialize LEDs*/
    RadioShieldLedInit(RADIO_SHIELD_LED);
    //BSP_LED_Init(LED2); 

    sensors_changed(&button_sensor);


}
/**
* @}
*/

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Clear Wake Up Flag */
 /* __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  SystemClock_Config();
	printf("exit stop mode\r\n");
*/

}
#endif /*OLD_IMPL*/


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin==USER_BUTTON_PIN){
	 sensors_changed(&button_sensor);
	}
}

/**
  * @brief  Configures system clock after wake-up from STOP: enable HSI, PLL
  *         and select PLL as system clock source.
  * @param  None
  * @retval None
  */

static void SystemClockConfig_STOP(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Get the Oscillators configuration according to the internal RCC registers */
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

  /* After wake-up from STOP reconfigure the system clock: Enable HSI and PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
}

static void LowPower_Config(void)
{
 GPIO_InitTypeDef GPIO_InitStructure = {0};

 HAL_PWREx_EnableUltraLowPower();
 	HAL_PWREx_EnableFastWakeUp();

  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();

  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */

  GPIO_InitStructure.Pin = GPIO_PIN_All;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

  /* Disable GPIOs clock */

  __GPIOA_CLK_DISABLE();
  __GPIOB_CLK_DISABLE();
  __GPIOC_CLK_DISABLE();
  __GPIOD_CLK_DISABLE();
  __GPIOH_CLK_DISABLE();


}
/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
