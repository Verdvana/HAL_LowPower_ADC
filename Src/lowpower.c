#include "lowpower.h"
#include "stm32l0xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "main.h"

	void SleepMode_Measure(void)
	{
		
	SystemMSIClock_Config();  //开启MSI（65KHz），切换系统时钟至MSI
	SystemHSIClock_Off();     //关闭HSI，关闭PLL
		
	__HAL_FLASH_SLEEP_POWERDOWN_ENABLE();
	
  /* Suspend Tick increment to prevent wakeup by Systick interrupt. 
     Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base) */
  HAL_SuspendTick();
   
  /* Request to enter SLEEP mode */
  HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
   
  /* Resume Tick interrupt if disabled prior to sleep mode entry */
  HAL_ResumeTick();
   
	SystemClock_Config();    //开启HSI
	
}


void StandbyMode_Measure(void)
{
  /* Disable all used wakeup sources: Pin1(PA.0) */
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
   
  /* Clear the related wakeup pin flag */
  //__HAL_PWR_CLEAR_WAKEUP_FLAG(PWR_WAKEUP_PIN_FLAG1);
   
  /* Re-enable all used wakeup sources: Pin1(PA.0) */
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
 
  /* Request to enter STANDBY mode  */
  HAL_PWR_EnterSTANDBYMode();
}


