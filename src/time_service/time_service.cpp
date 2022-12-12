#include "time_service.h"

#ifndef TIME_SERVICE_H
#define TIME_SERVICE_H
extern "C"
{
	void SysTick_Handler(void)
	{
		time_service::systemTime++;
	}
}

namespace time_service
{
	volatile uint32_t systemTime = 0;
	
	void startTime(void)
	{
		SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	}
	
	void stopTime(void)
	{
		SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk);
	}
	
	void delay_ms(float delta)
	{
		uint32_t start = getCurTime();
		while(getCurTime() - start <= delta);
	}
	
	void init()
	{
		if(SysTick_Config(SystemCoreClock/1000))
		{
			while(true);
		}
	}
  
  uint32_t getCurTime(void)
  {
	 return systemTime;
	}
  
  uint32_t getCurTime_micros(void)
  {
	 return systemTime * 1000;
	}
  #endif
}
