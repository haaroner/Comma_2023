#pragma once
#include "project_config.h"



class pin
{
	public:
//		pin(GPIO_TypeDef* GPIOx,
//		uint32_t pinNumber,
//		GPIOMode_TypeDef modeGPIO,
//		GPIOSpeed_TypeDef speedGPIO,
//		GPIOOType_TypeDef typeGPIO,
//		GPIOPuPd_TypeDef pupdGpio,
//		uint16_t pinSourceGPIO,
//		uint8_t AFGPIO);
	pin(char pin_gpio, uint8_t pin_num, uint16_t function);
	void pinInit();
	void pwmInit(uint32_t RCC_TIMx,uint16_t _prescaler, uint32_t _period, uint32_t _start_pulse, uint8_t channel, TIM_TypeDef* TIMx);
	void pwm(uint32_t pulse);
	void setBit();
	void resetBit();
	GPIO_TypeDef* getGPIOx();
	uint16_t getPinNumber();
	private:
		GPIO_TypeDef* _GPIOx;
		uint32_t _pinNumber;
		GPIOMode_TypeDef _modeGPIO;
		GPIOSpeed_TypeDef _speedGPIO;
		GPIOOType_TypeDef _typeGPIO;
		GPIOPuPd_TypeDef _pupdGpio;
	uint16_t _pinSourceGPIO;
	uint8_t _AFGPIO;
	TIM_TypeDef* _TIMx;
	uint8_t _channel;
		
};
