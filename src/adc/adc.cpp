#include "adc.h"

Adc::Adc(ADC_TypeDef* ADCx,
          uint8_t curChannel)//,          
				 //pin &sig):m_sig(sig)
{
	m_numberOfChannels = 1;
	m_ADCx = ADCx;
	m_curChannel = curChannel;
  if(ADCx == ADC1)
    m_RCC_APB2Periph_ADCx = RCC_APB2Periph_ADC1;
  else if(ADCx == ADC2)
    m_RCC_APB2Periph_ADCx = RCC_APB2Periph_ADC2;
  else if(ADCx == ADC3)
    m_RCC_APB2Periph_ADCx = RCC_APB2Periph_ADC3;
  cur = 1;
}

void Adc::adcInit(uint8_t _num_of_cycles)
{
	//m_sig.pinInit();
	RCC_APB2PeriphClockCmd(m_RCC_APB2Periph_ADCx, ENABLE);
	ADC_CommonInitTypeDef cADC;
	cADC.ADC_Mode = ADC_Mode_Independent;
	cADC.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;//!!!
  switch (_num_of_cycles)
  {
    case 5: cADC.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; break;
    case 10: cADC.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_10Cycles; break;
    case 15: cADC.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_15Cycles; break;
    case 20: cADC.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles; break;
    default: cADC.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; break;
  }
	cADC.ADC_Prescaler = ADC_Prescaler_Div2;
	
	ADC_CommonInit(&cADC);
}

void Adc::startAdc()
{
		ADC_InitTypeDef adc;
		adc.ADC_Resolution = ADC_Resolution_12b; 
		adc.ADC_ScanConvMode = ENABLE;
		adc.ADC_ContinuousConvMode = ENABLE;
		adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	  adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
		adc.ADC_DataAlign = ADC_DataAlign_Right;
		adc.ADC_NbrOfConversion = m_numberOfChannels;
		
		ADC_Init(m_ADCx, &adc);
		ADC_Cmd(m_ADCx, ENABLE);
		//ADC_SoftwareStartConv(m_ADCx);
}
void Adc::sendMeChannel(uint8_t chan)
{
    channel[cur] = chan;
    cur++;
}
void Adc::setChannel()
{
    for(uint8_t i = 1; i<=m_numberOfChannels; i++)
    {
	//ADC_EOCOnEachRegularChannelCmd(m_ADCx, ENABLE);
			ADC_RegularChannelConfig(m_ADCx, channel[i], i, ADC_SampleTime_56Cycles);
//	ADC_RegularChannelConfig(ADC3, , 2, ADC_SampleTime_56Cycles);
    }
}

ADC_TypeDef* Adc::getAdc()
{
	return m_ADCx;
}

uint8_t Adc::getChannel()
{
  return m_curChannel;
}

void Adc::adcDmaInit()
{
	ADC_DMARequestAfterLastTransferCmd(m_ADCx, ENABLE);
	ADC_EOCOnEachRegularChannelCmd(m_ADCx, ENABLE);
	ADC_DMACmd(m_ADCx, ENABLE);
	ADC_SoftwareStartConv(m_ADCx);
}
