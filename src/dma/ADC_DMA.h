//#include "adc.h"
//#include "dma.h"
//#include "pin_setup.h"
//#include "project_config.h"
//enum Adc_nums
//{
//  adc1,
//  adc2, 
//  adc3,
//};
//class ADC_Reader
//{
//  public:
//    ADC_Reader(uint8_t _adc, uint8_t _adc_channel,pin &signal)
//    {
//      _signal signal;
//      switch(_adc)
//      {
//        case 1: _ADCx = ADC1; _rcc_adc_periph = RCC_APB2Periph_ADC1; break;
//        case 2: _ADCx = ADC2; break;
//        case 3: _ADCx = ADC3; break;
//      }
//      my_adc(_ADCx, 1, _adc_channel, _rcc_adc_periph)
//    }
//  private:
//    Adc my_adc;
//    ADC_TypeDef* _ADCx;
//    uint32_t _rcc_adc_periph;
//    pin _signal;
//};