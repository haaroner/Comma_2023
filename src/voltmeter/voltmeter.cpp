#include "voltmeter.h"

voltmeter::voltmeter(Dma &dma_meter):_dma_meter(dma_meter)
{
  _kv = 0.003538695; //4.385 * 0.000807;
  _data = 12.6;
};

float voltmeter::get_voltage(uint16_t _data)
{
 return float(_kv * _data);
}