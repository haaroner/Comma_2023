#include "project_config.h"
#define PI 3.1416
#define ec 2.7182

double convert_data(double max, double data)
{
  uint16_t _data;
  _data = (data - max) / max + 1;
  
  if(_data > 1)
    _data = 1;
  else if(_data < 0)
    _data = 0;
  
  return _data;
}

uint16_t fractional(uint16_t _data, double k1, double k2, double k3)
{
  _data = convert_data(100, _data);
  _data = k1 * pow(_data, pow(k2, k3));
  return _data;
}
  
uint16_t exponent(uint16_t _data, double k1, double k2)
{
  return k1 * pow(ec, double(_data) * k2);
}

int8_t sgn(int32_t num)
{
  return int(abs(double(num)) / num);
}