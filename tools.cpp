#include "tools.h"

int lead_to_degree_borders(int _num)
{
  while(_num < -180 || _num > 180)
  {
    if(_num < -180)
        _num += 360;
    else if(_num > 180)
        _num -= 360;
  }
  return _num;
}

int lead_to_borders(int max, int min, int _num)
{
  while(_num < min || _num > max)
  {
    if(_num < min)
        _num += max - min;
    else if(_num > max)
        _num -= max - min;
  }
  return _num;
}

int constrain(int max, int min, int _num)
{
  if(_num > max) _num = max;
  else if(_num < min) _num = min;
  
  return _num;
}