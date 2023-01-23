#include "TSSP.h"

TSSP::TSSP(pin &write_1, pin &write_2, pin &write_3, pin &write_4,
                                          pin &read_1, pin &read_2):
_write_1(write_1), _write_2(write_2), _write_3(write_3), _write_4(write_4),
                                      _read_1(read_1), _read_2(read_2)
{ 
  for(int i = 0; i < 32; i++)
  {
    _data[i] = 0;
    _result[i] = 0;
  }
}

void TSSP::get_data()
{
  _x = 0;
  _y = 0;
  for(int j = 0; j < 10; j++) // getting data from sensors 10 times
  {
    for(int i = 0; i < 16; i++)
    {
      set_addres(i);
      _data[31 - i] += _read_1.read();
      _data[i] += _read_2.read();
    }
  }
  
  for(int i = 0; i < 32; i++)
  {
    // how much iterations with high signal 1 - 50% and more; 0 - less then 50%
    _result[i] = floor(double(_data[i] / 5)); 
    _x += _result[i] * sin(double(get_angle_from_index(i) / 57.3));
    _y += _result[i] * cos(double(get_angle_from_index(i) / 57.3));
  }
  // getting result angle
  _result_angle = int(atan2(double(_x), double(_y)) * 57.3) + 180;
  
  if(_result_angle < -180)
      _result_angle += 360;
  else if(_result_angle > 180)
      _result_angle -= 360;
  
  // getting result distance
  _result_distance = int(sqrt(pow(double(_x), 2) + pow(double(_y), 2)));
}

int16_t TSSP::get_angle()
{
  if(_result_angle < -180)
      _result_angle += 360;
  else if(_result_angle > 180)
      _result_angle -= 360;
    
  return _result_angle;
}

uint16_t TSSP::get_distance()
{
  return _result_distance;
}

void TSSP::set_addres(uint8_t _data)
{
  uint16_t channels[16][4] = {
                                {0,0,0,0},
                                {0,0,0,1},
                                {0,0,1,0},
                                {0,0,1,1},
                                {0,1,0,0},
                                {0,1,0,1},
                                {0,1,1,0},
                                {0,1,1,1},
                                {1,0,0,0},
                                {1,0,0,1},
                                {1,0,1,0},
                                {1,0,1,1},
                                {1,1,0,0},
                                {1,1,0,1},
                                {1,1,1,0},
                                {1,1,1,1}
                              };
  _write_1.write(channels[_data][0]);
  _write_2.write(channels[_data][1]);
  _write_3.write(channels[_data][2]);
  _write_4.write(channels[_data][3]);
                              
}

uint8_t TSSP::get_angle_from_index(uint16_t _num)
{
  _num *= 11.25;
  _num += 180;
  
  if(_num > 360)
    _num -= 360;
  return _num;
}