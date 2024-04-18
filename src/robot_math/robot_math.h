#pragma once

#include "tools.h"
#include "project_config.h"
#define PI 3.1416
#define ec 2.7182

#define RAD2DEG	57.2957795130823208767
#define DEG2RAD	0.01745329251994329576

int ang_result = 0, len_result = 0, result = 0;
int x1 = 0, x2 = 0, y1 = 0, y2 = 0, _x_result = 0, _y_result = 0, x3, t;

double convert_data(double max, double data)
{
  uint16_t _data;
  _data = (data - max) / max + 1;
  
  if(_data > 1.0)
    _data = 1;
  else if(_data < 0.0)
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

int my_abs(int _num)
{
  return int(abs(double(_num)));
}

double my_pow(double _num, double _exp)
{
  return double(pow(double(_num), double(_exp)));
}

int my_sgn(int32_t num)
{
  if(num == 0)
    return 1;
  return my_abs(num) / num;
}

int sum_of_vectors(int16_t ang1, uint16_t len1, int16_t ang2, uint16_t len2)
{
  x1 = int(len1 * sin(double(ang1 * DEG2RAD)));
  y1 = int(len1 * cos(double(ang1 * DEG2RAD)));
          
  x2 = int(len2 * sin(double(ang2 * DEG2RAD)));
  y2 = int(len2 * cos(double(ang2 * DEG2RAD)));
          
  _x_result = x1 + x2;
  _y_result = y1 + y2;
  
  if(_x_result == 0) _x_result = 1;
  if(_y_result == 0) _y_result = 1;
          
  ang_result = int(atan2(double(_x_result), double(_y_result)) * RAD2DEG);
          
  len_result = int(sqrt(pow(double(_x_result), double(2)) + pow(double(_y_result), double(2))));
  
  return ang_result;
}

uint16_t get_len_from_sum_of_vectors()
{
  return len_result;
}

polar_vector get_angle_to_point(int16_t _robot_x, int16_t _robot_y, int16_t _point_x, int16_t _point_y)
{
  _x_result = _point_x - _robot_x;
  _y_result = _point_y - _robot_y;
  
  if(_x_result == 0) _x_result = 1;
  if(_y_result == 0) _y_result = 1;
  
  struct polar_vector result;
  result.angle = atan2(double(_x_result), double(_y_result)) * RAD2DEG;
  result.length = sqrt(pow(double(_x_result), 2) + pow(double(_y_result), 2));
  
  return result;
}

polar_vector get_angle_to_point(point point1, point point2)
{
  _x_result = point2.x - point1.x;
  _y_result = point2.y - point1.y;
  
  if(_x_result == 0) _x_result = 1;
  if(_y_result == 0) _y_result = 1;
  
  struct polar_vector result;
  result.angle = atan2(double(_x_result), double(_y_result)) * RAD2DEG;
  result.length = sqrt(pow(double(_x_result), 2) + pow(double(_y_result), 2));
  
  return result;
}

//int get_distance_to_point(int _a1, int _b1, int _a2, int _b2)
//{
//  _x_result = _a2 - _a1;
//  _y_result = _b2 - _b1;
//  
//  if(_x_result == 0) _x_result = 1;
//  if(_y_result == 0) _y_result = 1;

//}

void calculate_angle_near_side_out(int16_t robot_x, int16_t angle, double speed, int16_t x_min, int16_t x_max)
{
  speed = speed / 30.03;
  x1 = speed * sin(angle / 57.3);
  if(x1 > 0) x2 = (x_max - robot_x) / 100;
  else x2 = (robot_x - x_min) / 100;
  
  x3 = x1 - x2;
  y1 = speed * cos(angle / 57.3) * t;
  
  if (x3 > 0)
    x1 = x2;

  ang_result = atan2(double(x1), double(y1)) * 57.3;
  len_result = sqrt(double(x1 * x1 + y1 * y1));

}

int map(int num, int min1, int max1, int min2, int max2)
{
  int len1 = max1 - min1, len2 = max2 - min2;
  double k12 = len2 / len1;
  result = ((num - min1) * k12) + min2;
  return result;

  
  
}