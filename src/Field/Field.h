#include "project_config.h"
#include "OpenMV.h"

int16_t standart_points[8][2] = {
  {-40, 170},
  {-10, 185},
  {10, 185},
  {40, 170},
  {0, 110},
  {-40, 60},
  {40, 60},
  {0, 25}
};

class positioning
{
  public:
    positioning(camera &camera): _camera(camera) 
    {
      _robot_x = 0;
      _robot_y = 0;
      _moving_angle = 0;
      _moving_speed = 0;
      _distance = 0;
    }
    float calculate_distance(int16_t _x, int16_t _y)
    {
      return sqrt(double(_x * _x + _y * _y)); 
    }
    uint16_t calculate_component(uint16_t _hyp, int16_t _comp)
    {
      return sqrt(double(_hyp * _hyp - _comp * _comp));
    }
    int16_t calculate_angle(int16_t _x, int16_t _y)
    {
      return atan2f(_y, _x) * RAD2DEG;
    }
    void go_to_point(int16_t _x, int16_t _y, uint16_t start_speed, uint16_t min_speed)
    {
      _distance = calculate_distance(_robot_x - _x, _y - _robot_y);
      _moving_angle = calculate_angle(_robot_x - _x, _y - _robot_y);
      _k_speed = (start_speed - min_speed) / _distance;
      _moving_speed = _k_speed * _distance + min_speed;
    }
    void go_to_standart_point(uint8_t _index, uint16_t start_speed, uint16_t min_speed)
    {
      int16_t _x = standart_points[_index][0];
      int16_t _y = standart_points[_index][1];
      go_to_point(_x, _y, start_speed, min_speed);
    }
    int16_t get_angle()
    {
      return _moving_angle;
    }
    uint16_t get_speed()
    {
      return _moving_speed;
    }
    uint16_t get_distance()
    {
      return _distance;
    }
    bool check_position(int16_t _x, int16_t _y, uint8_t _accuracy)
    {
      return calculate_distance(_robot_x - _x, _robot_y - _y) <= _accuracy;
    }
  private:
    camera _camera;
    int16_t _robot_x, _robot_y;
    int16_t _moving_angle, _moving_speed;
    uint16_t _distance;
    float _k_speed;
};
