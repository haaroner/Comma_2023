#include "robot_math.h"

class Queue
{
  public:
    Queue()
    {
      length = 20;
      for(int i = 0; i < length; i++)
      {
        masX[i] = 0;
        masY[i] = 0;
        masA[i] = 0;
        masSig[i] = 0;
      }
      counter = 0;
      _sub = 0; 
      _dist = 0;      
    }
    
    void push(point _point, uint8_t _significance)
    {
      if(counter < length - 1)
      {
        masX[counter] = _point.x;
        masY[counter] = _point.y;
        masA[counter] = _point.angle;
        masSig[counter] = constrain(2, 0, _significance);
        counter++;
      }
    }
    
    point pop()
    {
      point _sub_point;
      _sub_point.x = 0;
      _sub_point.y = 0;
      _sub_point.angle = 0;
      _sub_point.significanse = 0;
      if(counter > 0)
      {
        _sub_point.x = masX[0];
        _sub_point.y = masY[0];
        _sub_point.angle = masA[0];
        _sub_point.significanse = masSig[0];
        for(int i = 1; i < length - 1; i++)
        {
          masX[i - 1] = masX[i];
          masY[i - 1] = masY[i];
          masA[i - 1] = masA[i];
          masSig[i - 1] = masSig[i];
        }
        counter--;
      }
      return _sub_point;
    }
    
    int get_length()
    {
      return counter;
    }
    
    int calculate_distance(point _robot)
    {
        _dist = 0;
        _dist += get_angle_to_point(_robot.x, _robot.y, masX[0], masY[0]).length;
        
        if(counter >= 2)
        {
          for(int i = 0; i < counter - 1; i++)
          {
            _dist += get_angle_to_point(masX[i], masY[i] , masX[i+1], masY[i+1]).length;
          }
        }
        return _dist;
    }
    
    void clear()
    {
      for(int i = 0; i < length; i++)
      {
        masX[i] = 0;
        masY[i] = 0;
        masA[i] = 0;
        masSig[i] = 0;
      }
      counter = 0;
    }
    
    private:
      uint8_t length;
      volatile int masX[20], masY[20], masA[20], masSig[20], counter, _sub, _dist;
};
