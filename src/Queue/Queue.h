#include "robot_math.h"

class Queue
{
  public:
    Queue()
    {
      length = 20;
      for(int i = 0; i < length; i++)
        mas[i] = 0;
      counter = 0;
      _sub = 0;      
    }
    
    void push(int _num)
    {
      if(counter < length - 1)
      {
        mas[counter] = _num;
        counter++;
      }
    }
    
    int pop()
    {
      if(counter > 0)
      {
        _sub = mas[0];
        for(int i = 1; i < length - 1; i++)
          mas[i - 1] = mas[i];
        
        counter--;
      }
      return _sub;
    }
    
    int read(uint8_t _addr)
    {
      if(_addr < length)
        return mas[_addr];
    }
    
    int get_length()
    {
      return counter;
    }
    
    void clear()
    {
      for(int i = 0; i < length; i++)
        mas[i] = 0;
      counter = 0;
    }
    
    private:
      uint8_t length;
      volatile int mas[20], counter, _sub;
};