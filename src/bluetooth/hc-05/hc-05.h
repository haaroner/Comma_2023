#include "project_config.h"
#include "usart2.h"
#include "usart3.h"
#include "usart6.h"

class hc_05
{
  public:
    hc_05(uint8_t uart_num)
    {
      _uart_num = uart_num;
    }
    void send(uint8_t data)
    {
      _crc_data[0] = data;
      if(_uart_num == uart_2)
      {
        usart2::write(255);
        usart2::write(data);
        //usart2::write(crc8(_crc_data, 1));
      }
      else if(_uart_num == uart_3)
      {
        usart3::write(255);
        usart3::write(data);
        //usart3::write(crc8(_crc_data, 1));
      }
      else if(_uart_num == uart_6)
      {
        usart6::write(255);
        usart6::write(data);
        //usart6::write(crc8(_crc_data, 1));
      }
    }
    uint8_t read()
    {
      if(_uart_num == uart_3)
      {
        if(usart3::available() > 1)
        {
          if(usart3::read() == 255)
            _data = usart3::read();
        }
        return _data;
      }
      
      if(_uart_num == uart_3)
      {
        if(usart3::available() > 1)
        {
          if(usart3::read() == 255)
            _data = usart3::read();
        }
        return _data;
      }
      
      if(_uart_num == uart_6)
      {
        if(usart6::available() > 1)
        {
          if(usart6::read() == 255)
            _data = usart6::read();
        }
        return _data;
      }
    }
    uint8_t available()
    {
      if(_uart_num == uart_2) return usart2::available();
      else if(_uart_num == uart_3) return usart3::available();
      else if(_uart_num == uart_6) return usart6::available();
      else return 0;
    }
  private:
    uint8_t crc8(uint8_t* data, int len)
    {
      uint8_t crc = 0xFF, i, j;
      for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
          if (crc & 0x80) crc = (char)((crc << 1) ^ 0x31);
           else crc <<= 1;
           }
      }
      return crc;
    }
    uint8_t _uart_num;
    uint8_t _data;
    uint8_t _crc_data[2];
};