#include "project_config.h"
#include "pin_setup.h"

class TSSP
{
  public:
    TSSP(pin &write_1, pin &write_2, pin &write_3, pin &write_4,
                                        pin &read_1, pin &read_2);
    void get_data();
    int16_t get_angle();
    uint16_t get_distance();
  private:
    void set_addres(uint8_t _data);
    uint8_t get_angle_from_index(uint16_t _num);
    pin _write_1, _write_2, _write_3, _write_4;
    pin _read_1, _read_2;
    uint8_t *_channels[16][4];
    uint16_t _data[32];
    double _x, _y;
    uint16_t _result[32];
    int16_t _result_angle;
    uint16_t _result_distance;
};