#pragma once

#include "project_config.h"
#include "pin_setup.h"
#include "SPI.h"

namespace Spi_3
{
    void spi_init(int mode, int work_mode, int direction, int dataSize, int baudRate, int firstBit);
    void spi_deinit();
    void send(uint16_t data);
    uint16_t read();
    uint8_t available();
    extern SPI_TypeDef *_SPIx;
    extern int _spi_mode;
    extern int _spi_work_mode;
    extern int _spi_num;
    extern int _spi_direction;
    extern int _data_size;
    extern int _baud_rate;
    extern int _first_bit;
    extern uint16_t _data;
    extern uint16_t _rx_buffer[8];
    extern uint16_t _cur_rx_read;
    extern uint16_t _cur_rx_write;
    extern uint16_t _buf_size;
};
