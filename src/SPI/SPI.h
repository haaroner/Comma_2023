#ifndef SPI_H
#define SPI_H

enum spi_mode
{
  master_mode,
  slave_mode
};

enum spi_work_mode
{
  simple,
  interruption
};

enum spi_num
{
  spi_1,
  spi_2,
  spi_3
};

enum direction
{
  fullDuplex,
  two_lines_RxOnly,
  RxOnly,
  TxOnly
  
};

enum spi_data_size
{
  d8b, //8 bit data
  d16b //16 bit data
};

enum spi_baud_rate
{
  b2r, 
  b4r,
  b8r,
  b16r,
  b32r,
  b64r,
  b128r,
  b256r
};

enum spi_first_bit
{
  msb,
  lsb
};

#endif

