#include <stdlib.h>

#include "SPI3_DOT.h"
#include "SSD1306.h"


// the memory buffer for the LCD
uint8_t buffer[128*64] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 192, 240, 240, 56, 60, 60, 28, 28, 28, 28, 28, 24, 120, 112, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 48, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 192, 248, 252, 62, 14, 7, 3, 3, 3, 7, 7, 14, 254, 252, 240, 0, 0, 0, 255, 255, 255, 28, 6, 7, 3, 3, 7, 15, 254, 252, 252, 14, 6, 3, 3, 3, 7, 254, 254, 248, 0, 0, 0, 0, 255, 255, 255, 28, 6, 7, 3, 3, 7, 15, 254, 252, 252, 14, 6, 3, 3, 3, 7, 254, 254, 248, 0, 0, 0, 0, 6, 135, 135, 195, 195, 195, 195, 199, 207, 254, 252, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 15, 31, 62, 56, 112, 112, 112, 96, 96, 112, 112, 48, 56, 24, 0, 0, 1, 15, 31, 62, 112, 112, 96, 96, 96, 112, 112, 56, 31, 15, 7, 0, 0, 0, 127, 127, 127, 0, 0, 0, 0, 0, 0, 0, 127, 127, 127, 0, 0, 0, 0, 0, 0, 127, 127, 127, 0, 0, 0, 0, 127, 127, 127, 0, 0, 0, 0, 0, 0, 0, 127, 127, 127, 0, 0, 0, 0, 0, 0, 127, 127, 127, 0, 0, 0, 12, 63, 63, 115, 97, 96, 96, 96, 48, 56, 127, 127, 0, 0, 0, 0, 0, 128, 248, 248, 120, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 192, 192, 192, 0, 0, 0, 0, 0, 0, 0, 0, 192, 192, 192, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 7, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 48, 56, 120, 112, 112, 224, 225, 225, 193, 192, 192, 128, 128, 128, 128, 128, 192, 193, 193, 225, 224, 224, 112, 112, 120, 56, 48, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 3, 3, 3, 3, 3, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define ssd1306_swap(a, b) { int16_t t = a; a = b; b = t; }

// the most basic function, set a single pixel
void SSD1306::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((x < 0) || (x >= SSD1306_LCDWIDTH) || (y < 0) || (y >= SSD1306_LCDHEIGHT))
    return;

  // check rotation, move pixel around if necessary
  switch (rotation) {
  case 1:
    ssd1306_swap(x, y);
    x = SSD1306_LCDWIDTH - x - 1;
    break;
  case 2:
    x = SSD1306_LCDWIDTH - x - 1;
    y = SSD1306_LCDHEIGHT - y - 1;
    break;
  case 3:
    ssd1306_swap(x, y);
    y = SSD1306_LCDHEIGHT - y - 1;
    break;
	default:
		break;
  }

  // x is which column
    switch (color)
    {
      case WHITE:   buffer[x+ (y/8)*SSD1306_LCDWIDTH] |=  (1 << (y&7)); break;
      case BLACK:   buffer[x+ (y/8)*SSD1306_LCDWIDTH] &= ~(1 << (y&7)); break;
      case INVERSE: buffer[x+ (y/8)*SSD1306_LCDWIDTH] ^=  (1 << (y&7)); break;
    }

}


// constructor for hardware SPI - we indicate DataCommand, ChipSelect, Reset
SSD1306::SSD1306(unsigned int SPI_NUM, pin &dc, pin &rst, pin &cs, uint16_t PWR_EN, uint16_t SCK, uint16_t MOSI): _rst(rst),
                                                                                                                  _cs(cs),
                                                                                                                  _dc(dc)
{
	
	spiNum = SPI_NUM;
  //dc = DC;
  //rst = RST;
  //cs = CS;
	//sck = SCK;
	//mosi = MOSI;
  //hwSPI = true;
	//pwr_en = PWR_EN;
	
	
	//initPin(PWR_EN, OUTPUTPP);
	//setPin(PWR_EN, 1);
	
}

void SSD1306::draw()
{
	display();
}

void SSD1306::begin(uint8_t vccstate, bool reset) {
  _vccstate = vccstate;

	rotation = 0;
	
  // set pin directions
  //initPin(dc, OUTPUTPP);
  //initPin(cs, OUTPUTPP);

	//initSPI(spiNum, MASTER, 8, 2, cs, sck, -1, mosi);
  initSPI3(1, 8, 2);
  
  // Setup reset pin direction (used by both SPI and I2C)
  //initPin(rst, OUTPUTPP);
  
  //setPin(rst, HIGH);
  _rst.setBit();
  
  // VDD (3.3V) goes high at start, lets just chill for a ms
  time_service::delay_ms(1);
  // bring reset low
  _rst.resetBit();
  //setPin(rst, LOW);
  
  // wait 10ms
  time_service::delay_ms(10);
  
  // bring out of reset
 // setPin(rst, HIGH);
  _rst.setBit();
  // turn on VCC (9V?)
  

  // Init sequence
  ssd1306_command(SSD1306_DISPLAYOFF);                    // 0xAE
  ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
  ssd1306_command(0x80);                                  // the suggested ratio 0x80

  ssd1306_command(SSD1306_SETMULTIPLEX);                  // 0xA8
  ssd1306_command(SSD1306_LCDHEIGHT - 1);

  ssd1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
  ssd1306_command(0x0);                                   // no offset
  ssd1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
  ssd1306_command(SSD1306_CHARGEPUMP);                    // 0x8D
  if (vccstate == SSD1306_EXTERNALVCC)
    { ssd1306_command(0x10); }
  else
    { ssd1306_command(0x14); }
  ssd1306_command(SSD1306_MEMORYMODE);                    // 0x20
  ssd1306_command(0x00);                                  // 0x0 act like ks0108
  ssd1306_command(SSD1306_SEGREMAP | 0x1);
  ssd1306_command(SSD1306_COMSCANDEC);

 #if defined SSD1306_128_32
  ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
  ssd1306_command(0x02);
  ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
  ssd1306_command(0x8F);

#elif defined SSD1306_128_64
  ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
  ssd1306_command(0x12);
  ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
  if (vccstate == SSD1306_EXTERNALVCC)
    { ssd1306_command(0x9F); }
  else
    { ssd1306_command(0xCF); }

#elif defined SSD1306_96_16
  ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
  ssd1306_command(0x2);   //ada x12
  ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
  if (vccstate == SSD1306_EXTERNALVCC)
    { ssd1306_command(0x10); }
  else
    { ssd1306_command(0xAF); }

#endif

  ssd1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
  if (vccstate == SSD1306_EXTERNALVCC)
    { ssd1306_command(0x22); }
  else
    { ssd1306_command(0xF1); }
  ssd1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
  ssd1306_command(0x40);
  ssd1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
  ssd1306_command(SSD1306_NORMALDISPLAY);                 // 0xA6

  ssd1306_command(SSD1306_DEACTIVATE_SCROLL);

  ssd1306_command(SSD1306_DISPLAYON);//--turn on oled panel
}


void SSD1306::invert(uint8_t i) {
  if (i) {
    ssd1306_command(SSD1306_INVERTDISPLAY);
  } else {
    ssd1306_command(SSD1306_NORMALDISPLAY);
  }
}

void SSD1306::ssd1306_command(uint8_t c) {

  // SPI

//  setPin(cs, HIGH);
//  setPin(dc, LOW);
//  setPin(cs, LOW);

  _cs.setBit();
  _dc.resetBit();
  //time_service::delay_ms(1);//????
  for(int i = 0; i < 35; i++);  
  _cs.resetBit();
  
  fastSPIwrite(c);

  //setPin(cs, HIGH);
  _cs.setBit();
}

// startscrollright
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void SSD1306::startscrollright(uint8_t start, uint8_t stop){
  ssd1306_command(SSD1306_RIGHT_HORIZONTAL_SCROLL);
  ssd1306_command(0X00);
  ssd1306_command(start);
  ssd1306_command(0X00);
  ssd1306_command(stop);
  ssd1306_command(0X00);
  ssd1306_command(0XFF);
  ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrollleft
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void SSD1306::startscrollleft(uint8_t start, uint8_t stop){
  ssd1306_command(SSD1306_LEFT_HORIZONTAL_SCROLL);
  ssd1306_command(0X00);
  ssd1306_command(start);
  ssd1306_command(0X00);
  ssd1306_command(stop);
  ssd1306_command(0X00);
  ssd1306_command(0XFF);
  ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagright
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void SSD1306::startscrolldiagright(uint8_t start, uint8_t stop){
  ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
  ssd1306_command(0X00);
  ssd1306_command(SSD1306_LCDHEIGHT);
  ssd1306_command(SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
  ssd1306_command(0X00);
  ssd1306_command(start);
  ssd1306_command(0X00);
  ssd1306_command(stop);
  ssd1306_command(0X01);
  ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagleft
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void SSD1306::startscrolldiagleft(uint8_t start, uint8_t stop){
  ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
  ssd1306_command(0X00);
  ssd1306_command(SSD1306_LCDHEIGHT);
  ssd1306_command(SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
  ssd1306_command(0X00);
  ssd1306_command(start);
  ssd1306_command(0X00);
  ssd1306_command(stop);
  ssd1306_command(0X01);
  ssd1306_command(SSD1306_ACTIVATE_SCROLL);
}

void SSD1306::stopscroll(void){
  ssd1306_command(SSD1306_DEACTIVATE_SCROLL);
}

// Dim the display
// dim = true: display is dimmed
// dim = false: display is normal
void SSD1306::dim(bool dim) {
  uint8_t contrast;

  if (dim) {
    contrast = 0; // Dimmed display
  } else {
    if (_vccstate == SSD1306_EXTERNALVCC) {
      contrast = 0x9F;
    } else {
      contrast = 0xCF;
    }
  }
  // the range of contrast to too small to be really useful
  // it is useful to dim the display
  ssd1306_command(SSD1306_SETCONTRAST);
  ssd1306_command(contrast);
}

void SSD1306::display() {
  ssd1306_command(SSD1306_COLUMNADDR);
  ssd1306_command(0);   // Column start address (0 = reset)
  ssd1306_command(SSD1306_LCDWIDTH-1); // Column end address (127 = reset)

  ssd1306_command(SSD1306_PAGEADDR);
  ssd1306_command(0); // Page start address (0 = reset)
  #if SSD1306_LCDHEIGHT == 64
    ssd1306_command(7); // Page end address
  #endif
  #if SSD1306_LCDHEIGHT == 32
    ssd1306_command(3); // Page end address
  #endif
  #if SSD1306_LCDHEIGHT == 16
    ssd1306_command(1); // Page end address
  #endif


  // SPI

//  setPin(cs, HIGH);
//  setPin(dc, HIGH);
//  setPin(cs, LOW);

  _cs.setBit();
  _dc.setBit();
  _cs.resetBit();

	
  for (uint16_t i=0; i<(SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8); i++) {
    fastSPIwrite(buffer[i]);
  }

  //setPin(cs, HIGH);
  _cs.setBit();
}

void SSD1306::display(uint8_t *buf) {
  ssd1306_command(SSD1306_COLUMNADDR);
  ssd1306_command(0);   // Column start address (0 = reset)
  ssd1306_command(SSD1306_LCDWIDTH-1); // Column end address (127 = reset)

  ssd1306_command(SSD1306_PAGEADDR);
  ssd1306_command(0); // Page start address (0 = reset)
  #if SSD1306_LCDHEIGHT == 64
    ssd1306_command(7); // Page end address
  #endif
  #if SSD1306_LCDHEIGHT == 32
    ssd1306_command(3); // Page end address
  #endif
  #if SSD1306_LCDHEIGHT == 16
    ssd1306_command(1); // Page end address
  #endif


  // SPI

//  setPin(cs, HIGH);
//  setPin(dc, HIGH);
//  setPin(cs, LOW);

  _cs.setBit();
  _dc.setBit();
  _cs.resetBit();
	
  for (uint16_t i=0; i<(SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8); i++) {
    fastSPIwrite(buf[i]);
  }

  //setPin(cs, HIGH);
  _cs.setBit();
}

// clear everything
void SSD1306::clear(void) {
  memset(buffer, 0, (SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8));
}


inline void SSD1306::fastSPIwrite(uint8_t d) 
{
	writeSPI3(d);
  
}

void SSD1306::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
  bool bSwap = false;
  switch(rotation) {
    case 0:
      // 0 degree rotation, do nothing
      break;
    case 1:
      // 90 degree rotation, swap x & y for rotation, then invert x
      bSwap = true;
      ssd1306_swap(x, y);
      x = SSD1306_LCDWIDTH - x - 1;
      break;
    case 2:
      // 180 degree rotation, invert x and y - then shift y around for height.
      x = SSD1306_LCDWIDTH - x - 1;
      y = SSD1306_LCDHEIGHT - y - 1;
      x -= (w-1);
      break;
    case 3:
      // 270 degree rotation, swap x & y for rotation, then invert y  and adjust y for w (not to become h)
      bSwap = true;
      ssd1306_swap(x, y);
      y = SSD1306_LCDHEIGHT - y - 1;
      y -= (w-1);
      break;
  }

  if(bSwap) {
    drawFastVLineInternal(x, y, w, color);
  } else {
    drawFastHLineInternal(x, y, w, color);
  }
}

void SSD1306::drawFastHLineInternal(int16_t x, int16_t y, int16_t w, uint16_t color) {
  // Do bounds/limit checks
  if(y < 0 || y >= SSD1306_LCDHEIGHT) { return; }

  // make sure we don't try to draw below 0
  if(x < 0) {
    w += x;
    x = 0;
  }

  // make sure we don't go off the edge of the display
  if( (x + w) > SSD1306_LCDWIDTH) {
    w = (SSD1306_LCDWIDTH - x);
  }

  // if our width is now negative, punt
  if(w <= 0) { return; }

  // set up the pointer for  movement through the buffer
  uint8_t *pBuf = buffer;
  // adjust the buffer pointer for the current row
  pBuf += ((y/8) * SSD1306_LCDWIDTH);
  // and offset x columns in
  pBuf += x;

  uint8_t mask = 1 << (y&7);

  switch (color)
  {
  case WHITE:         while(w--) { *pBuf++ |= mask; }; break;
    case BLACK: mask = ~mask;   while(w--) { *pBuf++ &= mask; }; break;
  case INVERSE:         while(w--) { *pBuf++ ^= mask; }; break;
  }
}

void SSD1306::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
  bool bSwap = false;
  switch(rotation) {
    case 0:
      break;
    case 1:
      // 90 degree rotation, swap x & y for rotation, then invert x and adjust x for h (now to become w)
      bSwap = true;
      ssd1306_swap(x, y);
      x = SSD1306_LCDWIDTH - x - 1;
      x -= (h-1);
      break;
    case 2:
      // 180 degree rotation, invert x and y - then shift y around for height.
      x = SSD1306_LCDWIDTH - x - 1;
      y = SSD1306_LCDHEIGHT - y - 1;
      y -= (h-1);
      break;
    case 3:
      // 270 degree rotation, swap x & y for rotation, then invert y
      bSwap = true;
      ssd1306_swap(x, y);
      y = SSD1306_LCDHEIGHT - y - 1;
      break;
  }

  if(bSwap) {
    drawFastHLineInternal(x, y, h, color);
  } else {
    drawFastVLineInternal(x, y, h, color);
  }
}


//SSD1306::SSD1306(void)
//{
//}

void SSD1306::drawFastVLineInternal(int16_t x, int16_t __y, int16_t __h, uint16_t color) {

  // do nothing if we're off the left or right side of the screen
  if(x < 0 || x >= 128) { return; }

  // make sure we don't try to draw below 0
  if(__y < 0) {
    // __y is negative, this will subtract enough from __h to account for __y being 0
    __h += __y;
    __y = 0;

  }

  // make sure we don't go past the height of the display
  if( (__y + __h) > SSD1306_LCDHEIGHT) {
    __h = (SSD1306_LCDHEIGHT - __y);
  }

  // if our height is now negative, punt
  if(__h <= 0) {
    return;
  }

  // this display doesn't need ints for coordinates, use local byte registers for faster juggling
  uint8_t y = __y;
  uint8_t h = __h;


  // set up the pointer for fast movement through the buffer
  uint8_t *pBuf = buffer;
  // adjust the buffer pointer for the current row
  pBuf += ((y/8) * SSD1306_LCDWIDTH);
  // and offset x columns in
  pBuf += x;

  // do the first partial byte, if necessary - this requires some masking
  uint8_t mod = (y&7);
  if(mod) {
    // mask off the high n bits we want to set
    mod = 8-mod;

    // note - lookup table results in a nearly 10% performance improvement in fill* functions
    // register uint8_t mask = ~(0xFF >> (mod));
    static uint8_t premask[8] = {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };
    uint8_t mask = premask[mod];

    // adjust the mask if we're not going to reach the end of this byte
    if( h < mod) {
      mask &= (0XFF >> (mod-h));
    }

  switch (color)
    {
    case WHITE:   *pBuf |=  mask;  break;
    case BLACK:   *pBuf &= ~mask;  break;
    case INVERSE: *pBuf ^=  mask;  break;
    }

    // fast exit if we're done here!
    if(h<mod) { return; }

    h -= mod;

    pBuf += SSD1306_LCDWIDTH;
  }


  // write solid bytes while we can - effectively doing 8 rows at a time
  if(h >= 8) {
    if (color == INVERSE)  {          // separate copy of the code so we don't impact performance of the black/white write version with an extra comparison per loop
      do  {
      *pBuf=~(*pBuf);

        // adjust the buffer forward 8 rows worth of data
        pBuf += SSD1306_LCDWIDTH;

        // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
        h -= 8;
      } while(h >= 8);
      }
    else {
      // store a local value to work with
      uint8_t val = (color == WHITE) ? 255 : 0;

      do  {
        // write our value in
      *pBuf = val;

        // adjust the buffer forward 8 rows worth of data
        pBuf += SSD1306_LCDWIDTH;

        // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
        h -= 8;
      } while(h >= 8);
      }
    }

  // now do the final partial byte, if necessary
  if(h) {
    mod = h & 7;
    // this time we want to mask the low bits of the byte, vs the high bits we did above
    // register uint8_t mask = (1 << mod) - 1;
    // note - lookup table results in a nearly 10% performance improvement in fill* functions
    static uint8_t postmask[8] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };
    uint8_t mask = postmask[mod];
    switch (color)
    {
      case WHITE:   *pBuf |=  mask;  break;
      case BLACK:   *pBuf &= ~mask;  break;
      case INVERSE: *pBuf ^=  mask;  break;
    }
  }
}

void SSD1306::drawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
  if(x1 > x2)
  {
    sub = x1;
    x1 = x2;
    x2 = sub;
  }
  if(y1 > y2)
  {
    sub = y1;
    y1 = y2;
    y2 = sub;
  }
  dX = x2 - x1;
  //y2 *= -1;
  //y1 *= -1;
  dY = y2 - y1;
  alpha = atan2(double(dY), double(dX));
  if(my_abs(dX) >= my_abs(dY))
  {
    for(int i = 0; i < my_abs(dX); i++)
    {
      x3 = x2 - i * (dX / my_abs(dX));
      y3 = my_abs(y1 + x3 * tan(alpha));
      drawPixel(int(x3), int(y3), WHITE);
    }
  }
  else
  {
    for(int i = 0; i < my_abs(dY); i++)
    {
      y3 = my_abs(y1 + i * (dY / my_abs(dY)));//x3 * tan(alpha) * -1;
      x3 = x1 + y3 / tan(alpha);
      
      drawPixel(int(x3), int(y3), WHITE);
    }
  }
}