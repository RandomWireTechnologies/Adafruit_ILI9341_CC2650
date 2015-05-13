/***************************************************
  This is our library for the Adafruit ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "ili9341.h"



void ILI9341_writecommand(uint8_t c) {
  *dcport &=  ~dcpinmask;
  //digitalWrite(_dc, LOW);
  //*clkport &= ~clkpinmask; // clkport is a NULL pointer when hwSPI==true
  //digitalWrite(_sclk, LOW);
  *csport &= ~cspinmask;
  //digitalWrite(_cs, LOW);

  spiwrite(c);

  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
}


void ILI9341_writedata(uint8_t c) {
  *dcport |=  dcpinmask;
  //digitalWrite(_dc, HIGH);
  //*clkport &= ~clkpinmask; // clkport is a NULL pointer when hwSPI==true
  //digitalWrite(_sclk, LOW);
  *csport &= ~cspinmask;
  //digitalWrite(_cs, LOW);
  
  spiwrite(c);

  //digitalWrite(_cs, HIGH);
  *csport |= cspinmask;
} 



void ILI9341_setup(void) {
  // Turn on display

  ILI9341_writecommand(0xEF);
  ILI9341_writedata(0x03);
  ILI9341_writedata(0x80);
  ILI9341_writedata(0x02);

  ILI9341_writecommand(0xCF);  
  ILI9341_writedata(0x00); 
  ILI9341_writedata(0XC1); 
  ILI9341_writedata(0X30); 

  ILI9341_writecommand(0xED);  
  ILI9341_writedata(0x64); 
  ILI9341_writedata(0x03); 
  ILI9341_writedata(0X12); 
  ILI9341_writedata(0X81); 
 
  ILI9341_writecommand(0xE8);  
  ILI9341_writedata(0x85); 
  ILI9341_writedata(0x00); 
  ILI9341_writedata(0x78); 

  ILI9341_writecommand(0xCB);  
  ILI9341_writedata(0x39); 
  ILI9341_writedata(0x2C); 
  ILI9341_writedata(0x00); 
  ILI9341_writedata(0x34); 
  ILI9341_writedata(0x02); 
 
  ILI9341_writecommand(0xF7);  
  ILI9341_writedata(0x20); 

  ILI9341_writecommand(0xEA);  
  ILI9341_writedata(0x00); 
  ILI9341_writedata(0x00); 
 
  ILI9341_writecommand(ILI9341_PWCTR1);    //Power control 
  ILI9341_writedata(0x23);   //VRH[5:0] 
 
  ILI9341_writecommand(ILI9341_PWCTR2);    //Power control 
  ILI9341_writedata(0x10);   //SAP[2:0];BT[3:0] 
 
  ILI9341_writecommand(ILI9341_VMCTR1);    //VCM control 
  ILI9341_writedata(0x3e); //对比度调节
  ILI9341_writedata(0x28); 
  
  ILI9341_writecommand(ILI9341_VMCTR2);    //VCM control2 
  ILI9341_writedata(0x86);  //--
 
  ILI9341_writecommand(ILI9341_MADCTL);    // Memory Access Control 
  ILI9341_writedata(0x48);

  ILI9341_writecommand(ILI9341_PIXFMT);    
  ILI9341_writedata(0x55); 
  
  ILI9341_writecommand(ILI9341_FRMCTR1);    
  ILI9341_writedata(0x00);  
  ILI9341_writedata(0x18); 
 
  ILI9341_writecommand(ILI9341_DFUNCTR);    // Display Function Control 
  ILI9341_writedata(0x08); 
  ILI9341_writedata(0x82);
  ILI9341_writedata(0x27);  
 
  ILI9341_writecommand(0xF2);    // 3Gamma Function Disable 
  ILI9341_writedata(0x00); 
 
  ILI9341_writecommand(ILI9341_GAMMASET);    //Gamma curve selected 
  ILI9341_writedata(0x01); 
 
  ILI9341_writecommand(ILI9341_GMCTRP1);    //Set Gamma 
  ILI9341_writedata(0x0F); 
  ILI9341_writedata(0x31); 
  ILI9341_writedata(0x2B); 
  ILI9341_writedata(0x0C); 
  ILI9341_writedata(0x0E); 
  ILI9341_writedata(0x08); 
  ILI9341_writedata(0x4E); 
  ILI9341_writedata(0xF1); 
  ILI9341_writedata(0x37); 
  ILI9341_writedata(0x07); 
  ILI9341_writedata(0x10); 
  ILI9341_writedata(0x03); 
  ILI9341_writedata(0x0E); 
  ILI9341_writedata(0x09); 
  ILI9341_writedata(0x00); 
  
  ILI9341_writecommand(ILI9341_GMCTRN1);    //Set Gamma 
  ILI9341_writedata(0x00); 
  ILI9341_writedata(0x0E); 
  ILI9341_writedata(0x14); 
  ILI9341_writedata(0x03); 
  ILI9341_writedata(0x11); 
  ILI9341_writedata(0x07); 
  ILI9341_writedata(0x31); 
  ILI9341_writedata(0xC1); 
  ILI9341_writedata(0x48); 
  ILI9341_writedata(0x08); 
  ILI9341_writedata(0x0F); 
  ILI9341_writedata(0x0C); 
  ILI9341_writedata(0x31); 
  ILI9341_writedata(0x36); 
  ILI9341_writedata(0x0F); 

  ILI9341_writecommand(ILI9341_SLPOUT);    //Exit Sleep 
  delay(120); 		
  ILI9341_writecommand(ILI9341_DISPON);    //Display on 

}


void ILI9341_setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {

  ILI9341_writecommand(ILI9341_CASET); // Column addr set
  ILI9341_writedata(x0 >> 8);
  ILI9341_writedata(x0 & 0xFF);     // XSTART 
  ILI9341_writedata(x1 >> 8);
  ILI9341_writedata(x1 & 0xFF);     // XEND

  ILI9341_writecommand(ILI9341_PASET); // Row addr set
  ILI9341_writedata(y0>>8);
  ILI9341_writedata(y0);     // YSTART
  ILI9341_writedata(y1>>8);
  ILI9341_writedata(y1);     // YEND

  ILI9341_writecommand(ILI9341_RAMWR); // write to RAM
}


void ILI9341_pushColor(uint16_t color) {
  
  //digitalWrite(_dc, HIGH);
  *dcport |=  dcpinmask;
  //digitalWrite(_cs, LOW);
  *csport &= ~cspinmask;

  spiwrite(color >> 8);
  spiwrite(color);

  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
}

void ILI9341_drawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  setAddrWindow(x,y,x+1,y+1);

  //digitalWrite(_dc, HIGH);
  *dcport |=  dcpinmask;
  //digitalWrite(_cs, LOW);
  *csport &= ~cspinmask;

  spiwrite(color >> 8);
  spiwrite(color);

  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
}


void ILI9341_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;

  if((y+h-1) >= _height) 
    h = _height-y;

  setAddrWindow(x, y, x, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  *dcport |=  dcpinmask;
  //digitalWrite(_dc, HIGH);
  *csport &= ~cspinmask;
  //digitalWrite(_cs, LOW);

  while (h--) {
    spiwrite(hi);
    spiwrite(lo);
  }
  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
}


void ILI9341_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  setAddrWindow(x, y, x+w-1, y);

  uint8_t hi = color >> 8, lo = color;
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
  //digitalWrite(_dc, HIGH);
  //digitalWrite(_cs, LOW);
  while (w--) {
    spiwrite(hi);
    spiwrite(lo);
  }
  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
}

void ILI9341_fillScreen(uint16_t color) {
  fillRect(0, 0,  _width, _height, color);
}

// fill a rectangle
void Adafruit_ILI9341::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  *dcport |=  dcpinmask;
  //digitalWrite(_dc, HIGH);
  *csport &= ~cspinmask;
  //digitalWrite(_cs, LOW);

  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      spiwrite(hi);
      spiwrite(lo);
    }
  }
  //digitalWrite(_cs, HIGH);
  *csport |= cspinmask;
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t ILI9341_color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void ILI9341_setRotation(uint8_t m) {

  ILI9341_writecommand(ILI9341_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     ILI9341_writedata(MADCTL_MX | MADCTL_BGR);
     _width  = ILI9341_TFTWIDTH;
     _height = ILI9341_TFTHEIGHT;
     break;
   case 1:
     ILI9341_writedata(MADCTL_MV | MADCTL_BGR);
     _width  = ILI9341_TFTHEIGHT;
     _height = ILI9341_TFTWIDTH;
     break;
  case 2:
    ILI9341_writedata(MADCTL_MY | MADCTL_BGR);
     _width  = ILI9341_TFTWIDTH;
     _height = ILI9341_TFTHEIGHT;
    break;
   case 3:
     ILI9341_writedata(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
     _width  = ILI9341_TFTHEIGHT;
     _height = ILI9341_TFTWIDTH;
     break;
  }
}


void ILI9341_invertDisplay(uint8_t i) {
  ILI9341_writecommand(i ? ILI9341_INVON : ILI9341_INVOFF);
}

