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
#include "board.h"
#include "bsp_spi.h"
#include "sensor.h"

static int16_t ILI9341_width = ILI9341_TFTWIDTH;
static int16_t ILI9341_height = ILI9341_TFTHEIGHT;
static uint8_t ILI9341_rotation = 0;
static PIN_Handle hGpio = 0;
static uint8_t spiBuffer[256] ={0};

void ILI9341_init(PIN_Handle gpio) {
    hGpio = gpio;
}


void ILI9341_writecommand(uint8_t c) {
    PIN_setOutputValue(hGpio, Board_LCD_DC, Board_LCD_DC_CMD);
    //digitalWrite(_dc, LOW);
    //*clkport &= ~clkpinmask; // clkport is a NULL pointer when hwSPI==true
    //digitalWrite(_sclk, LOW);
    PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_ON);
    //digitalWrite(_cs, LOW);
    
    bspSpiWrite(&c,1);
    
    PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_OFF);
    //digitalWrite(_cs, HIGH);
}


void ILI9341_writedata(uint8_t c) {
  PIN_setOutputValue(hGpio, Board_LCD_DC, Board_LCD_DC_DATA);
  //digitalWrite(_dc, HIGH);
  //*clkport &= ~clkpinmask; // clkport is a NULL pointer when hwSPI==true
  //digitalWrite(_sclk, LOW);
  PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_ON);
  //digitalWrite(_cs, LOW);
  
  bspSpiWrite(&c,1);

  //digitalWrite(_cs, HIGH);
  PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_OFF);
} 

void ILI9341_writeDataArray(uint8_t *buffer,uint8_t len) {
  PIN_setOutputValue(hGpio, Board_LCD_DC, Board_LCD_DC_DATA);
  //digitalWrite(_dc, HIGH);
  //*clkport &= ~clkpinmask; // clkport is a NULL pointer when hwSPI==true
  //digitalWrite(_sclk, LOW);
  PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_ON);
  //digitalWrite(_cs, LOW);
  
  bspSpiWrite(buffer,len);

  //digitalWrite(_cs, HIGH);
  PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_OFF);
} 

uint8_t ILI9341_readcommand8(uint8_t c, uint8_t index) {
   PIN_setOutputValue(hGpio, Board_LCD_DC, Board_LCD_DC_CMD);
   PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_ON);
   
   uint8_t writeData = 0xD9;
   bspSpiWrite(&writeData,1);
    
   PIN_setOutputValue(hGpio, Board_LCD_DC, Board_LCD_DC_DATA);
   bspSpiWrite(&writeData,1);
   PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_OFF);

   PIN_setOutputValue(hGpio, Board_LCD_DC, Board_LCD_DC_CMD);
   //digitalWrite(_sclk, LOW);
   PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_ON);
   bspSpiWrite(&c,1);
   
   PIN_setOutputValue(hGpio, Board_LCD_DC, Board_LCD_DC_DATA);
   uint8_t r=0;
   bspSpiRead(&r,1);
   PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_OFF);
   return r;
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


void ILI9341_setup(void) {
  bspSpiOpen();
  ILI9341_writecommand(0xEF);
  spiBuffer[0] = 0x03;
  spiBuffer[1] = 0x80;
  spiBuffer[2] = 0x02;
  ILI9341_writeDataArray(spiBuffer,3);

  ILI9341_writecommand(0xCF);
  spiBuffer[0] = 0x00; 
  spiBuffer[1] = 0XC1; 
  spiBuffer[2] = 0X30;
  ILI9341_writeDataArray(spiBuffer,3);

  ILI9341_writecommand(0xED);  
  spiBuffer[0] = 0x64; 
  spiBuffer[1] = 0x03; 
  spiBuffer[2] = 0X12; 
  spiBuffer[3] = 0X81; 
  ILI9341_writeDataArray(spiBuffer,4);
 
  ILI9341_writecommand(0xE8);  
  spiBuffer[0] = 0x85; 
  spiBuffer[1] = 0x00; 
  spiBuffer[2] = 0x78;
  ILI9341_writeDataArray(spiBuffer,3);

  ILI9341_writecommand(0xCB);  
  spiBuffer[0] = 0x39; 
  spiBuffer[1] = 0x2C; 
  spiBuffer[2] = 0x00; 
  spiBuffer[3] = 0x34; 
  spiBuffer[4] = 0x02;
  ILI9341_writeDataArray(spiBuffer,5);
 
  ILI9341_writecommand(0xF7);  
  spiBuffer[0] = 0x20;
  ILI9341_writeDataArray(spiBuffer,1);

  ILI9341_writecommand(0xEA);  
  spiBuffer[0] = 0x00; 
  spiBuffer[1] = 0x00;
  ILI9341_writeDataArray(spiBuffer,2);
 
  ILI9341_writecommand(ILI9341_PWCTR1);    //Power control 
  spiBuffer[0] = 0x23;   //VRH[5:0] 
  ILI9341_writeDataArray(spiBuffer,1);
 
  ILI9341_writecommand(ILI9341_PWCTR2);    //Power control 
  spiBuffer[0] = 0x10;   //SAP[2:0];BT[3:0] 
  ILI9341_writeDataArray(spiBuffer,1);
 
  ILI9341_writecommand(ILI9341_VMCTR1);    //VCM control 
  spiBuffer[0] = 0x3e; //对比度调节
  spiBuffer[1] = 0x28; 
  ILI9341_writeDataArray(spiBuffer,2);
  
  ILI9341_writecommand(ILI9341_VMCTR2);    //VCM control2 
  spiBuffer[0] = 0x86;  //--
  ILI9341_writeDataArray(spiBuffer,1);
 
  ILI9341_writecommand(ILI9341_MADCTL);    // Memory Access Control 
  spiBuffer[0] = 0x48;
  ILI9341_writeDataArray(spiBuffer,1);

  ILI9341_writecommand(ILI9341_PIXFMT);    
  spiBuffer[0] = 0x55; 
  ILI9341_writeDataArray(spiBuffer,1);
  
  ILI9341_writecommand(ILI9341_FRMCTR1);    
  spiBuffer[0] = 0x00;  
  spiBuffer[1] = 0x18; 
  ILI9341_writeDataArray(spiBuffer,2);
 
  ILI9341_writecommand(ILI9341_DFUNCTR);    // Display Function Control 
  spiBuffer[0] = 0x08; 
  spiBuffer[1] = 0x82;
  spiBuffer[2] = 0x27;  
  ILI9341_writeDataArray(spiBuffer,3);
 
  ILI9341_writecommand(0xF2);    // 3Gamma Function Disable 
  spiBuffer[0] = 0x00; 
  ILI9341_writeDataArray(spiBuffer,1);
 
  ILI9341_writecommand(ILI9341_GAMMASET);    //Gamma curve selected 
  spiBuffer[0] = 0x01; 
  ILI9341_writeDataArray(spiBuffer,1);
 
  ILI9341_writecommand(ILI9341_GMCTRP1);    //Set Gamma 
  spiBuffer[0 ] = 0x0F; 
  spiBuffer[1 ] = 0x31; 
  spiBuffer[2 ] = 0x2B; 
  spiBuffer[3 ] = 0x0C; 
  spiBuffer[4 ] = 0x0E; 
  spiBuffer[5 ] = 0x08; 
  spiBuffer[6 ] = 0x4E; 
  spiBuffer[7 ] = 0xF1; 
  spiBuffer[8 ] = 0x37; 
  spiBuffer[9 ] = 0x07; 
  spiBuffer[10] = 0x10; 
  spiBuffer[11] = 0x03; 
  spiBuffer[12] = 0x0E; 
  spiBuffer[13] = 0x09; 
  spiBuffer[14] = 0x00; 
  ILI9341_writeDataArray(spiBuffer,15);
  
  ILI9341_writecommand(ILI9341_GMCTRN1);    //Set Gamma 
  spiBuffer[0 ] = 0x00; 
  spiBuffer[1 ] = 0x0E; 
  spiBuffer[2 ] = 0x14; 
  spiBuffer[3 ] = 0x03; 
  spiBuffer[4 ] = 0x11; 
  spiBuffer[5 ] = 0x07; 
  spiBuffer[6 ] = 0x31; 
  spiBuffer[7 ] = 0xC1; 
  spiBuffer[8 ] = 0x48; 
  spiBuffer[9 ] = 0x08; 
  spiBuffer[10] = 0x0F; 
  spiBuffer[11] = 0x0C; 
  spiBuffer[12] = 0x31; 
  spiBuffer[13] = 0x36; 
  spiBuffer[14] = 0x0F; 
  ILI9341_writeDataArray(spiBuffer,15);

  ILI9341_writecommand(ILI9341_SLPOUT);    //Exit Sleep 
  delay_ms(120); 		
  ILI9341_writecommand(ILI9341_DISPON);    //Display on 

}

//void ILI9341_setup(void) {
//  // Turn on display
//  PIN_setOutputValue(hGpio, Board_LCD_PWR, Board_LCD_PWR_ON);
//  delay_ms(100);
//  ILI9341_writecommand(ILI9341_DISPOFF);
// 
//  ILI9341_writecommand(ILI9341_PWCTR1);    //Power control 
//  ILI9341_writedata(0x23);   //VRH[5:0] 
// 
//  ILI9341_writecommand(ILI9341_PWCTR2);    //Power control 
//  ILI9341_writedata(0x10);   //SAP[2:0];BT[3:0] 
// 
//  ILI9341_writecommand(ILI9341_VMCTR1);    //VCM control 
//  ILI9341_writedata(0x2B);
//  ILI9341_writedata(0x2B); 
//  
//  ILI9341_writecommand(ILI9341_VMCTR2);    //VCM control2 
//  ILI9341_writedata(0xC0);  //--
// 
//  ILI9341_writecommand(ILI9341_MADCTL);    // Memory Access Control 
//  ILI9341_writedata(0x88);
//
//  ILI9341_writecommand(ILI9341_PIXFMT);    
//  ILI9341_writedata(0x55); 
//  
//  ILI9341_writecommand(ILI9341_FRMCTR1);    
//  ILI9341_writedata(0x00);  
//  ILI9341_writedata(0x1B); 
// 
//  ILI9341_writecommand(ILI9341_ENTRYMODE); // 
//  ILI9341_writedata(0x07);
//  
//  ILI9341_writecommand(ILI9341_SLPOUT);    //Exit Sleep 
//  delay_ms(150); 		
//  ILI9341_writecommand(ILI9341_DISPON);    //Display on 
//  delay_ms(500);
//  ILI9341_setAddrWindow(0,0,ILI9341_TFTWIDTH-1,ILI9341_TFTHEIGHT-1);
//}

void ILI9341_drawFontBitmap(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint16_t bg_color, uint8_t *bitmap) {
    // Rudimentary clipping
    if(((x+w-1) >= ILI9341_width) || ((y+h-1) >= ILI9341_height)) return;
    
    ILI9341_setAddrWindow(x,y,x+w-1,y+h-1);
    
    uint8_t colors[2] = {color >> 8,color};
    uint8_t bg_colors[2] = {bg_color >> 8, bg_color};
    uint16_t length=w*h;
    uint8_t byte_width = w/8;
    uint8_t left_over_bits = w%8;
    if (left_over_bits) {
        // If bits don't fit evenly into bytes
        byte_width++;
    }
    uint8_t bit_pointer = 7;

     //digitalWrite(_dc, HIGH);
    PIN_setOutputValue(hGpio, Board_LCD_DC, Board_LCD_DC_DATA);
    //digitalWrite(_cs, LOW);
    PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_ON);
    
    for (uint16_t y=0;y<h;y++) {
        for (uint16_t x=0;x<byte_width;x++) {
            for(int8_t bit=0;bit<8;bit++) {
                if ((x < (byte_width-1)) || (bit<left_over_bits)) {
                    if (*bitmap & (1 << (7-bit))) {
                        bspSpiWrite(colors,2);
                    } else {
                        bspSpiWrite(bg_colors,2);
                    }
                }
            }
            bitmap++;            
        }
    }
    
    PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_OFF);
}


void ILI9341_pushColor(uint16_t color) {
    
    //digitalWrite(_dc, HIGH);
    PIN_setOutputValue(hGpio, Board_LCD_DC, Board_LCD_DC_DATA);
    //digitalWrite(_cs, LOW);
    PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_ON);

    uint8_t colors[2] = {color >> 8,color};
    bspSpiWrite(colors,2);
    
    PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_OFF);
    //digitalWrite(_cs, HIGH);
}

void ILI9341_drawPixel(int16_t x, int16_t y, uint16_t color) {

    if((x < 0) ||(x >= ILI9341_width) || (y < 0) || (y >= ILI9341_height)) return;
    
    ILI9341_setAddrWindow(x,y,x+1,y+1);
    
    //digitalWrite(_dc, HIGH);
    PIN_setOutputValue(hGpio, Board_LCD_DC, Board_LCD_DC_DATA);
    //digitalWrite(_cs, LOW);
    PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_ON);
    
    uint8_t colors[2] = {color >> 8,color};
    bspSpiWrite(colors,2);
    
    PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_OFF);
    //digitalWrite(_cs, HIGH);
}


void ILI9341_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {

  // Rudimentary clipping
  if((x >= ILI9341_width) || (y >= ILI9341_height)) return;

  if((y+h-1) >= ILI9341_height) 
    h = ILI9341_height-y;

  ILI9341_setAddrWindow(x, y, x, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  PIN_setOutputValue(hGpio, Board_LCD_DC, Board_LCD_DC_DATA);
  //digitalWrite(_dc, HIGH);
  PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_ON);
  //digitalWrite(_cs, LOW);

  while (h--) {
    bspSpiWrite(&hi,1);
    bspSpiWrite(&lo,1);
  }
  PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_OFF);
  //digitalWrite(_cs, HIGH);
}


void ILI9341_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {

  // Rudimentary clipping
  if((x >= ILI9341_width) || (y >= ILI9341_height)) return;
  if((x+w-1) >= ILI9341_width)  w = ILI9341_width-x;
  ILI9341_setAddrWindow(x, y, x+w-1, y);

  uint8_t hi = color >> 8, lo = color;
  PIN_setOutputValue(hGpio, Board_LCD_DC, Board_LCD_DC_DATA);
  PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_ON);
  //digitalWrite(_dc, HIGH);
  //digitalWrite(_cs, LOW);
  while (w--) {
    bspSpiWrite(&hi,1);
    bspSpiWrite(&lo,1);
  }
  PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_OFF);
  //digitalWrite(_cs, HIGH);
}


// fill a rectangle
void ILI9341_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= ILI9341_width) || (y >= ILI9341_height)) return;
  if((x + w - 1) >= ILI9341_width)  w = ILI9341_width  - x;
  if((y + h - 1) >= ILI9341_height) h = ILI9341_height - y;

  ILI9341_setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  PIN_setOutputValue(hGpio, Board_LCD_DC, Board_LCD_DC_DATA);
  //digitalWrite(_dc, HIGH);
  PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_ON);
  //digitalWrite(_cs, LOW);
  uint8_t count = 0;
  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
//      bspSpiWrite(&hi,1);
//      bspSpiWrite(&lo,1);
        spiBuffer[count++] = hi;
        spiBuffer[count++] = lo;
        if (count == 0) {
            bspSpiWrite(spiBuffer,256);
        }
//        spiBuffer[0] = hi;
//        spiBuffer[1] = lo;
//        bspSpiWrite(spiBuffer,2);
    }
  }
  if (count != 0) {
    bspSpiWrite(spiBuffer,count);
  }
  //digitalWrite(_cs, HIGH);
  PIN_setOutputValue(hGpio, Board_LCD_CS, Board_LCD_CS_OFF);
}

void ILI9341_fillScreen(uint16_t color) {
    ILI9341_fillRect(0, 0,  ILI9341_width, ILI9341_height, color);
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
  ILI9341_rotation = m % 4; // can't be higher than 3
  switch (ILI9341_rotation) {
   case 0:
     ILI9341_writedata(MADCTL_MX | MADCTL_BGR);
     ILI9341_width  = ILI9341_TFTWIDTH;
     ILI9341_height = ILI9341_TFTHEIGHT;
     break;
   case 1:
     ILI9341_writedata(MADCTL_MV | MADCTL_BGR);
     ILI9341_width  = ILI9341_TFTHEIGHT;
     ILI9341_height = ILI9341_TFTWIDTH;
     break;
  case 2:
    ILI9341_writedata(MADCTL_MY | MADCTL_BGR);
     ILI9341_width  = ILI9341_TFTWIDTH;
     ILI9341_height = ILI9341_TFTHEIGHT;
    break;
   case 3:
     ILI9341_writedata(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
     ILI9341_width  = ILI9341_TFTHEIGHT;
     ILI9341_height = ILI9341_TFTWIDTH;
     break;
  }
}


void ILI9341_invertDisplay(uint8_t i) {
  ILI9341_writecommand(i ? ILI9341_INVON : ILI9341_INVOFF);
}

