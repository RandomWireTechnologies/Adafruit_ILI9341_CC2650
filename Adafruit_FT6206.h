/*************************************************** 
  This is a library for the Adafruit Capacitive Touch Screens

  ----> http://www.adafruit.com/products/1947
 
  Check out the links above for our tutorials and wiring diagrams
  This chipset uses I2C to communicate

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#ifndef _ADAFRUIT_FT6206_H
#define _ADAFRUIT_FT6206_H


#define FT6206_ADDR           0x38
#define FT6206_G_FT5201ID     0xA8
#define FT6206_REG_NUMTOUCHES 0x02

#define FT6206_NUM_X             0x33
#define FT6206_NUM_Y             0x34

#define FT6206_REG_MODE 0x00
#define FT6206_REG_CALIBRATE 0x02
#define FT6206_REG_WORKMODE 0x00
#define FT6206_REG_FACTORYMODE 0x40
#define FT6206_REG_THRESHHOLD 0x80
#define FT6206_REG_POINTRATE 0x88
#define FT6206_REG_FIRMVERS 0xA6
#define FT6206_REG_CHIPID 0xA3
#define FT6206_REG_VENDID 0xA8

// calibrated for Adafruit 2.8" ctp screen
#define FT6206_DEFAULT_THRESSHOLD 128

struct stTS_Point {
  int16_t x;
  int16_t y;
  int16_t z;
};

typedef struct stTS_Point TS_Point;

TS_Point TS_Point_Init(int16_t x, int16_t y, int16_t z);
*TS_Point TS_Point_New(int16_t x, int16_t y, int16_t z);
uint8_t TS_Point_notEqual(TS_Point *p1, TS_Point *p2);
uint8_t TS_Point_equals(TS_Point *p1, TS_Point *p2);


uint8_t Adafruit_FT6206_begin(uint8_t thresh = FT6206_DEFAULT_THRESSHOLD);  

void Adafruit_FT6206_writeRegister8(uint8_t reg, uint8_t val);
uint8_t Adafruit_FT6206_readRegister8(uint8_t reg);

void Adafruit_FT6206_readData(uint16_t *x, uint16_t *y);
void Adafruit_FT6206_autoCalibrate(void); 

uint8_t Adafruit_FT6206_touched(void);
TS_Point Adafruit_FT6206_getPoint(void);

#endif // _ADAFRUIT_FT6206_H