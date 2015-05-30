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

//
//#if ARDUINO >= 100
// #include "Arduino.h"
//#else
// #include "WProgram.h"
//#endif
//
//#include <Wire.h>

#include <Adafruit_FT6206.h>
#include "bsp_i2c.h"

// Private member variables from class become static variables.
static uint8_t touches = 0;
static uint16_t touchX[2];
static uint16_t touchY[2];
static uint16_t touchID[2];




/**************************************************************************/
/*! 
    @brief  Setups the HW
*/
/**************************************************************************/
uint8_t Adafruit_FT6206_begin(uint8_t threshhold) {
  Wire.begin();

  // change threshhold to be higher/lower
  Adafruit_FT6206_writeRegister8(FT6206_REG_THRESHHOLD, threshhold);
  
  if ((Adafruit_FT6206_readRegister8(FT6206_REG_VENDID) != 17) || (Adafruit_FT6206_readRegister8(FT6206_REG_CHIPID) != 6)) return false;
  /* 
  Serial.print("Vend ID: "); Serial.println(Adafruit_FT6206_readRegister8(FT6206_REG_VENDID));
  Serial.print("Chip ID: "); Serial.println(Adafruit_FT6206_readRegister8(FT6206_REG_CHIPID));
  Serial.print("Firm V: "); Serial.println(Adafruit_FT6206_readRegister8(FT6206_REG_FIRMVERS));
  Serial.print("Point Rate Hz: "); Serial.println(Adafruit_FT6206_readRegister8(FT6206_REG_POINTRATE));
  Serial.print("Thresh: "); Serial.println(Adafruit_FT6206_readRegister8(FT6206_REG_THRESHHOLD));
  */
  // dump all registers
  /*
  for (int16_t i=0; i<0x20; i++) {
    Serial.print("I2C $"); Serial.print(i, HEX);
    Serial.print(" = 0x"); Serial.println(Adafruit_FT6206_readRegister8(i), HEX);
  }
  */
  return true;
}

// DONT DO THIS - REALLY - IT DOESNT WORK
void Adafruit_FT6206_autoCalibrate(void) {
 Adafruit_FT6206_writeRegister8(FT6206_REG_MODE, FT6206_REG_FACTORYMODE);
 delay(100);
 //Serial.println("Calibrating...");
 Adafruit_FT6206_writeRegister8(FT6206_REG_CALIBRATE, 4);
 delay(300);
 for (uint8_t i = 0; i < 100; i++) {
   uint8_t temp;
   temp = Adafruit_FT6206_readRegister8(FT6206_REG_MODE);
   Serial.println(temp, HEX);
   //return to normal mode, calibration finish 
   if (0x0 == ((temp & 0x70) >> 4))
     break;
 }
 delay(200);
 //Serial.println("Calibrated");
 delay(300);
 Adafruit_FT6206_writeRegister8(FT6206_REG_MODE, FT6206_REG_FACTORYMODE);
 delay(100);
 Adafruit_FT6206_writeRegister8(FT6206_REG_CALIBRATE, 5);
 delay(300);
 Adafruit_FT6206_writeRegister8(FT6206_REG_MODE, FT6206_REG_WORKMODE);
 delay(300);
}


uint8_t Adafruit_FT6206_touched(void) {
  
  uint8_t n = Adafruit_FT6206_readRegister8(FT6206_REG_NUMTOUCHES);
  if ((n == 1) || (n == 2)) return true;
  return false;
}

/*****************************/

void Adafruit_FT6206_readData(uint16_t *x, uint16_t *y) {

    uint8_t i2cdat[16];
    uint8_t data = 0;
    if (bspI2cSelect(REGISTER_INTERFACE_I2C0,FT6206_ADDR)) {
        bspI2cWrite(data,1);
        bspI2cRead(i2cdat,16);
        bspI2cDeselect(); // Release I2C driver back to other tasks
    } else {
        *x = 0;
        *y = 0;
        return;
    }
  /*
  for (int16_t i=0; i<0x20; i++) {
    Serial.print("I2C $"); Serial.print(i, HEX); Serial.print(" = 0x"); Serial.println(i2cdat[i], HEX);
  }
  */

    touches = i2cdat[0x02];
    
    //Serial.println(touches);
    if (touches > 2) {
        touches = 0;
        *x = *y = 0;
    }
    if (touches == 0) {
        *x = *y = 0;
        return;
    }
    
    /*
    if (touches == 2) Serial.print('2');
    for (uint8_t i=0; i<16; i++) {
    // Serial.print("0x"); Serial.print(i2cdat[i], HEX); Serial.print(" ");
    }
    */
    
    /*
    Serial.println();
    if (i2cdat[0x01] != 0x00) {
    Serial.print("Gesture #"); 
    Serial.println(i2cdat[0x01]);
    }
    */
    
    //Serial.print("# Touches: "); Serial.print(touches);
    for (uint8_t i=0; i<2; i++) {
        touchX[i] = i2cdat[0x03 + i*6] & 0x0F;
        touchX[i] <<= 8;
        touchX[i] |= i2cdat[0x04 + i*6]; 
        touchY[i] = i2cdat[0x05 + i*6] & 0x0F;
        touchY[i] <<= 8;
        touchY[i] |= i2cdat[0x06 + i*6];
        touchID[i] = i2cdat[0x05 + i*6] >> 4;
    }
    /*
    Serial.println();
    for (uint8_t i=0; i<touches; i++) {
      Serial.print("ID #"); Serial.print(touchID[i]); Serial.print("\t("); Serial.print(touchX[i]);
      Serial.print(", "); Serial.print(touchY[i]);
      Serial.print (") ");
    }
    Serial.println();
    */
    *x = touchX[0]; *y = touchY[0];
}

TS_Point Adafruit_FT6206_getPoint(void) {
  uint16_t x, y;
  uint8_t z;
  readData(&x, &y);
  return TS_Point_Init(x, y, 1);
}


uint8_t Adafruit_FT6206_readRegister8(uint8_t reg) {
    uint8_t x ;
    // use i2c
    if (bspI2cSelect(REGISTER_INTERFACE_I2C0,FT6206_ADDR)) {
        bspI2cWrite(reg,1);
        bspI2cRead(x,1);
        bspI2cDeselect(); // Release I2C driver back to other tasks
    } else {
        return 0;
    }

    //  Serial.print("$"); Serial.print(reg, HEX); 
    //  Serial.print(": 0x"); Serial.println(x, HEX);
    
    return x;
}

void Adafruit_FT6206_writeRegister8(uint8_t reg, uint8_t val) {
    // use i2c
    uint8_t i2cData[2] = {reg,val};
    if (bspI2cSelect(REGISTER_INTERFACE_I2C0,FT6206_ADDR)) {
        bspI2cWrite(i2cData,2);
        bspI2cDeselect(); 
    }
}

/****************/

TS_Point TS_Point_Init(int16_t x0, int16_t y0, int16_t z0) {
  TS_Point point;
  point.x = x0;
  point.y = y0;
  point.z = z0;
  return point;
}

TS_Point* TS_Point_New(int16_t x0, int16_t y0, int16_t z0) {
  TS_Point* point = malloc(sizeof(TS_Point));
  point->x = x0;
  point->y = y0;
  point->z = z0;
  return point;
}

uint8_t TS_Point_equals(TS_Point *p1, TS_Point *p2) {
  return  ((p1->x == p2->x) && (p1->y == p2->y) && (p1->z == p2->z));
}

uint8_t TS_Point_notEqual(TS_Point *p1, TS_Point *p2) {
  return  ((p1->x != p2->x) || (p1->y != p2->y) || (p1->z != p2->z));
}
