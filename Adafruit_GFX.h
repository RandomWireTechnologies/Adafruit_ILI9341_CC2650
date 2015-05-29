#ifndef _ADAFRUIT_GFX_H
#define _ADAFRUIT_GFX_H

#include <stdint.h>

#define swap(a, b) { int16_t t = a; a = b; b = t; }


// These exist only with Adafruit_GFX (no subclass overrides)
void Adafruit_GFX_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void Adafruit_GFX_drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,uint16_t color);
void Adafruit_GFX_fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void Adafruit_GFX_fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,
   int16_t delta, uint16_t color);
void Adafruit_GFX_drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
   int16_t x2, int16_t y2, uint16_t color);
void Adafruit_GFX_fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
   int16_t x2, int16_t y2, uint16_t color);
void Adafruit_GFX_drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h,
   int16_t radius, uint16_t color);
void Adafruit_GFX_fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h,
   int16_t radius, uint16_t color);
void Adafruit_GFX_drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap,
   int16_t w, int16_t h, uint16_t color);
void Adafruit_GFX_drawBitmapBg(int16_t x, int16_t y, const uint8_t *bitmap,
   int16_t w, int16_t h, uint16_t color, uint16_t bg);
void Adafruit_GFX_drawXBitmap(int16_t x, int16_t y, const uint8_t *bitmap, 
   int16_t w, int16_t h, uint16_t color);
void Adafruit_GFX_drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color,
   uint16_t bg, uint8_t size);
void Adafruit_GFX_setCursor(int16_t x, int16_t y);
void Adafruit_GFX_setTextColor(uint16_t c);
void Adafruit_GFX_setTextColorBg(uint16_t c, uint16_t bg);
void Adafruit_GFX_setTextSize(uint8_t s);
void Adafruit_GFX_setTextWrap(uint8_t w);
void Adafruit_GFX_setRotation(uint8_t r);
void Adafruit_GFX_print(char *str);

  void initButton(int16_t x, int16_t y, 
		      uint8_t w, uint8_t h, 
		      uint16_t outline, uint16_t fill, uint16_t textcolor,
		      char *label, uint8_t textsize);
  void drawButton(uint8_t inverted);
  uint8_t contains(int16_t x, int16_t y);

  void press(uint8_t p);
  uint8_t isPressed();
  uint8_t justPressed();
  uint8_t justReleased();



#endif // _ADAFRUIT_GFX_H
