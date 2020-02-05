/*
 * LCD_display.h
 *
 *  Created on: 29Jan.,2020
 *      Author: jingchen
 */

#ifndef LCD_DISPLAY_H_
#define LCD_DISPLAY_H_


#include <stdint.h>
#include "DFR0529_LCD.h"

#define DISPLAY_ERR_PARAM -2

extern const uint8_t table_character_6_8[][6];
typedef int16_t (* pfCharacterFont_t) (uint8_t* pCh, uint8_t* pBuf, uint8_t* width, uint8_t* height);


void print1(const char str[]);
void Display(uint16_t width_, uint16_t height_);
void setCursor(int16_t x, int16_t y);
void setTextColor(uint16_t color);
void setTextBackground(uint16_t color);
void setTextSize(uint8_t size);//,uint16_t width,uint16_t height);
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num);
uint16_t getLineWidth();
void setLineWidth(uint16_t w);

#endif /* LCD_DISPLAY_H_ */

