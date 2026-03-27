/*
 * ili9341_GFX.h
 *
 *  Created on: Mar 6, 2026
 *      Author: mrkmo
 */

#ifndef ILI9341_GFX_H
#define ILI9341_GFX_H

#include <stdint.h>
#include "ili9341.h"
#include "fonts.h"

typedef struct {
    uint8_t width;
    uint8_t height;
    const uint16_t *data;
} FontDef;

void ILI9341_Set_Address(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void ILI9341_Draw_Pixel(uint16_t x, uint16_t y, uint16_t color);

void ILI9341_Fill_Screen(uint16_t color);

void ILI9341_Draw_Char(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color);

void ILI9341_Draw_Text(uint16_t x, uint16_t y, char *str, FontDef font, uint16_t color);

void ILI9341_Draw_Rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

void ILI9341_Draw_Rectangle_Filled(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

void MenuText(uint16_t menuVal);

void MenuBar (uint16_t stat*, uint16_t *oldStat);

void Battery (uint16_t batt*, uint16_t *oldBatt);

#endif
