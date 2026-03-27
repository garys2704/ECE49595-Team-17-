/*
 * ili9341_GFX.c
 *
 *  Created on: Mar 6, 2026
 *      Author: mrkmo
 */

#include "ili9341.h"
#include "ili9341_GFX.h"
#include "fonts.h"

#define TFT_WIDTH  240
#define TFT_HEIGHT 320

#define rx 0
#define ry 50
#define rh 200
#define rw 20

#define bx 0
#define by 60
#define bw 60
#define bh 20

/*
ILI9341_Fill_Screen(0x0000);

ILI9341_Draw_Text(0, 0, "Hello", Font_7x10, 0xFFFF);

*/


void ILI9341_Set_Address(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
	ili9341_WriteReg(0x2A);
	ili9341_WriteData(x1 >> 8);
	ili9341_WriteData(x1 & 0xFF);
	ili9341_WriteData(x2 >> 8);
	ili9341_WriteData(x2 & 0xFF);

	ili9341_WriteReg(0x2B);
	ili9341_WriteData(y1 >> 8);
	ili9341_WriteData(y1 & 0xFF);
	ili9341_WriteData(y2 >> 8);
	ili9341_WriteData(y2 & 0xFF);

	ili9341_WriteReg(0x2C);
}

void ILI9341_Draw_Pixel(uint16_t x, uint16_t y, uint16_t color) {
    ILI9341_Set_Address(x, y, x, y);
    ili9341_WriteData(color);

void ILI9341_Fill_Screen(uint16_t color) {
	ILI9341_Set_Address(0, 0, 239, 319);
	for(uint16_t i = 0; i < 240UL * 320UL; i++)
	{
		ili9341_WriteData(color);
	}
}


void ILI9341_Draw_Rectangle_Filled(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
	ILI9341_Set_Address(x, y, x + w - 1, y + h - 1);
	for(uint16_t i = 0; i < w * h; i++)
	{
		ili9341_WriteData(color);
	}
}

void ILI9341_Draw_Rectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
	ILI9341_Set_Address(x, y, x, y + h);
	for(uint16_t i = 0; i <= h; i++)
	{
		ili9341_WriteData(color);
	}

	ILI9341_Set_Address(x, y, x + w, y);
	for(uint16_t i = 0; i <= w; i++)
	{
		ili9341_WriteData(color);
	}

	ILI9341_Set_Address(x + w, y, x + w, y + h);
	for(uint16_t i = 0; i <= h; i++)
	{
		ili9341_WriteData(color);
	}

	ILI9341_Set_Address(x, y + h, x + w, y + h);
	for(uint16_t i = 0; i <= w; i++)
	{
		ili9341_WriteData(color);
	}
}

void ILI9341_Draw_Char(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color) {
	uint32_t i, b, j;

    const uint8_t *bitmap = &font.data[(ch - 32) * font.height];

    for (i = 0; i < font.height; i++) {
        b = bitmap[i];
        for (j = 0; j < font.width; j++) {
            if ((b << j) & 0x8000) {
                ILI9341_Draw_Pixel(x + j, y + i, color);
            }
        }
    }
}

void ILI9341_Draw_Text(uint16_t x, uint16_t y, char *str, FontDef font, uint16_t color) {
    while (*str) {
        ILI9341_Draw_Char(x, y, *str, font, color);
        x += font.width;
        str++;
    }
}

void ILI9341_Draw_Bar(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
	ILI9341_Draw_Rectangle_Filled(x, y, w, h, color);

	ILI9341_Draw_Rectangle(rx, ry, rw, rh, color);
}

void ILI9341_Draw_Battery(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
	ILI9341_Draw_Rectangle_Filled(x, y, w, h, color);

	ILI9341_Draw_Rectangle(bx, by, bw, bh, color);
}

void MenuText(int menuVal) {
    ILI9341_Draw_Rectangle_Filled(0, 0, TFT_WIDTH, 50, ILI9341_Black);

    if(menuVal == 0){
        ILI9341_Draw_Text(0, 0, "Volume", Font_11x18, ILI9341_White);
    }
    else if(menuVal == 1){
        ILI9341_Draw_Text(0, 0, "Sensitivity", Font_11x18, ILI9341_White);
    }
}

void MenuBar(int* stat) {
    int cx = TFT_WIDTH / 2;
    int base = TFT_HEIGHT - ry;

    if(*stat > oldStat){
        ILI9341_Draw_Rectangle_Filled(cx - rw/2, base - *stat, rw, *stat - oldStat, ILI9341_Green);
    }
    else {
        ILI9341_Draw_Rectangle_Filled(cx - rw/2, base - oldStat, rw, oldStat - *stat, ILI9341_Black);
    }

    ILI9341_Draw_Rectangle(cx - rw/2, base - rh, rw, rh, ILI9341_Green);

    oldStat = *stat;
}

void BatteryBar(int* batt) {
    int cx = TFT_WIDTH / 2 + TFT_WIDTH / 4;
    int base = cx - bw / 2;

    if(*batt > oldBatt){
        ILI9341_Draw_Rectangle_Filled(oldBatt - base, TFT_HEIGHT - by - bh, *batt - oldBatt, bh, ILI9341_Green);
    }
    else {
        ILI9341_Draw_Rectangle_Filled(*batt - base, TFT_HEIGHT - by - bh, oldBatt - *batt, bh, ILI9341_Black);
    }

    ILI9341_Draw_Rectangle(base, TFT_HEIGHT - by - bh, bw, bh, ILI9341_Green);

    oldBatt = *batt;
}

